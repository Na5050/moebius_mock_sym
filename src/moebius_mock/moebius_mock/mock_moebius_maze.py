import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf2_ros
import os
import math
import numpy as np
import cv2
from PIL import Image as PILImage

from panda3d.core import loadPrcFileData, DirectionalLight, Vec4, Texture, CardMaker, GraphicsOutput, FrameBufferProperties, WindowProperties, GraphicsPipe
# Автовыбор GPU/CPU, скрытый офскрин-режим, отключение окон, теней, звука и устройств ввода
loadPrcFileData("", "window-type offscreen")
loadPrcFileData("", "audio-library none")
loadPrcFileData("", "shadow-depth-bits 0")
loadPrcFileData("", "framebuffer-multisample 0")
loadPrcFileData("", "multisamples 0")
loadPrcFileData("", "support-keyboard 0")
loadPrcFileData("", "support-mouse 0")
loadPrcFileData("", "notify-level fatal")
loadPrcFileData("", "notify-level-device error")

from direct.showbase.ShowBase import ShowBase
from cv_bridge import CvBridge
# --- ГЛОБАЛЬНЫЕ КОНСТАНТЫ СИМУЛЯТОРА ---
ARUCO_BOX_THICKNESS = 0.05  # Толщина маркерной коробки: ровно 5 см
class PandaRenderer(ShowBase):
    # Разрешение подложки пола: 512 пикселей на 1 метр лабиринта (1 пиксель = 2 мм)
    SCALE_M_TO_PX = 512.0

    def __init__(self, cam_w, cam_h, walls, sign_zones, maze_height, maze_width, line_points, aruco_markers, sign_w, sign_h, aruco_w, aruco_h):
        # 1. ХАК ДЛЯ СИСТЕМНЫХ ПУТЕЙ PANDA3D (Принудительно подтягиваем встроенные модели)
        import os
        import panda3d
        from panda3d.core import getModelPath
        
        panda_dir = os.path.dirname(panda3d.__file__)
        models_dir = os.path.join(panda_dir, "models")
        
        # Загружаем ShowBase. Движок инициализирует конвейер
        ShowBase.__init__(self)
        
        if os.path.exists(models_dir):
            getModelPath().appendPath(models_dir)

        self.cam_width = cam_w
        self.cam_height = cam_h
        
        # Настройка 4-канального offscreen-буфера в оперативной памяти (RGBA)
        fb_props = FrameBufferProperties.getDefault()
        win_props = WindowProperties.size(self.cam_width, self.cam_height)

        self.buffer = self.graphicsEngine.makeOutput(
            self.pipe, "cpu_offscreen_buffer", 0, fb_props, win_props,
            GraphicsPipe.BF_refuse_window | GraphicsPipe.BF_fb_props_optional
        )

        self.tex = Texture()
        self.buffer.addRenderTexture(self.tex, GraphicsOutput.RTM_copy_ram, GraphicsOutput.RTP_color)

        dr = self.buffer.makeDisplayRegion()
        dr.setCamera(self.cam)

        # Настройка линзы: FOV 120° и ближняя плоскость 1 см, чтобы пол не резался
        self.cam.node().getLens().setFov(120.0)
        self.cam.node().getLens().setAspectRatio(self.cam_width / self.cam_height)
        self.cam.node().getLens().setNearFar(0.01, 100.0)

        # Контрастное освещение сцены (как в Gazebo Classic)
        dlight = DirectionalLight('sim_sun')
        dlight.setColor(Vec4(1.0, 1.0, 1.0, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(0, -75, 0)
        self.render.setLight(dlnp)

        from panda3d.core import AmbientLight
        alight = AmbientLight('sim_ambient')
        alight.setColor(Vec4(0.4, 0.4, 0.4, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

        # --- 1. ГЕНЕРАЦИЯ СВЕТЛО-СЕРОГО ПОЛА С АНТИ-РАЗМЫТИЕМ ПОПЕРЕЧНЫХ ЛИНИЙ ---
        scale_m_to_px = self.SCALE_M_TO_PX
        floor_w_meters = float(maze_width) + 2.0
        floor_h_meters = float(maze_height) + 2.0
        floor_px_w = int(floor_w_meters * scale_m_to_px)
        floor_px_h = int(floor_h_meters * scale_m_to_px)
        floor_img = np.ones((floor_px_h, floor_px_w, 3), dtype=np.uint8) * 220 

        # Отрисовка черной линии трассы толщиной 5 см (смещение +1 метр учтено)
        if len(line_points) >= 6:
            line_thickness = max(1, int(0.05 * scale_m_to_px))
            pts = np.array(line_points).reshape(-1, 2)
            px_pts = ((pts + 1.0) * scale_m_to_px).astype(np.int32)
            cv2.polylines(floor_img, [px_pts], isClosed=False, color=(0, 0, 0), thickness=line_thickness)

        pil_img = PILImage.fromarray(cv2.cvtColor(floor_img, cv2.COLOR_BGR2RGB))
        p3d_tex = Texture()
        p3d_tex.setup2dTexture(floor_px_w, floor_px_h, Texture.T_unsigned_byte, Texture.F_rgb)
        p3d_tex.setRamImage(pil_img.tobytes())

        # ЖЕЛЕЗНОЕ ИСПРАВЛЕНИЕ ЧЕТКОСТИ:
        # Отключаем мипмаппинг (сглаживание), который превращал тонкие поперечные линии в "мыло" на CPU.
        # FT_nearest заставляет процессор рендерить пиксели текстуры без интерполяции и размытия.
        p3d_tex.setMinfilter(Texture.FT_nearest)
        p3d_tex.setMagfilter(Texture.FT_nearest)
        
        # Включаем максимальную анизотропную фильтрацию (16x) для удержания детализации под острым углом
        p3d_tex.setAnisotropicDegree(16)

        cm = CardMaker('floor_card')
        cm.setFrame(-1.0, floor_w_meters - 1.0, -1.0, floor_h_meters - 1.0)
        floor_node = self.render.attachNewNode(cm.generate())
        floor_node.setTexture(p3d_tex)
        floor_node.setP(-90)
        floor_node.setZ(0.0) 

        # --- 2. РЕНДЕРИНГ ТОЛСТЫХ ОБЪЕМНЫХ КУБОВ СТЕН 1х1х1 МЕТР ---
        # --- 2. ДИНАМИЧЕСКИЙ РЕНДЕРИНГ ВСЕХ СТЕН И МАРКЕРНЫХ КОРОБОК ИЗ MASSIBA WALLS ---
        for wall in walls:
            w_x1, w_x2, w_y1, w_y2 = wall
            
            # Вычисляем честные физические размеры преграды, запеченной в walls
            # (Для стен лабиринта тут будет 1.0, для ArUco-коробок — 0.05 по одной оси и box_w по другой)
            dx = abs(w_x2 - w_x1)
            dy = abs(w_y2 - w_y1)
            
            # Загружаем базовый 3D-меш куба из встроенных ресурсов Panda3D (размер по умолчанию 1х1х1м)
            wall_cube = self.loader.loadModel("box")
            wall_cube.reparentTo(self.render)
            
            # ЖЕЛЕЗНОЕ ИСПРАВЛЕНИЕ МАСШТАБА:
            # Динамически растягиваем куб под честные размеры преграды.
            # По осям X и Y передаем вычисленные дельты dx и dy. 
            # По оси Z (высота) жестко ставим 1.0 метр для стен лабиринта, 
            # либо можешь привязать к высоте маркера, но оставим 1.0 для монолитности.
            wall_cube.setScale(dx, dy, 1.0)
            
            # Ставим куб строго в левый нижний угол его математической ячейки (w_x1, w_y1) на уровень пола (Z=0.0)
            # Благодаря setScale, куб аккуратно вырастет точно до границ w_x2 и w_y2!
            wall_cube.setPos(w_x1, w_y1, 0.0)
            
            # Красим все физические преграды Лидара в фирменный бирюзовый цвет Gazebo Classic
            wall_cube.setColor(0.0, 0.6, 0.6, 1)

        # Размещение знаков на передних гранях толстых кубов
        for (mx, my), symbol in sign_zones.items():
            ros_y = float(maze_height - 1 - my)
            sign_box = self.loader.loadModel("box")
            sign_box.reparentTo(self.render)
            
            sign_box.setPos(mx + 0.5 - (sign_w / 2.0), ros_y - 0.01, 0.5 - (sign_h / 2.0)) 
            sign_box.setScale(sign_w, 0.02, sign_h) 

            try:
                sign_tex = Texture()
                if symbol == 'S': sign_tex.read("textures/stop_sign.png")
                elif symbol == 'L': sign_tex.read("textures/turn_left.png")
                elif symbol == 'R': sign_tex.read("textures/turn_right.png")
                sign_box.setTexture(sign_tex, 1)
            except:
                if symbol == 'S': sign_box.setColor(1, 0, 0, 1)
                elif symbol == 'L': sign_box.setColor(0, 0, 1, 1)
                elif symbol == 'R': sign_box.setColor(0, 1, 0, 1)

        # --- 3. НАКЛЕИВАНИЕ ARUCO-МАРКЕРОВ НА ВНЕШНИЕ ГРАНИ ТОНКИХ КУБОВ (ЧЕРЕЗ МОДУЛЬНУЮ ТОЛЩИНУ) ---
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            for i in range(0, len(aruco_markers), 4):
                ax, ay, side_idx, aid = aruco_markers[i:i+4]
                side = int(side_idx) % 4
                
                # Координаты опорного угла из YAML
                w_x1 = float(int(ax))
                w_y1 = float(int(ay))

                # Рассчитываем геометрию на основе глобальной константы из модуля
                box_width = aruco_w + 0.10
                half_box_w = box_width / 2.0
                half_th = ARUCO_BOX_THICKNESS / 2.0
                
                # Микроскопический отступ (1 миллиметр), чтобы наклейка не сливалась со стенкой
                gap = 0.001 

                # Генерируем ArUco маркер с исправлением инверсии цветов
                m_img = cv2.aruco.generateImageMarker(aruco_dict, id=int(aid), sidePixels=256)
                m_img = cv2.bitwise_not(m_img) 
                m_rgb = cv2.cvtColor(m_img, cv2.COLOR_GRAY2RGB)
                
                pil_m = PILImage.fromarray(m_rgb)
                m_tex = Texture()
                m_tex.setup2dTexture(256, 256, Texture.T_unsigned_byte, Texture.F_rgb)
                m_tex.setRamImage(pil_m.tobytes())
                
                cm_m = CardMaker('aruco_sticker')
                cm_m.setFrame(-aruco_w/2.0, aruco_w/2.0, -aruco_h/2.0, aruco_h/2.0)
                sticker = self.render.attachNewNode(cm_m.generate())
                sticker.setTexture(m_tex, 1)

                z_pos = 0.5  # Центр по высоте

                # МАТЕМАТИЧЕСКИ ИДЕАЛЬНОЕ СОВПАДЕНИЕ С ЗАПЕКАНИЕМ:
                # Наклейка выносится ровно на внешнюю плоскость бирюзовой преграды (half_th + gap)
                if side == 0:    # 0 -> Смотрит вправо
                    bx = w_x1 + half_th
                    by = w_y1 + half_box_w
                    sticker.setPos(bx + half_th + gap, by, z_pos)
                    sticker.setHpr(90, 0, 0)
                elif side == 1:  # 1 -> Смотрит вперед
                    bx = w_x1 + half_box_w
                    by = w_y1 + half_th
                    sticker.setPos(bx, by + half_th + gap, z_pos)
                    sticker.setHpr(180, 0, 0)
                elif side == 2:  # 2 -> Смотрит влево
                    bx = w_x1 - half_th
                    by = w_y1 + half_box_w
                    sticker.setPos(bx - half_th - gap, by, z_pos)
                    sticker.setHpr(-90, 0, 0)
                elif side == 3:  # 3 -> Смотрит назад
                    bx = w_x1 + half_box_w
                    by = w_y1 - half_th
                    sticker.setPos(bx, by - half_th - gap, z_pos)
                    sticker.setHpr(0, 0, 0)
                    
        except Exception as e:
            print(f"Ошибка 3D-рендеринга наклеек ArUco: {e}")

class MockMoebiusMaze(Node):
    def __init__(self):
        super().__init__('mock_moebius_maze')
        
        # --- ОБЪЯВЛЯЕМ И СЧИТЫВАЕМ ВНЕШНИЕ ПАРАМЕТРЫ ---
        self.declare_parameter('initial_pose', [1.5, 1.5, 0.0])
        self.declare_parameter('maze_map', ["####", "#  #", "####"])
        
        # Считываем позицию [x, y, theta]
        init_pose = self.get_parameter('initial_pose').get_parameter_value().double_array_value
        self.x = init_pose[0]
        self.y = init_pose[1]
        self.th = init_pose[2]
        
        # Считываем карту лабиринта
        raw_maze = self.get_parameter('maze_map').get_parameter_value().string_array_value
        
        # ЖЕЛЕЗНОЕ ИСПРАВЛЕНИЕ: принудительно очищаем строки от разметки YAML/ROS2
        self.maze = [str(line).strip() for line in raw_maze]
        # ОТЛАДКА: Печатаем карту в консоль, заменяя пробелы на точки '.', 
        # чтобы точно увидеть, сколько метров коридора считалось из YAML
        # ИСПРАВЛЕННЫЙ ВЫВОД КАРТЫ: Используем логгер ROS2 вместо print()
        self.get_logger().info("--- ПРОВЕРКА КАРТЫ РОБОТОМ ---")
        for line in self.maze:
            # Заменяем пробелы на точки для наглядности коридоров
            visual_line = str(line).replace(' ', '.')
            self.get_logger().info(visual_line)
        self.get_logger().info("-----------------------------")
        
        # Пересчитываем размеры на основе чистого списка
        self.maze_height = len(self.maze)
        self.maze_width = len(self.maze[0]) if self.maze_height > 0 else 0
        
        # Предзапекаем координаты стен для NumPy
        # Предзапекаем координаты стен для NumPy и сохраняем зоны знаков
        walls = []
        self.sign_zones = {} # Новый словарь для хранения типов знаков
        obstacle_symbols = ['#', 'S', 'L', 'R'] # Все типы препятствий

        for y in range(self.maze_height):
            for x in range(self.maze_width):
                symbol = self.maze[y][x]
                if symbol in obstacle_symbols:
                    # Инвертируем y, чтобы Строка 0 стала самым верхом (maze_height - 1 - y)
                    ros_y = float(self.maze_height - 1 - y)
                    walls.append([float(x), float(x) + 1.0, ros_y, ros_y + 1.0])
                    
                    # Если это буква знака, запоминаем её позицию
                    if symbol != '#':
                        self.sign_zones[(x, y)] = symbol

        # --- ИНИЦИАЛИЗАЦИЯ 3D-ДВИЖОК ПО ОРИГИНАЛЬНОМУ SELF.WALLS ---
        self.declare_parameter('line_points', [1.5, 1.5, 2.5, 1.5])
        self.declare_parameter('sign_width', 0.3)
        self.declare_parameter('sign_height', 0.3)
        self.declare_parameter('aruco_width', 0.4)
        self.declare_parameter('aruco_height', 0.4)
        self.declare_parameter('aruco_markers', [2.0, 2.0, 0.0, 42.0]) # Пока не внедряем в физику

        self.bridge = CvBridge()

        # Читаем параметры из YAML
        line_pts = self.get_parameter('line_points').get_parameter_value().double_array_value
        aruco_mk = self.get_parameter('aruco_markers').get_parameter_value().double_array_value
        sign_w = self.get_parameter('sign_width').get_parameter_value().double_value
        sign_h = self.get_parameter('sign_height').get_parameter_value().double_value
        aruco_w = self.get_parameter('aruco_width').get_parameter_value().double_value
        aruco_h = self.get_parameter('aruco_height').get_parameter_value().double_value
        #ARUCO_BOX_THICKNESS = 0.05

        # 2. ОСТОРОЖНОЕ ЗАПЕКАНИЕ: дописываем кубы ArUco-маркеров как полноценные жесткие кубы 1х1м
        # Координаты ay принудительно инвертируем (maze_height - 1 - ay), чтобы они совпали с сеткой ROS 2!
        for i in range(0, len(aruco_mk), 4):
            ax, ay, side_idx, aid = aruco_mk[i:i+4]
            side = int(side_idx) % 4
            
            # Координаты левого нижнего угла куба лабиринта, к которому привязан маркер
            w_x1 = float(int(ax))
            w_y1 = float(int(ay))
            w_x2 = w_x1
            w_y2 = w_y1

            # Ширина коробки-подложки: размер маркера + 5 см с каждой стороны (+10 см всего)
            box_width = aruco_w + 0.10
            half_box_w = box_width / 2.0
            half_th = ARUCO_BOX_THICKNESS / 2.0

            # Математический расчет ортогональных границ [x1, x2, y1, y2] для Лидара и физики
            if side == 0:    # 0 -> Смотрит вправо (Прижата к правой грани куба w_x2)
                cx = w_x2 + half_th
                cy = w_y1 + half_box_w
                walls.append([cx - half_th, cx + half_th, cy - half_box_w, cy + half_box_w])
            elif side == 1:  # 1 -> Смотрит вперед (Прижата к дальней грани куба w_y2)
                cx = w_x1 + half_box_w
                cy = w_y2 + half_th
                walls.append([cx - half_box_w, cx + half_box_w, cy - half_th, cy + half_th])
            elif side == 2:  # 2 -> Смотрит влево (Прижата к левой грани куба w_x1)
                cx = w_x1 - half_th
                cy = w_y1 + half_box_w
                walls.append([cx - half_th, cx + half_th, cy - half_box_w, cy + half_box_w])
            elif side == 3:  # 3 -> Смотрит назад (Прижата к ближней грани куба w_y1)
                cx = w_x1 + half_box_w
                cy = w_y1 - half_th
                walls.append([cx - half_box_w, cx + half_box_w, cy - half_th, cy + half_th])

        self.walls = np.array(walls, dtype=np.float64)
        # Скорости Moebius
        self.vx = 0.0; self.vy = 0.0; self.vth = 0.0
        
        self.lidar_points = 360 
        self.dt = 0.02 # Физика и одометрия на 50 Гц
        self.sim_time_sec = 0.0
        
        # Новое разрешение камеры
        self.cam_width = 320
        self.cam_height = 240
        self.frame_count = 0
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.cam_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer = self.create_timer(self.dt, self.update_loop)

        # Инициализируем публикатор статической карты мира
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Предзапекаем сообщение OccupancyGrid один раз при старте
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'odom' # Или 'map', если у вас настроен map_server
        
        # Настраиваем метаданные (1 символ лабиринта = 1 метр в мире)
        self.map_msg.info.resolution = 1.0  # Размер одной ячейки (1 метр)
        self.map_msg.info.width = self.maze_width
        self.map_msg.info.height = self.maze_height
        
        # Начало координат сетки карты (0,0) совпадает с нижним левым углом
        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.0
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        
        # Заполняем массив данных карты: 100 — стена (черный цвет), 0 — пусто (белый цвет)
        map_data = []
        # Идем снизу вверх по мировой оси Y (от последней строки к первой)
        for y in reversed(range(self.maze_height)):
            for x in range(self.maze_width):
                if self.maze[y][x] in ['#', 'S', 'L', 'R']:
                    map_data.append(100) # Стена занята
                else:
                    map_data.append(0)   # Свободная зона коридора
                    
        self.map_msg.data = map_data
        
        # Таймер для периодической публикации карты (например, раз в секунду)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        # Передаем оригинальный self.walls, созданный твоим кодом.
        self.renderer = PandaRenderer(
            self.cam_width, self.cam_height, self.walls, 
            self.sign_zones, self.maze_height, self.maze_width, line_pts, aruco_mk,
            sign_w, sign_h, aruco_w, aruco_h
        )

        self.get_logger().info("Нода Moebius запущена (Разрешение камеры: 320x240, Векторный NumPy).")

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def update_loop(self):
        # 1. СИМУЛЯЦИЯ ВРЕМЕНИ
        self.sim_time_sec += self.dt
        sec = int(self.sim_time_sec)
        nanosec = int((self.sim_time_sec - sec) * 1e9)
        time_msg = Time(sec=sec, nanosec=nanosec)
        
        clock_msg = Clock()
        clock_msg.clock = time_msg
        self.clock_pub.publish(clock_msg)
        
        # 2. ОДОМЕТРИЯ (МЕКАНУМ)
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * self.dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * self.dt
        delta_th = self.vth * self.dt
        
        new_x = self.x + delta_x; new_y = self.y + delta_y
        if self.is_clear(new_x, self.y): self.x = new_x
        if self.is_clear(self.x, new_y): self.y = new_y
        self.th += delta_th
        q = self.euler_to_quaternion(0, 0, self.th)

        # Публикуем TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = time_msg; t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x; t.transform.translation.y = self.y; t.transform.translation.z = 0.0

        # Разворачиваем кватернион по индексам (исправлено)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

        # Публикуем /odom
        odom = Odometry()
        odom.header.stamp = time_msg; odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y; odom.pose.pose.position.z = 0.0

        # Разворачиваем кватернион по индексам для одометрии (исправлено)
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

        # 3. ВЕКТОРНЫЙ ЛИДАР NUMPY С ОПРЕДЕЛЕНИЕМ ИНДЕКСА СТЕНЫ
        laser_x = self.x - 0.035 * math.cos(self.th)
        laser_y = self.y - 0.035 * math.sin(self.th)

        scan = LaserScan()
        scan.header.stamp = time_msg; scan.header.frame_id = 'lidar_link'
        scan.angle_min = 0.0; scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / self.lidar_points
        scan.range_min = 0.15; scan.range_max = 12.0

        angles = np.arange(self.lidar_points) * scan.angle_increment + self.th
        cos_a = np.cos(angles)[:, np.newaxis]
        sin_a = np.sin(angles)[:, np.newaxis]
        cos_a = np.where(np.abs(cos_a) < 1e-5, 1e-5, cos_a)
        sin_a = np.where(np.abs(sin_a) < 1e-5, 1e-5, sin_a)

        r_x1 = (self.walls[:, 0] - laser_x) / cos_a
        r_x2 = (self.walls[:, 1] - laser_x) / cos_a
        r_y1 = (self.walls[:, 2] - laser_y) / sin_a
        r_y2 = (self.walls[:, 3] - laser_y) / sin_a

        y_at_x1 = laser_y + r_x1 * sin_a; y_at_x2 = laser_y + r_x2 * sin_a
        x_at_y1 = laser_x + r_y1 * cos_a; x_at_y2 = laser_x + r_y2 * cos_a

        valid_x1 = (y_at_x1 >= self.walls[:, 2]) & (y_at_x1 <= self.walls[:, 3]) & (r_x1 > 0)
        valid_x2 = (y_at_x2 >= self.walls[:, 2]) & (y_at_x2 <= self.walls[:, 3]) & (r_x2 > 0)
        valid_y1 = (x_at_y1 >= self.walls[:, 0]) & (x_at_y1 <= self.walls[:, 1]) & (r_y1 > 0)
        valid_y2 = (x_at_y2 >= self.walls[:, 0]) & (x_at_y2 <= self.walls[:, 1]) & (r_y2 > 0)

        r_x1 = np.where(valid_x1, r_x1, 12.0); r_x2 = np.where(valid_x2, r_x2, 12.0)
        r_y1 = np.where(valid_y1, r_y1, 12.0); r_y2 = np.where(valid_y2, r_y2, 12.0)

        all_r = np.minimum(np.minimum(r_x1, r_x2), np.minimum(r_y1, r_y2))
        ranges = np.min(all_r, axis=1)
        ranges = np.clip(ranges, scan.range_min, scan.range_max)

        # НАХОДИМ ИНДЕКС СТЕНЫ ДЛЯ КАЖДОГО ЛУЧА
        # argmin показывает, какая именно из стен в массиве self.walls оказалась ближе всего
        closest_wall_indices = np.argmin(all_r, axis=1)

        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

        # --- 4. ПОЛНОСТЬЮ АППАРАТНАЯ 3D-КАМЕРА (СТРОГАЯ СОНАПРАВЛЕННОСТЬ ОСЕЙ) ---
        self.frame_count += 1
        if self.frame_count % 3 == 0:  # Частота кадров камеры ~16 Гц
            
            # ЧЕСТНОЕ ДИРЕКТНОЕ СОПОСТАВЛЕНИЕ 1-в-1:
            # Твой X (вправо по матрице) -> это Panda X (вправо)
            # Твой Y (вверх по матрице)   -> это Panda Y (вперед/вверх)
            # Высота камеры — 15 см над полом
            self.renderer.cam.setPos(float(self.x), float(self.y), 0.15)
            
            # Синхронизация угла курса:
            # Переводим радианы th в градусы и вычитаем 90.0, 
            # чтобы выровнять базовый нулевой вектор взгляда робота и движка Panda3D
            angle_deg = math.degrees(self.th) - 90.0
            self.renderer.cam.setHpr(angle_deg, 0, 0) # Смотрим строго вперед горизонтально

            # Жесткий синхронный запуск рендеринга кадра в память CPU
            self.renderer.graphicsEngine.renderFrame()
            self.renderer.graphicsEngine.syncFrame()

            # Копируем пиксели из RAM-буфера текстуры
            tex_data = self.renderer.tex.getRamImage()
            if tex_data is None:
                return

            # Выкачиваем изолированную копию байт из C++ ядра Panda3D
            raw_bytes = tex_data.get_data()
            img_np = np.frombuffer(raw_bytes, dtype=np.uint8).copy()
            
            # Проверка размера массива для 4 каналов (RGBA отдает RTM_copy_ram): 240 * 320 * 4
            expected_size = self.cam_height * self.cam_width * 4
            if img_np.size == 0 or img_np.size != expected_size:
                return

            # Формируем стандартную матрицу OpenCV
            frame_rgba = img_np.reshape((self.cam_height, self.cam_width, 4))
            
            # Отсекаем альфа-канал, переводя в чистый 3-канальный BGR (как в твоей старой версии)
            frame = cv2.cvtColor(frame_rgba, cv2.COLOR_BGRA2BGR)
            frame = cv2.flip(frame, 0) # Исправляем вертикальный переворот буфера Panda3D

            # Ручная сборка сообщения Image — в точности как у тебя (100% совместимость с RViz)
            img_msg = Image()
            img_msg.header.stamp = time_msg
            img_msg.header.frame_id = 'camera_link'
            img_msg.height = self.cam_height
            img_msg.width = self.cam_width
            img_msg.encoding = 'bgr8'
            img_msg.step = self.cam_width * 3  # Жесткий шаг строки: 320 * 3 = 960 байт
            img_msg.data = frame.tobytes()

            self.cam_pub.publish(img_msg)


    def is_clear(self, x, y):
        mx = int(x)
        # ИСПРАВЛЕНИЕ: переводим физический Y из ROS 2 назад в индекс строки (сверху-вниз)
        my = int(self.maze_height - 1 - int(y))
        
        # Строгая защита от вылета за границы карты
        if mx < 0 or mx >= self.maze_width or my < 0 or my >= self.maze_height: 
            return False
            
        # Проверяем символ в очищенной матрице
        return self.maze[my][mx] not in ['#', 'S', 'L', 'R']

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        return [qx, qy, qz, qw]

    def publish_map(self):
        # Обновляем штамп времени симуляции в шапке карты
        sec = int(self.sim_time_sec)
        nanosec = int((self.sim_time_sec - sec) * 1e9)
        self.map_msg.header.stamp.sec = sec
        self.map_msg.header.stamp.nanosec = nanosec
        
        # Публикуем карту в топик /map
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args); node = MockMoebiusMaze(); rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
