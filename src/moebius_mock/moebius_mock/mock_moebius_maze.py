# Сначала импортируем ТОЛЬКО функцию настроек
from panda3d.core import loadPrcFileData
loadPrcFileData("", "window-type offscreen")
loadPrcFileData("", "audio-library-name null")
loadPrcFileData("", "win-size 320 240")
loadPrcFileData("", "support-keyboard 0")
loadPrcFileData("", "support-mouse 0")
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
# loadPrcFileData("", "notify-level-display debug")
# loadPrcFileData("", "textures-power-2 none")
# loadPrcFileData("", "driver-generate-mipmaps 0")
# loadPrcFileData("", "textures-square none")
from panda3d.core import DirectionalLight, Vec4, Texture, CardMaker, GraphicsOutput, FrameBufferProperties, WindowProperties, GraphicsPipe
# Автовыбор GPU/CPU, скрытый офскрин-режим, отключение окон, теней, звука и устройств ввода
#loadPrcFileData("", "window-type offscreen")
#loadPrcFileData("", "audio-library none")
#loadPrcFileData("", "shadow-depth-bits 0")
#loadPrcFileData("", "framebuffer-multisample 0")
#loadPrcFileData("", "multisamples 0")
#loadPrcFileData("", "support-keyboard 0")
#loadPrcFileData("", "support-mouse 0")
#loadPrcFileData("", "notify-level fatal")
#loadPrcFileData("", "notify-level-device error")
#loadPrcFileData("", "window-type offscreen")    # Строго скрытый режим
#loadPrcFileData("", "audio-library-name null")  # Выключаем звук
#loadPrcFileData("", "load-display osmesadisplay")
#loadPrcFileData("", "win-size 320 240")         # Фиксируем разрешение кадра робота

# ДОБАВЬТЕ ЭТИ СТРОЧКИ: Они сбросят EGL и заставят Панду работать в чистом RAM-контексте
#loadPrcFileData("", "load-display pandagl")
#loadPrcFileData("", "aux-display egldisplay")

#loadPrcFileData("", "threading-model ")          # Оставляем пустую строку!
#loadPrcFileData("", "support-threads 0")
#loadPrcFileData("", "gl-finish 1")               # Форсирует за

from direct.showbase.ShowBase import ShowBase
#from cv_bridge import CvBridge
from panda3d.core import PNMImage, StringStream
# --- ГЛОБАЛЬНЫЕ КОНСТАНТЫ СИМУЛЯТОРА ---
BOX_THICKNESS = 0.05  # Толщина маркерной коробки: ровно 5 см
class PandaRenderer(ShowBase):
    # Разрешение подложки пола: 512 пикселей на 1 метр лабиринта (1 пиксель = 2 мм)
    SCALE_M_TO_PX = 512.0
    #def __init__(self):
    def __init__(self, ros_node_ref, cam_w, cam_h, walls, road_sgn_poses, road_sgn_files, maze_height, maze_width, line_points, aruco_markers, sign_w, sign_h, aruco_w, aruco_h):
        # 1. ХАК ДЛЯ СИСТЕМНЫХ ПУТЕЙ PANDA3D (Принудительно подтягиваем встроенные модели)
        import os
        import panda3d
        from panda3d.core import getModelPath
        
        panda_dir = os.path.dirname(panda3d.__file__)
        models_dir = os.path.join(panda_dir, "models")
        
        # Загружаем ShowBase. Движок инициализирует конвейер
        super().__init__(self)
        self.ros_node = ros_node_ref
        
        if os.path.exists(models_dir):
            getModelPath().appendPath(models_dir)

        self.cam_width = cam_w
        self.cam_height = cam_h
        
        # Настройка 4-канального offscreen-буфера в оперативной памяти (RGBA)
        #fb_props = FrameBufferProperties.getDefault()
        #win_props = WindowProperties.size(self.cam_width, self.cam_height)

        #self.buffer = self.graphicsEngine.makeOutput(
        #    self.pipe, "cpu_offscreen_buffer", 0, fb_props, win_props,
        #    GraphicsPipe.BF_refuse_window | GraphicsPipe.BF_fb_props_optional
        #)

        #self.tex = Texture()
        #self.buffer.addRenderTexture(self.tex, GraphicsOutput.RTM_copy_ram, GraphicsOutput.RTP_color)

        #dr = self.buffer.makeDisplayRegion()
        #dr.setCamera(self.cam)

        # Теперь нам не нужен сложный makeOutput, используем стандартное скрытое окно self.win
#         dr = self.win.makeDisplayRegion()
#         dr.setCamera(self.cam)


        # Настройка линзы: FOV 120° и ближняя плоскость 1 см, чтобы пол не резался
        self.cam.node().getLens().setFov(120.0)
        self.cam.node().getLens().setAspectRatio(self.cam_width / self.cam_height)
        self.cam.node().getLens().setNearFar(0.01, 100.0)
        self.scene = self.loader.loadModel("models/box")
        self.scene.reparentTo(self.render)
        self.scene.setPos(0, 5, -0.5)
        self.scene.setColor(1.0, 0.0, 0.0, 1.0)

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

#         pil_img = PILImage.fromarray(cv2.cvtColor(floor_img, cv2.COLOR_BGR2RGB))
#         p3d_tex = Texture()
#         p3d_tex.setup2dTexture(floor_px_w, floor_px_h, Texture.T_unsigned_byte, Texture.F_rgb)
#         p3d_tex.setRamImage(pil_img.tobytes())
        # Создаем пустую текстуру под размер пола
        p3d_tex = Texture()
        p3d_tex.setup2dTexture(floor_px_w, floor_px_h, Texture.T_unsigned_byte, Texture.F_rgb)
        
        # Конвертируем OpenCV матрицу в байты напрямую
        rgb_bytes = cv2.cvtColor(floor_img, cv2.COLOR_BGR2RGB).tobytes()
        
        # Безопасно загружаем байты с флагом компрессии CM_on, чтобы убрать BadAccess
        p3d_tex.setRamImage(rgb_bytes, Texture.CM_on)
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

        # --- 3. НАКЛЕИВАНИЕ ARUCO-МАРКЕРОВ НА ВНЕШНИЕ ГРАНИ ТОНКИХ КУБОВ (ЧЕРЕЗ МОДУЛЬНУЮ ТОЛЩИНУ) ---
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            for i in range(0, len(aruco_markers), 4):
                ax, ay, side_idx, aid = aruco_markers[i:i+4]
                side = int(side_idx) % 4
                
                # Координаты опорного угла из YAML
                w_x1 = float(ax)
                w_y1 = float(ay)

                # Рассчитываем геометрию на основе глобальной константы из модуля
                box_width = aruco_w + 0.10
                half_box_w = box_width / 2.0
                half_th = BOX_THICKNESS / 2.0
                
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

        # --- 4. НАКЛЕИВАНИЕ ДОРОЖНЫХ ЗНАКОВ НАПРЯМУЮ ИЗ ФАЙЛОВ КАРТИНОК ИЗ YAML ---
        try:
            # Считаем, сколько знаков у нас задано в массиве позиций
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('moebius_mock')

            num_signs = len(road_sgn_poses) // 3
            
            for idx in range(num_signs):
                # Извлекаем тройку чисел для текущего знака
                sx = road_sgn_poses[idx * 3]
                sy = road_sgn_poses[idx * 3 + 1]
                side_idx = road_sgn_poses[idx * 3 + 2]
                
                # Достаем имя файла картинки из строкового массива по этому же индексу знака!
                # Если в YAML файлов меньше, чем знаков, защищаем код от падения по IndexError
                if idx < len(road_sgn_files):
                    img_filename = road_sgn_files[idx]
                else:
                    img_filename = "default_fallback.png"

                # Загружаем битовый файл текстуры знака напрямую по переданному имени файла
                sign_tex = Texture()
                texture_path = os.path.join(pkg_dir, 'textures', img_filename)
                success = sign_tex.read(texture_path)
                
                # Если файл картинки не найден в папке textures/, создаем резервную серую заглушку
                if not success:
                    print(f"Предупреждение: Файл текстуры textures/{img_filename} не найден!")
                    sign_tex.setup2dTexture(16, 16, Texture.T_unsigned_byte, Texture.F_rgb)
                    sign_tex.setClearColor(Vec4(0.3, 0.3, 0.3, 1)) # Серый цвет
                    sign_tex.clear()

                # Клеим картинку знака строго на внешнюю грань бирюзовой тонкой коробки!
                # Координаты опорного угла из YAML
                w_x1 = float(sx)
                w_y1 = float(sy)

                # Рассчитываем геометрию на основе глобальной константы из модуля
                box_width = sign_w + 0.10
                half_box_w = box_width / 2.0
                half_th = BOX_THICKNESS / 2.0

                # Микроскопический отступ (1 миллиметр), чтобы наклейка не сливалась со стенкой
                gap = 0.001

                # Генерируем ArUco маркер с исправлением инверсии цветов
                #m_img = cv2.aruco.generateImageMarker(aruco_dict, id=int(aid), sidePixels=256)
                #m_img = cv2.bitwise_not(m_img)
                #m_rgb = cv2.cvtColor(m_img, cv2.COLOR_GRAY2RGB)

                #pil_m = PILImage.fromarray(m_rgb)
                #m_tex = Texture()
                #m_tex.setup2dTexture(256, 256, Texture.T_unsigned_byte, Texture.F_rgb)
                #m_tex.setRamImage(pil_m.tobytes())

                cm_m = CardMaker('sign_sticker')
                cm_m.setFrame(-sign_w/2.0, sign_w/2.0, -sign_h/2.0, sign_h/2.0)
                sticker = self.render.attachNewNode(cm_m.generate())
                sticker.setTexture(sign_tex, 1)

                z_pos = 0.5  # Центр по высоте

                # МАТЕМАТИЧЕСКИ ИДЕАЛЬНОЕ СОВПАДЕНИЕ С ЗАПЕКАНИЕМ:
                # Наклейка выносится ровно на внешнюю плоскость бирюзовой преграды (half_th + gap)
                if side_idx == 0:    # 0 -> Смотрит вправо
                    bx = w_x1 + half_th
                    by = w_y1 + half_box_w
                    sticker.setPos(bx + half_th + gap, by, z_pos)
                    sticker.setHpr(90, 0, 0)
                elif side_idx == 1:  # 1 -> Смотрит вперед
                    bx = w_x1 + half_box_w
                    by = w_y1 + half_th
                    sticker.setPos(bx, by + half_th + gap, z_pos)
                    sticker.setHpr(180, 0, 0)
                elif side_idx == 2:  # 2 -> Смотрит влево
                    bx = w_x1 - half_th
                    by = w_y1 + half_box_w
                    sticker.setPos(bx - half_th - gap, by, z_pos)
                    sticker.setHpr(-90, 0, 0)
                elif side_idx == 3:  # 3 -> Смотрит назад
                    bx = w_x1 + half_box_w
                    by = w_y1 - half_th
                    sticker.setPos(bx, by - half_th - gap, z_pos)
                    sticker.setHpr(0, 0, 0)
                
        except Exception as e:
            print(f"Ошибка 3D-рендеринга файлов дорожных знаков: {e}")
        self.taskMgr.add(self.panda_render_task, "PandaRenderTask")

    def panda_render_task(self, task):
        node = self.ros_node
        
        # Если фоновый поток еще не просил кадр — просто ждем
        if not node.render_requested:
            return task.cont
            
        node.render_requested = False

#         # --- БЕЗОПАСНЫЙ РЕНДЕР СТРОГО В MAIN THREAD ---
#         # Движение камеры берем из переменных ноды
#         cam_x = 3.0 * math.sin(node.angle)
#         cam_y = 5.0 + 3.0 * math.cos(node.angle)
#         self.cam.setPos(cam_x, cam_y, 0.5)
#         self.cam.lookAt(0, 5, -0.5)

        # Вызываем рендеринг кадра (в родном потоке OpenGL)
        self.cam.setPos(float(node.x), float(node.y), 0.15)
        angle_deg = math.degrees(node.th) - 90.0
        self.cam.setHpr(angle_deg, 0, 0)
        self.graphicsEngine.renderFrame()

        # Захват изображения кадра
        if self.win.getScreenshot(node.pnm_img):
            stream = StringStream()
            node.pnm_img.write(stream, "bmp")
            raw_bytes = stream.getData()

            if raw_bytes:
                img_np = np.frombuffer(raw_bytes, dtype=np.uint8)
                frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)

                if frame is not None and frame.size > 0:
                    node.publish_frame_message(frame)

        return task.cont

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

        # --- ИНИЦИАЛИЗАЦИЯ 3D-ДВИЖОК ПО ОРИГИНАЛЬНОМУ SELF.WALLS ---
        self.declare_parameter('line_points', [1.5, 1.5, 2.5, 1.5])
        self.declare_parameter('sign_width', 0.3)
        self.declare_parameter('sign_height', 0.3)
        self.declare_parameter('aruco_width', 0.4)
        self.declare_parameter('aruco_height', 0.4)
        self.declare_parameter('aruco_markers', [2.0, 2.0, 0.0, 42.0]) # Пока не внедряем в физику
        self.declare_parameter('road_signs_poses', [3.0, 2.0, 3.0]) # Позиции знаков (числа)
        self.declare_parameter('road_signs_files', ["red.png"]) # Имена файлов (строки)

        #self.bridge = CvBridge()

        # Читаем параметры из YAML
        line_pts = self.get_parameter('line_points').get_parameter_value().double_array_value
        aruco_mk = self.get_parameter('aruco_markers').get_parameter_value().double_array_value
        sign_w = self.get_parameter('sign_width').get_parameter_value().double_value
        sign_h = self.get_parameter('sign_height').get_parameter_value().double_value
        aruco_w = self.get_parameter('aruco_width').get_parameter_value().double_value
        aruco_h = self.get_parameter('aruco_height').get_parameter_value().double_value
        road_sgn_poses = self.get_parameter('road_signs_poses').get_parameter_value().double_array_value
        road_sgn_files = self.get_parameter('road_signs_files').get_parameter_value().string_array_value

        # 2. ОСТОРОЖНОЕ ЗАПЕКАНИЕ: дописываем кубы знаков / маркеров как полноценные жесткие кубы
        for i in range(0, len(road_sgn_poses), 3):
            ax, ay, side_idx = road_sgn_poses[i:i+3]
            side = int(side_idx) % 4

            # Координаты левого нижнего угла куба лабиринта, к которому привязан маркер
            w_x1 = float(ax)
            w_y1 = float(ay)
            w_x2 = w_x1
            w_y2 = w_y1

            # Ширина коробки-подложки: размер маркера + 5 см с каждой стороны (+10 см всего)
            box_width = sign_w + 0.10
            half_box_w = box_width / 2.0
            half_th = BOX_THICKNESS / 2.0

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
        for i in range(0, len(aruco_mk), 4):
            ax, ay, side_idx, aid = aruco_mk[i:i+4]
            side = int(side_idx) % 4
            
            # Координаты левого нижнего угла куба лабиринта, к которому привязан маркер
            w_x1 = float(ax)
            w_y1 = float(ay)
            w_x2 = w_x1
            w_y2 = w_y1

            # Ширина коробки-подложки: размер маркера + 5 см с каждой стороны (+10 см всего)
            box_width = aruco_w + 0.10
            half_box_w = box_width / 2.0
            half_th = BOX_THICKNESS / 2.0

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
        
        #self.timer = self.create_simulator_timer(self.dt, self.update_loop)

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
        # --- ЗАПОЛНЕНИЕ КАРТЫ С ВЫДЕЛЕНИЕМ ЗОН ARUCO СЕРЫМ ЦВЕТОМ (45) ---
        map_data = []
        for y in reversed(range(self.maze_height)):
            for x in range(self.maze_width):
                
                # Проверяем, привязан ли к этой ячейке (x, y) ArUco-маркер из YAML
                is_aruco_cell = False
                for i in range(0, len(aruco_mk), 4):
                    ax = int(aruco_mk[i])
                    ay = int(self.maze_height-aruco_mk[i+1])
                    if x == ax and y == ay:
                        is_aruco_cell = True
                        break

                # Проверяем, привязан ли к этой ячейке (x, y) знак из YAML
                for i in range(0, len(road_sgn_poses), 3):
                    ax = int(road_sgn_poses[i]) #+half_th if road_sgn_poses[i+2]%2==0 else ((sign_w + 0.10) / 2.0))
                    ay = int(self.maze_height-(road_sgn_poses[i+1])) #+half_th if road_sgn_poses[i+2]%2==1 else ((sign_w + 0.10) / 2.0)))
                    if x == ax and y == ay:
                        is_aruco_cell = True
                        break

                # Присваиваем веса ячейкам для RViz2 /map
                if self.maze[y][x] in ['#', 'S', 'L', 'R']:
                    map_data.append(100) # Глухая стена лабиринта -> ЧЕРНЫЙ цвет
                elif is_aruco_cell:
                    map_data.append(45)  # Зона тонкого ArUco-щита -> СЕРЫЙ цвет
                else:
                    map_data.append(0)   # Свободная зона коридора -> БЕЛЫЙ цвет
                    
        self.map_msg.data = map_data
        
        # Таймер для периодической публикации карты (например, раз в секунду)
        self.map_timer = self.create_timer(1.0, self.publish_map)
        
        
        self.render_requested = False
        self.latest_time_msg = None
        self.pnm_img = PNMImage(self.cam_width, self.cam_height)

        # Передаем оригинальный self.walls, созданный твоим кодом.
        self.renderer = PandaRenderer(self,
            self.cam_width, self.cam_height, self.walls, 
            road_sgn_poses, road_sgn_files, self.maze_height, self.maze_width, line_pts, aruco_mk,
            sign_w, sign_h, aruco_w, aruco_h
        )
        self.timer = self.create_simulator_timer(self.dt, self.update_loop)
        self.get_logger().info("Нода Moebius запущена (Разрешение камеры: 320x240, Векторный NumPy).")

    def create_simulator_timer(self, period_sec, callback):
        """
        ДИНАМИЧЕСКИЙ АДАПТИВНЫЙ ТАЙМЕР СИМУЛЯТОРА.
        Автоматически рассчитывает и печатает Real-Time Factor (RTF) раз в секунду.
        """
        def timer_worker():
            import time
            next_tick = time.time()

            # --- ПЕРЕМЕННЫЕ ДЛЯ РАСЧЕТА RTF ---
            last_report_wall_time = time.time()
            last_report_sim_time = self.sim_time_sec # Начальное SimTime симулятора

            while True:
                start_calc = time.time()
                try:
                    # Вызываем физический шаг (update_loop)
                    callback()
                except Exception as e:
                    print(f"\033[91m[SIMULATOR CORE ERROR]: {e}\033[0m", flush=True)

                # --- ТАКТ ПОД СЧЕТ RTF (Раз в секунду реального времени) ---
                current_wall_time = time.time()
                wall_delta = current_wall_time - last_report_wall_time

                if wall_delta >= 1.0: # Прошла 1 реальная секунда на часах хоста
                    current_sim_time = self.sim_time_sec
                    sim_delta = current_sim_time - last_report_sim_time

                    # Главная формула: отношение виртуального времени к реальному
                    rtf = sim_delta / wall_delta

                    # Цветной красивый вывод фактора времени в консоль лаунчера
                    # Округляем до двух знаков для идеальной считываемости
                    print(f"[\033[94mMOEBIUS CORE\033[0m] Real-Time Factor (RTF): \033[92m{rtf:.2f}\033[0m", flush=True)

                    # Сдвигаем точки отсчета на следующий круг
                    last_report_wall_time = current_wall_time
                    last_report_sim_time = current_sim_time

                # --- СТАНДАРТНАЯ АДАПТИВНАЯ МАТЕМАТИКА СОН СИМУЛЯТОРА ---
                next_tick += period_sec
                sleep_time = next_tick - time.time()

                if sleep_time > 0.005:
                    # Машина справляется (расчет < целевого FPS)
                    time.sleep(sleep_time)
                else:
                    # Процессор нагружен тяжелыми нодами ИИ. Уходим в Slow-Motion.
                    next_tick = time.time()
                    time.sleep(0.005) # Окно безопасности для выдоха ОС

        import threading
        threading.Thread(target=timer_worker, daemon=True, name="Moebius_Adaptive_Physics").start()


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
            #self.renderer.cam.setPos(float(self.x), float(self.y), 0.15)
            
            # Синхронизация угла курса:
            # Переводим радианы th в градусы и вычитаем 90.0, 
            # чтобы выровнять базовый нулевой вектор взгляда робота и движка Panda3D
            #angle_deg = math.degrees(self.th) - 90.0

            #self.renderer.cam.setHpr(angle_deg, 0, 0) # Смотрим строго вперед горизонтально

            # Жесткий синхронный запуск рендеринга кадра в память CPU
            #self.renderer.graphicsEngine.openWindows() # перед renderFrame()
            #self.renderer.graphicsEngine.renderFrame()
            #self.renderer.graphicsEngine.syncFrame()

            # Копируем пиксели из RAM-буфера текстуры
            #tex_data = self.renderer.tex.getRamImage()
            #if tex_data is None:
            #    return

            # Выкачиваем изолированную копию байт из C++ ядра Panda3D
            #raw_bytes = tex_data.get_data()
            #img_np = np.frombuffer(raw_bytes, dtype=np.uint8).copy()
            
            # Проверка размера массива для 4 каналов (RGBA отдает RTM_copy_ram): 240 * 320 * 4
            #expected_size = self.cam_height * self.cam_width * 4
            #if img_np.size == 0 or img_np.size != expected_size:
            #    return

            # Формируем стандартную матрицу OpenCV
            #frame_rgba = img_np.reshape((self.cam_height, self.cam_width, 4))
            
            # Отсекаем альфа-канал, переводя в чистый 3-канальный BGR (как в твоей старой версии)
            #frame = cv2.cvtColor(frame_rgba, cv2.COLOR_BGRA2BGR)
            #frame = cv2.flip(frame, 0) # Исправляем вертикальный переворот буфера Panda3D

            #self.renderer.cam.setHpr(angle_deg, 0, 0) 
            #print(f"Камера стоит в: X={self.x}, Y={self.y}, Угол={angle_deg}")
            # Отрисовка кадра
            self.latest_time_msg =  self.get_clock().now().to_msg()
            self.render_requested = True
            #self.renderer.graphicsEngine.renderFrame()
            
#             # Новый безопасный захват пикселей из скрытого окна
#             from panda3d.core import PNMImage, StringStream
#             img = PNMImage(self.cam_width, self.cam_height)
#             
#             if not self.renderer.win.getScreenshot(img):
#                 return
#                 
#             stream = StringStream()
#             img.write(stream, "bmp")
#             raw_bytes = stream.getData()
#             
#             if not raw_bytes:
#                 return
#                 
#             # Декодируем BMP-байты напрямую в готовую BGR матрицу OpenCV
#             img_np = np.frombuffer(raw_bytes, dtype=np.uint8)
#             frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
#             
#             if frame is None or frame.size == 0:
#                 return
#                 
#             # Переворот cv2.flip и конвертация цвета больше НЕ НУЖНЫ, imdecode всё сделал сам!
# 
# 
#             # Ручная сборка сообщения Image — в точности как у тебя (100% совместимость с RViz)
#             img_msg = Image()
#             img_msg.header.stamp = time_msg
#             img_msg.header.frame_id = 'camera_link'
#             img_msg.height = self.cam_height
#             img_msg.width = self.cam_width
#             img_msg.encoding = 'bgr8'
#             img_msg.step = self.cam_width * 3  # Жесткий шаг строки: 320 * 3 = 960 байт
#             img_msg.data = frame.tobytes()
# 
#             self.cam_pub.publish(img_msg)

    def publish_frame_message(self, frame):
        """ Метод вызывается Главным Потоком, когда скриншот готов """
        msg = Image()
        msg.header.stamp = self.latest_time_msg if self.latest_time_msg else self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = self.cam_height
        msg.width = self.cam_width
        msg.encoding = 'bgr8'
        msg.step = self.cam_width * 3
        msg.data = frame.tobytes()
        self.cam_pub.publish(msg)


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
    rclpy.init(args=args); node = MockMoebiusMaze()
    import threading
    # Чтобы taskMgr Панды мог крутиться параллельно со spin() в рамках одного скрипта,
    # мы запускаем spin в отдельном потоке, освобождая Главный Поток Python для ядра Panda3D
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    node.renderer.run()
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
