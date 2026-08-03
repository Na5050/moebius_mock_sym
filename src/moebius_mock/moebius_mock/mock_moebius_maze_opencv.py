import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf2_ros
import math
import numpy as np

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
        
        self.timer = self.create_simulator_timer(self.dt, self.update_loop)

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

        # 4. ПОЛНОСТЬЮ ВЕКТОРНАЯ МАК-КАМЕРА С FOV 120° (ИСПРАВЛЕНА РАСПАКОВКА NUMPY)
        self.frame_count += 1
        if self.frame_count % 3 == 0:  # Частота кадров ~16 Гц
            img = Image()
            img.header.stamp = time_msg; img.header.frame_id = 'camera_link'
            img.height = self.cam_height; img.width = self.cam_width
            img.encoding = 'bgr8'; img.step = self.cam_width * 3

            # Базовый кадр (Потолок и Пол в формате BGR)
            frame = np.zeros((self.cam_height, self.cam_width, 3), dtype=np.uint8)
            frame[0:120, :, 0] = 100  # B потолка
            frame[0:120, :, 1] = 50   # G потолка
            frame[0:120, :, 2] = 50   # R потолка
            frame[120:240, :, :] = 60 # Серый пол

            # Настраиваем FOV 120 градусов
            fov_deg = 120.0
            fov_rad = math.radians(fov_deg)

            cols = np.arange(self.cam_width)
            # Угол для каждой колонки кадра (от -60° до +60°)
            rel_angles = -((cols - self.cam_width // 2) / (self.cam_width // 2)) * (fov_rad / 2.0)
            
            # Переводим в градусные индексы Лидара (-60° ... +60°)
            rel_angles_deg = np.degrees(rel_angles).astype(int)
            lidar_indices = (rel_angles_deg) % 360
            
            dist_cols = np.array(scan.ranges)[lidar_indices]

            # Мягкая и естественная высота стен по твоей оригинальной формуле
            wall_heights = (self.cam_height / (dist_cols + 0.001)).astype(int)
            wall_heights = np.clip(wall_heights, 0, self.cam_height)

            start_y = self.cam_height // 2 - wall_heights // 2
            end_y = self.cam_height // 2 + wall_heights // 2
            brightness = np.clip(180 - dist_cols * 18, 0, 255).astype(np.uint8)

            # 1. Заливаем фоновые бирюзовые стены
            for col in range(self.cam_width):
                y_top = start_y[col]
                y_bottom = end_y[col]
                if wall_heights[col] > 0:
                    frame[y_top:y_bottom, col, 0] = brightness[col] # B
                    frame[y_top:y_bottom, col, 1] = brightness[col] # G
                    frame[y_top:y_bottom, col, 2] = 0               # R

            # Переносим индексы стен Лидара на колонки камеры
            wall_indices_for_cols = closest_wall_indices[lidar_indices]

            # Группируем видимые колонки по индексам стен
            visible_walls = {}
            for col in range(self.cam_width):
                w_idx = wall_indices_for_cols[col]
                if w_idx not in visible_walls:
                    visible_walls[w_idx] = []
                visible_walls[w_idx].append(col)

            # 2. РЕНДЕРИНГ ТРАПЕЦИЙ С АДЕКВАТНЫМ НАКЛОНОМ КРАЕВ
            import cv2

            for w_idx, columns in visible_walls.items():
                wall_coords = self.walls[w_idx]
                
                # ЖЕЛЕЗНОЕ ИСПРАВЛЕНИЕ: Достаем скалярные значения по индексам из [x1, x2, y1, y2]
                w_x1 = float(wall_coords[0])
                w_y1 = float(wall_coords[2])
                
                mx = int(w_x1)
                my = int(self.maze_height - 1 - int(w_y1))

                if (mx, my) in self.sign_zones:
                    total_cols = len(columns)
                    
                    if total_cols >= 3:
                        one_third = total_cols // 3
                        center_cols = columns[one_third : total_cols - one_third]
                        
                        if len(center_cols) >= 2:
                            # Берем левый и правый лучи центральной трети знака
                            left_col = center_cols[0]
                            right_col = center_cols[-1]

                            # Берём чистые высоты из сбалансированного массива стен
                            h_left = wall_heights[left_col]
                            h_right = wall_heights[right_col]
                            
                            y_top_left = start_y[left_col]
                            y_top_right = start_y[right_col]

                            # Строим вершины трапеции. Наклон линий теперь мягкий и реалистичный.
                            pt_top_left  = [left_col, y_top_left + h_left // 3]
                            pt_bot_left  = [left_col, y_top_left + (2 * h_left) // 3]
                            
                            pt_top_right = [right_col, y_top_right + h_right // 3]
                            pt_bot_right = [right_col, y_top_right + (2 * h_right) // 3]

                            # Назначаем цвет знака в BGR с тенями
                            mid_col = center_cols[len(center_cols) // 2]
                            br = brightness[mid_col]
                            
                            sign_type = self.sign_zones[(mx, my)]
                            if sign_type == 'S':    # STOP — Красный
                                color = (0, 0, int(br))
                            elif sign_type == 'L':  # Налево — Синий
                                color = (int(br), 0, 0)
                            elif sign_type == 'R':  # Направо — Зеленый
                                color = (0, int(br), 0)
                            else:
                                color = (0, 0, 0)

                            pts_arr = np.array([
                                pt_top_left,
                                pt_top_right,
                                pt_bot_right,
                                pt_bot_left
                            ], dtype=np.int32)

                            cv2.fillPoly(frame, [pts_arr], color)

            img.data = frame.tobytes()
            self.cam_pub.publish(img)


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
