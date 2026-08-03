#!/usr/bin/env python3
#Copyright (c) 2026 Jurij Polikarpov
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty # Для легкого топика сердцебиения
from std_srvs.srv import Trigger # Для сервиса Stop
import math
import threading
from collections import namedtuple
from ifra_mol_1.srv import AutopilotTime, AutopilotRelative
from ifra_mol_1.msg import AutopilotTelemetry


# Структура для передачи координат (доступ через точку)
Pose = namedtuple('Pose', ['x', 'y', 'yaw'])

class PIDController:
    """Классический ПИД-регулятор с защитой от интегрального насыщения."""
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit  # Anti-windup лимит
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error: float, dt: float = 0.05) -> float:
        self.integral += error * dt
        self.integral = max(min(self.integral, self.i_limit), -self.i_limit)
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0


class RobotAutopilot(Node):
    def __init__(self):
        super().__init__('robot_autopilot')
        
        # Мьютекс для защиты данных при параллельной работе сервисов и таймера
        self.lock = threading.Lock()
        
        # ROS 2 Издатель и Подписчик одометрии
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.telemetry_pub = self.create_publisher(AutopilotTelemetry, '/autopilot/telemetry', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        
        # --- СТРАХОВКА: Подписка на топик сердцебиения ИИ ---
        self.hb_sub = self.create_subscription(Empty, '/autopilot/heartbeat', self._hb_callback, 10)
        
        # --- ИНТЕРФЕЙС: Объявление быстрых ROS 2 Сервисов ---
        # ВНИМАНИЕ: Предполагается, что эти два сервиса скомпилированы в вашем пакете
        # Если вы используете стандартные, замените на свои типы данных
        #from robot_autopilot.srv import AutopilotTime, AutopilotRelative
        self.srv_time = self.create_service(AutopilotTime, 'autopilot/go_time', self._srv_go_time_callback)
        self.srv_relative = self.create_service(AutopilotRelative, 'autopilot/go_relative', self._srv_go_relative_callback)
        self.srv_stop = self.create_service(Trigger, 'autopilot/stop', self._srv_stop_callback)
        
        # Состояние одометрии (защищено lock)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        
        # Менеджер статусов и режимов
        self.status = "IDLE"            # IDLE, MOVING, SUCCESS, ABORTED_TIMEOUT, ABORTED_STUCK
        self.target_mode = "IDLE"       # IDLE, TIME_DRIVE, RELATIVE_MOVE
        self.target_start_time = None   
        self.target_duration_ns = 0     
        
        # Параметры Сторожа ИИ (Watchdog)
        self.last_heartbeat_time = self.get_clock().now()
        self.watchdog_timeout_ns = int(1.5 * 1e9)  # Робот встанет, если ИИ замолчит на 0.5 сек
        
        # Параметры Детектора застревания (Stuck Detector)
        self.last_pose_check_time = self.get_clock().now()
        self.stuck_start_x = 0.0
        self.stuck_start_y = 0.0
        self.stuck_start_yaw = 0.0
        self.stuck_timeout_ns = int(3.0 * 1e9)     # Таймаут застревания 3 секунды
        self.stuck_xy_threshold = 0.01             # Допуск сдвига 1 см
        self.stuck_yaw_threshold = 0.02            # Допуск поворота ~1.1°
        
        # Скорости и Рампа ускорений (dt = 0.05 сек)
        self.tgt_vx, self.tgt_vy, self.tgt_omega = 0.0, 0.0, 0.0
        self.current_vx, self.current_vy, self.current_omega = 0.0, 0.0, 0.0
        self.max_accel_linear_step = 0.4 * 0.05   
        self.max_accel_angular_step = 1.0 * 0.05  
        
        # --- ОБНОВЛЕННЫЕ НАСТРОЙКИ ПИД ДЛЯ ЧЕТКОГО ФИНИША БЕЗ РАСКАЧКИ ---
        
        # Ось X (Вперед-назад): Снижаем kp, чтобы убрать финишные рывки, 
        # приподнимаем ki, чтобы робот гарантированно дотягивал до уставки вовремя.
        self.pid_x = PIDController(kp=1.2, ki=0.15, kd=0.08, i_limit=0.08)
        
        # Ось Y (Боковое смещение для меканума): Ролики проскальзывают сильнее, 
        # поэтому здесь ki нужен чуть агрессивнее.
        self.pid_y = PIDController(kp=1.2, ki=0.18, kd=0.08, i_limit=0.08)
        
        # Угол Yaw (Вращение на месте): Здесь инерция платформы меньше, 
        # поэтому делаем регулятор более строгим для точного удержания 90 градусов.
        self.pid_yaw = PIDController(kp=1.5, ki=0.05, kd=0.05, i_limit=0.15)
        
        self.target_dx, self.target_dy, self.target_dyaw = 0.0, 0.0, 0.0
        self.start_x, self.start_y, self.start_yaw = 0.0, 0.0, 0.0
        self.xy_tolerance, self.yaw_tolerance = 0.015, 0.025       
        
        # Фоновый таймер 20 Гц
        self.control_timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info("Сервисный автопилот с топиком Heartbeat успешно запущен.")

    def _odom_callback(self, msg):
        """Обновление координат из одометрии."""
        with self.lock:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            self.has_odom = True

    def _hb_callback(self, msg):
        """Коллбэк топика страховочного сердцебиения. Вызывается асинхронно из сети ROS."""
        with self.lock:
            self.last_heartbeat_time = self.get_clock().now()

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _reset_start_pose(self):
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw

    # -------------------- КОЛБЭКИ ROS 2 СЕРВИСОВ (МГНОВЕННЫЙ ОТВЕТ) --------------------

    def _publish_telemetry_now(self):
        """Внеочередная публикация телеметрии с актуальным статусом."""
        tel_msg = AutopilotTelemetry()
        tel_msg.x = self.current_x
        tel_msg.y = self.current_y
        tel_msg.yaw = self.current_yaw
        tel_msg.status = self.status
        self.telemetry_pub.publish(tel_msg)

    def _srv_go_time_callback(self, request, response):
        """Сервис мгновенно принимает параметры времени, сбрасывает прошлую цель и сразу возвращает ответ."""
        with self.lock:
            # --- ДОБАВЛЕНО: Если робот уже куда-то ехал, логируем вытеснение цели ---
            if self.status == "MOVING":
                self.get_logger().warning(f"Текущая цель {self.target_mode} была принудительно ПРЕРВАНА новой командой!")
                # Если вашему Дереву Поведения обязательно нужен SUCCESS, чтобы переключить шаг, 
                # мы можем временно выставить его здесь перед перезаписью:
                # self.status = "SUCCESS" 
            if not self.has_odom:
                response.success = False
                response.status_message = "Rejected: No telemetry from /odom"
                return response

            self.get_logger().info("Цель Время.")

            self._reset_start_pose()
            self.tgt_vx = request.vx
            self.tgt_vy = request.vy
            self.tgt_omega = request.omega
            self.target_duration_ns = int(request.duration * 1e9)
            self.target_start_time = self.get_clock().now()
            
            # Сброс таймеров безопасности
            self.last_heartbeat_time = self.get_clock().now()
            self.last_pose_check_time = self.get_clock().now()
            # --- ДОБАВЬТЕ ЭТИ ТРИ СТРОКИ СЮДА ДЛЯ СБРОСА РАМПЫ ---
            #self.current_vx = 0.0
            #self.current_vy = 0.0
            #self.current_omega = 0.0
            #self.target_duration_ns = 0
            self.target_mode = "TIME_DRIVE"
            self.status = "MOVING"
            self._publish_telemetry_now()
            response.success = True
            response.status_message = f"Accepted TIME_DRIVE: {request.duration}s"
            return response

    def _srv_go_relative_callback(self, request, response):
        """Сервис мгновенно принимает параметры смещения ПИД и сразу отдает управление клиенту."""
        with self.lock:
            # --- ДОБАВЛЕНО: Если робот уже куда-то ехал, логируем вытеснение цели ---
            if self.status == "MOVING":
                self.get_logger().warning(f"Текущая цель {self.target_mode} была принудительно ПРЕРВАНА новой командой!")
                # Если вашему Дереву Поведения обязательно нужен SUCCESS, чтобы переключить шаг, 
                # мы можем временно выставить его здесь перед перезаписью:
                # self.status = "SUCCESS" 
            if not self.has_odom:
                response.success = False
                response.status_message = "Rejected: No telemetry from /odom"
                return response

            self.get_logger().info("Цель Позиция.")

            self._reset_start_pose()
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()
            
            self.target_dx = request.dx
            self.target_dy = request.dy
            self.target_dyaw = math.radians(request.dyaw_deg)
            
            # Лимиты скоростей для ПИД-регулятора
            self.tgt_vx = 0.4
            self.tgt_vy = 0.4
            self.tgt_omega = 0.8
            
            # Инициализация детектора застревания
            self.last_pose_check_time = self.get_clock().now()
            self.stuck_start_x = self.current_x
            self.stuck_start_y = self.current_y
            self.stuck_start_yaw = self.current_yaw
            
            # Сброс сторожевого таймера
            self.last_heartbeat_time = self.get_clock().now()

            # --- ДОБАВЬТЕ ЭТИ ТРИ СТРОКИ СЮДА ДЛЯ СБРОСА РАМПЫ ---
            #self.current_vx = 0.0
            #self.current_vy = 0.0
            #self.current_omega = 0.0

            self.target_mode = "RELATIVE_MOVE"
            self.status = "MOVING"
            self._publish_telemetry_now()
            response.success = True
            response.status_message = f"Accepted RELATIVE_MOVE: dx={request.dx}, dy={request.dy}"
            return response

    def _srv_stop_callback(self, request, response):
        """Сервис экстренной отмены целей. Отрабатывает за микросекунды."""
        with self.lock:
            # --- ДОБАВЛЕНО: Если робот уже куда-то ехал, логируем вытеснение цели ---
            if self.status == "MOVING":
                self.get_logger().warning(f"Текущая цель {self.target_mode} была принудительно ПРЕРВАНА новой командой!")
                # Если вашему Дереву Поведения обязательно нужен SUCCESS, чтобы переключить шаг, 
                # мы можем временно выставить его здесь перед перезаписью:
                # self.status = "SUCCESS" 
            self.get_logger().info("Цель Стоп.")
            self.target_mode = "IDLE"
            self.status = "IDLE"
            response.success = True
            response.message = "Autopilot successfully stopped and reset to IDLE."
            return response

    # --- ПУБЛИЧНЫЕ МЕТОДЫ ДЛЯ ЛОКАЛЬНОГО ВЫЗОВА (НА СЛУЧАЙ РАБОТЫ В ОДНОМ СТЭКЕ) ---

    def heartbeat(self):
        with self.lock:
            self.last_heartbeat_time = self.get_clock().now()

    def get_current_pose(self) -> Pose:
        with self.lock:
            return Pose(self.current_x, self.current_y, self.current_yaw)

    def _apply_ramp(self, current: float, target: float, max_step: float) -> float:
        error = target - current
        if abs(error) <= max_step:
            return target
        return current + math.copysign(max_step, error)

    # -------------------- ВНУТРЕННИЙ РАБОЧИЙ ЦИКЛ (Таймер 20 Гц) --------------------

    def _control_loop(self):
        """Фоновый исполнитель автопилота. Работает непрерывно в потоке Executor'а."""
        now = self.get_clock().now()
        ideal_vx, ideal_vy, ideal_omega = 0.0, 0.0, 0.0
        print(f"\033[91m[auto loop 1\033[0m", flush=True)

        with self.lock:
            # --- ЗАЩИТА 1: СТОРОЖЕВОЙ ТАЙМЕР ИИ (WATCHDOG) ---
            if self.target_mode != "IDLE" and self.status == "MOVING":
                elapsed_hb_ns = (now - self.last_heartbeat_time).nanoseconds
                if elapsed_hb_ns >= self.watchdog_timeout_ns:
                    self.get_logger().error("КРИТИКА: Потеря связи с ИИ по Сервису! Экстренный стоп.")
                    self.target_mode = "IDLE"
                    self.status = "ABORTED_TIMEOUT"

            if self.target_mode == "IDLE":
                pass

            elif self.target_mode == "TIME_DRIVE":
                elapsed_ns = (now - self.target_start_time).nanoseconds
                if elapsed_ns >= self.target_duration_ns:
                    self.get_logger().info("Цель TIME_DRIVE достигнута по времени. Сброс.")
                    self.target_mode = "IDLE"
                    self.status = "SUCCESS"
                else:
                    ideal_vx, ideal_vy, ideal_omega = self.tgt_vx, self.tgt_vy, self.tgt_omega

            elif self.target_mode == "RELATIVE_MOVE":
                # 1. Вычисляем глобальные координаты целевой точки на полу
                tgt_g_x = self.start_x + self.target_dx * math.cos(self.start_yaw) - self.target_dy * math.sin(self.start_yaw)
                tgt_g_y = self.start_y + self.target_dx * math.sin(self.start_yaw) + self.target_dy * math.cos(self.start_yaw)
                
                # 2. Находим текущую ошибку в глобальных координатах
                global_err_x = tgt_g_x - self.current_x
                global_err_y = tgt_g_y - self.current_y
                
                # 3. Математически точно проецируем глобальную ошибку на ТЕКУЩИЙ курс бампера робота
                error_x = global_err_x * math.cos(self.current_yaw) + global_err_y * math.sin(self.current_yaw)
                error_y = -global_err_x * math.sin(self.current_yaw) + global_err_y * math.cos(self.current_yaw)
                error_yaw = self._normalize_angle((self.start_yaw + self.target_dyaw) - self.current_yaw)
                
                # 4. Проверка допуска финиша
                if abs(error_x) < 0.03 and abs(error_y) < 0.03 and abs(error_yaw) < 0.05:
                    self.get_logger().info("Цель RELATIVE_MOVE успешно достигнута релейно. Сброс.")
                    self.target_mode = "IDLE"
                    self.status = "SUCCESS"
                else:
                    # --- ПОЛНОСТЬЮ ОТКЛЮЧИЛИ ПИД-РЕГУЛЯТОРЫ ---
                    # Вместо формул ПИД выдаем фиксированную безопасную скорость в сторону ошибки (On/Off закон)
                    # Если ошибка положительная — едем вперед (+0.2), если отрицательная — назад (-0.2)
                    
                    if abs(error_x) >= 0.03:
                        ideal_vx = math.copysign(0.2, error_x) # Фиксированная линейная скорость 0.2 м/с
                
                    if abs(error_y) >= 0.03:
                        ideal_vy = math.copysign(0.2, error_y)
                    
                    if abs(error_yaw) >= 0.05:
                        ideal_omega = math.copysign(0.4, error_yaw) # Фиксированная угловая скорость 0.4 рад/с
            # --- ПОЛНОСТЬЮ ОТКЛЮЧИЛИ РАМПУ УСКОРЕНИЙ ---
            # Фактическая скорость теперь мгновенно равна идеальной, без сглаживания
            self.current_vx = ideal_vx
            self.current_vy = ideal_vy
            self.current_omega = ideal_omega
        # Публикация за пределами lock
        msg = Twist()
        msg.linear.x = self.current_vx
        msg.linear.y = self.current_vy
        msg.angular.z = self.current_omega
        self.pub.publish(msg)

        # --- ДОБАВЛЕНО: НЕПРЕРЫВНАЯ ПУБЛИКАЦИЯ ГОТОВОЙ ТЕЛЕМЕТРИИ (20 Гц) ---
        tel_msg = AutopilotTelemetry()
        with self.lock:
            tel_msg.x = self.current_x
            tel_msg.y = self.current_y
            tel_msg.yaw = self.current_yaw
            tel_msg.status = self.status
            
        self.telemetry_pub.publish(tel_msg)

# ... здесь закончился метод _control_loop(self) из Части 2 ...

def main(args=None):
    rclpy.init(args=args)
    
    # Создаем экземпляр нашего автопилота
    node = RobotAutopilot()
    
    try:
        # КРИТИЧЕСКИ ВАЖНО: Этот вызов блокирует поток и заставляет ноду 
        # бесконечно крутить таймеры и подписки, не давая скрипту закрыться!
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Безопасное тушение при выходе
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
