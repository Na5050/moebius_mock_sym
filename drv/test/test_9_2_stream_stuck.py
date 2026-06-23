#!/usr/bin/env python3
"""
Тест 9.2: StreamAction — детектор застревания.
Проверяет: робот стоит на месте (одометрия не меняется),
StreamAction выводит предупреждение о застревании.

Запуск: python3 /home/sh/test/test_9_2_stream_stuck.py
"""
import sys
sys.path.append('/home/sh')
import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        StreamAction, Parallel, HeartbeatAction)

class StreamStuckTest(RobotDriver):
    def __init__(self):
        super().__init__('test_9_2_stream', tick_rate=5)
    
    def build_tree(self):
        # Намеренно возвращаем нулевую скорость
        def fake_stuck(driver):
            """Имитация застревания: робот не двигается"""
            x, y, yaw = driver.get_pose()
            driver.get_logger().info(
                f"🔒 Стою на месте: ({x:.2f}, {y:.2f})",
                throttle_duration_sec=0.5
            )
            return (0.0, 0.0, 0.0, 0.3)  # Всегда ноль
        
        def never_stop(driver):
            """Условие никогда не выполняется"""
            return False
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=" * 50),
                LogAction("ТЕСТ 9.2: Детектор застревания"),
                LogAction("Робот будет стоять 5 секунд..."),
                LogAction("Ожидаю предупреждения о застревании"),
                LogAction("=" * 50),
                
                # Этот StreamAction должен выдать WARNING
                StreamAction(
                    command_func=fake_stuck,
                    stop_condition=never_stop,
                    name="StuckStream",
                    stuck_xy_threshold=0.02,   # 2 см/сек
                    stuck_yaw_threshold=0.05   # ~3°/сек
                ),
                
                LogAction("❌ Не должно дойти (бесконечный цикл)"),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    node = StreamStuckTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Тест прерван пользователем")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
