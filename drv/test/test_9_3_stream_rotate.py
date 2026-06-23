#!/usr/bin/env python3
"""
Тест 9.3: StreamAction — робот крутится (не застревание).
Проверяет: робот вращается на месте (меняется yaw),
StreamAction НЕ выводит предупреждение о застревании.

Запуск: python3 /home/sh/test/test_9_3_stream_rotate.py
"""
import sys
sys.path.append('/home/sh')
import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        StreamAction, Parallel, HeartbeatAction)

class StreamRotateTest(RobotDriver):
    def __init__(self):
        super().__init__('test_9_3_stream', tick_rate=5)
    
    def build_tree(self):
        def rotate_and_scan(driver):
            """Крутится на месте, сканируя пространство"""
            x, y, yaw = driver.get_pose()
            front = driver.get_lidar_front(30)
            driver.get_logger().info(
                f"🔄 Сканирую: yaw={yaw:.1f}°, front={front:.2f}м",
                throttle_duration_sec=0.5
            )
            return (0.0, 0.0, 0.3,4.3)  # Только вращение
        
        def found_obstacle(driver):
            """Остановка если нашли препятствие сзади"""
            return driver.get_lidar_back(20) < 0.1
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=" * 50),
                LogAction("ТЕСТ 9.3: Вращение на месте"),
                LogAction("Робот крутится — НЕ застревание!"),
                LogAction("Предупреждений быть НЕ должно"),
                LogAction("Остановится когда сзади препятствие < 0.5м"),
                LogAction("=" * 50),
                
                StreamAction(
                    command_func=rotate_and_scan,
                    stop_condition=found_obstacle,
                    name="RotateStream"
                ),
                
                LogAction("✅ Вращение завершено (найдено препятствие)"),
                LogAction("=" * 50),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(StreamRotateTest())

if __name__ == '__main__':
    main()
