#!/usr/bin/env python3
"""
Тест 9.1: StreamAction — нормальное движение до препятствия.
Проверяет: робот едет, видит препятствие, останавливается.

Запуск: python3 /home/sh/test/test_9_1_stream_normal.py
"""
import sys
sys.path.append('/home/sh')
import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        StreamAction, Parallel, HeartbeatAction)

class StreamNormalTest(RobotDriver):
    def __init__(self):
        super().__init__('test_9_1_stream', tick_rate=5)
    
    def build_tree(self):
        STOP_DIST = 0.3  # Единый порог
        
        def forward_until_obstacle(driver):
            front = driver.get_lidar_front(20)
            vx = 0.1 if front > STOP_DIST else 0.0
            driver.get_logger().info(
                f"📡 front={front:.2f}м, vx={vx:.2f}",
                throttle_duration_sec=0.5
            )
            return (vx, 0.0, 0.0, 0.3)
        
        def obstacle_close(driver):
            return driver.get_lidar_front(20) <= STOP_DIST
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=" * 50),
                LogAction("ТЕСТ 9.1: Нормальное движение"),
                LogAction(f"Порог остановки: {STOP_DIST}м"),
                LogAction("Поставь препятствие перед роботом"),
                LogAction("=" * 50),
                
                StreamAction(
                    command_func=forward_until_obstacle,
                    stop_condition=obstacle_close,
                    name="NormalStream"
                ),
                
                LogAction("✅ Остановлен перед препятствием"),
                LogAction("=" * 50),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(StreamNormalTest())

if __name__ == '__main__':
    main()
