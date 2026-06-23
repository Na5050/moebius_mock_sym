#!/usr/bin/env python3
"""
Тест 6.2: StreamAction — исправленная логика.
Проблема: робот зависал между порогом движения (0.5м) и порогом остановки (0.4м)
Решение: 
  - Едем пока расстояние > порог остановки
  - Завершаем когда расстояние <= порог остановки

Запуск: python3 /home/sh/test/test_6_stream.py
"""
import sys
sys.path.append('/home/sh')

import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        StreamAction, Parallel, HeartbeatAction)

class StreamTest(RobotDriver):
    def __init__(self):
        super().__init__('test_6_stream', tick_rate=5)
    
    def build_tree(self):
        STOP_DISTANCE = 0.3  # Остановка когда препятствие ближе 30 см
        
        def follow_forward(driver):
            front = driver.get_lidar_front(20)
            
            # Едем пока расстояние больше порога остановки
            if front > STOP_DISTANCE:
                vx = 0.1  # Медленно для безопасности
            else:
                vx = 0.0  # Остановка (условие завершения сработает)
            
            x, y, yaw = driver.get_pose()
            driver.get_logger().info(
                f"📡 front={front:.2f}м | vx={vx:.2f} | "
                f"pose=({x:.2f},{y:.2f}) | status={driver.get_status()}",
                throttle_duration_sec=0.5
            )
            return (vx, 0.0, 0.0, 0.3)
        
        def obstacle_close(driver):
            """Остановка когда препятствие ближе порога."""
            front = driver.get_lidar_front(20)
            if front <= STOP_DISTANCE:
                driver.get_logger().info(f"🛑 Препятствие! front={front:.2f}м <= {STOP_DISTANCE}м")
                return True
            return False
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=== ТЕСТ 6.2: StreamAction (исправленный) ==="),
                LogAction(f"Еду вперёд до препятствия < {STOP_DISTANCE}м"),
                LogAction("(Поставь коробку перед роботом)"),
                
                StreamAction(command_func=follow_forward,
                            stop_condition=obstacle_close,
                            name="FollowUntilObstacle"),
                
                LogAction("✅ Остановлен перед препятствием!"),
                LogAction("=== ТЕСТ 6.2 ЗАВЕРШЁН ==="),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(StreamTest())

if __name__ == '__main__':
    main()
