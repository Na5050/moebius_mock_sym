#!/usr/bin/env python3
"""
Тест 9.4: StreamAction — предупреждение о мёртвой зоне.
Проверяет: разные пороги движения и остановки создают 
мёртвую зону, StreamAction предупреждает.

Запуск: python3 /home/sh/test/test_9_4_stream_deadzone.py
"""
import sys
sys.path.append('/home/sh')
import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        StreamAction, Parallel, HeartbeatAction)

class StreamDeadZoneTest(RobotDriver):
    def __init__(self):
        super().__init__('test_9_4_stream', tick_rate=5)
    
    def build_tree(self):
        MOVE_THRESHOLD = 0.5   # Едем если дальше 50 см
        STOP_THRESHOLD = 0.3   # Стоп если ближе 30 см
        # Мёртвая зона: 0.3-0.5 метра
        
        def forward_with_deadzone(driver):
            front = driver.get_lidar_front(20)
            vx = 0.1 if front > MOVE_THRESHOLD else 0.0
            driver.get_logger().info(
                f"📡 front={front:.2f}м, vx={vx:.2f} "
                f"[мёртвая зона: {STOP_THRESHOLD}-{MOVE_THRESHOLD}м]",
                throttle_duration_sec=0.5
            )
            return (vx, 0.0, 0.0, 0.3)
        
        def obstacle_close(driver):
            return driver.get_lidar_front(20) <= STOP_THRESHOLD
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=" * 50),
                LogAction("ТЕСТ 9.4: Детектор мёртвой зоны"),
                LogAction(f"Порог движения: {MOVE_THRESHOLD}м"),
                LogAction(f"Порог остановки: {STOP_THRESHOLD}м"),
                LogAction(f"⚠️  Мёртвая зона: {STOP_THRESHOLD}-{MOVE_THRESHOLD}м"),
                LogAction("Поставь препятствие в мёртвой зоне (~40см)"),
                LogAction("Ожидаю предупреждение о застревании"),
                LogAction("(Нажми Ctrl+C для выхода)"),
                LogAction("=" * 50),
                
                StreamAction(
                    command_func=forward_with_deadzone,
                    stop_condition=obstacle_close,
                    name="DeadZoneStream"
                ),
                
                LogAction("❌ Не должно дойти (зависнет в мёртвой зоне)"),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    node = StreamDeadZoneTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Тест прерван пользователем")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
