#!/usr/bin/env python3
"""
ТЕСТ 7: Интеграционный тест — Квадрат 0.5м с отладкой.
Проверяет полный цикл:
  - Heartbeat в параллель
  - 4 движения вперёд + 4 поворота
  - Retry на прямых участках
  - Логирование одометрии на каждом шаге

Запуск: python3 /home/sh/test/test_7_square.py
"""
import sys
sys.path.append('/home/sh')

import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        MoveRelativeAction, RetryAction, Parallel, HeartbeatAction, EvalAction)

class SquareIntegrationTest(RobotDriver):
    def __init__(self):
        super().__init__('test_7_square', tick_rate=10)
    
    def build_tree(self):
        side = 0.5  # Длина стороны квадрата в метрах
        
        # Вспомогательная функция для логирования позы
        def log_pose(driver, label=""):
            x, y, yaw = driver.get_pose()
            driver.get_logger().info(f"📍 {label}: x={x:.3f}, y={y:.3f}, yaw={yaw:.2f}°")
            return f"Pose: ({x:.2f}, {y:.2f})"
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=" * 50),
                LogAction("🏁 СТАРТ КВАДРАТА"),
                LogAction(f"📐 Сторона: {side}м"),
                EvalAction(lambda d: log_pose(d, "Старт"), "StartPose"),
                LogAction("=" * 50),
                
                # Сторона 1
                LogAction("▶️ Сторона 1/4: вперёд"),
                RetryAction(
                    MoveRelativeAction(dx=side, dy=0.0, dyaw_deg=0.0,
                                     name="Side1", exec_timeout=10.0),
                    max_attempts=2
                ),
                EvalAction(lambda d: log_pose(d, "После стороны 1"), "Pose1"),
                
                # Поворот 1
                LogAction("↻ Поворот 1/4: +90°"),
                MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0,
                                 name="Turn1", exec_timeout=10.0),
                EvalAction(lambda d: log_pose(d, "После поворота 1"), "Pose2"),
                
                # Сторона 2
                LogAction("▶️ Сторона 2/4: вперёд"),
                RetryAction(
                    MoveRelativeAction(dx=side, dy=0.0, dyaw_deg=0.0,
                                     name="Side2", exec_timeout=10.0),
                    max_attempts=2
                ),
                EvalAction(lambda d: log_pose(d, "После стороны 2"), "Pose3"),
                
                # Поворот 2
                LogAction("↻ Поворот 2/4: +90°"),
                MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0,
                                 name="Turn2", exec_timeout=10.0),
                EvalAction(lambda d: log_pose(d, "После поворота 2"), "Pose4"),
                
                # Сторона 3
                LogAction("▶️ Сторона 3/4: вперёд"),
                RetryAction(
                    MoveRelativeAction(dx=side, dy=0.0, dyaw_deg=0.0,
                                     name="Side3", exec_timeout=10.0),
                    max_attempts=2
                ),
                EvalAction(lambda d: log_pose(d, "После стороны 3"), "Pose5"),
                
                # Поворот 3
                LogAction("↻ Поворот 3/4: +90°"),
                MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0,
                                 name="Turn3", exec_timeout=10.0),
                EvalAction(lambda d: log_pose(d, "После поворота 3"), "Pose6"),
                
                # Сторона 4
                LogAction("▶️ Сторона 4/4: вперёд"),
                RetryAction(
                    MoveRelativeAction(dx=side, dy=0.0, dyaw_deg=0.0,
                                     name="Side4", exec_timeout=10.0),
                    max_attempts=2
                ),
                EvalAction(lambda d: log_pose(d, "После стороны 4"), "Pose7"),
                
                # Поворот 4 (возврат к исходному курсу)
                LogAction("↻ Поворот 4/4: +90° (возврат)"),
                MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0,
                                 name="Turn4", exec_timeout=10.0),
                EvalAction(lambda d: log_pose(d, "ФИНАЛ"), "FinalPose"),
                
                LogAction("=" * 50),
                LogAction("✅ КВАДРАТ ПРОЙДЕН УСПЕШНО!"),
                LogAction("=" * 50),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(SquareIntegrationTest())

if __name__ == '__main__':
    main()
