#!/usr/bin/env python3
"""
Тест 5.1: RetryAction при недостижимой цели.
Проверяет:
  - MoveRelativeAction возвращает FAILURE при таймауте
  - RetryAction повторяет 2 раза
  - После исчерпания попыток возвращает FAILURE

Запуск: python3 /home/sh/test/test_5_retry.py
"""
import sys
sys.path.append('/home/sh')

import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        MoveRelativeAction, RetryAction, Parallel, HeartbeatAction)

class RetryTest(RobotDriver):
    def __init__(self):
        super().__init__('test_5_retry', tick_rate=10)
    
    def build_tree(self):
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=== ТЕСТ 5.1: RetryAction ==="),
                LogAction("Пытаюсь проехать 5м за 3 сек (невозможно)..."),
                LogAction("Ожидаю 2 попытки, затем FAILURE..."),
                
                RetryAction(
                    MoveRelativeAction(dx=5.0, dy=0.0, dyaw_deg=0.0,
                                     name="Impossible5m", exec_timeout=3.0),
                    max_attempts=2
                ),
                
                LogAction("ЭТА СТРОКА НЕ ДОЛЖНА ПОЯВИТЬСЯ (RetryAction вернул FAILURE)"),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(RetryTest())

if __name__ == '__main__':
    main()
