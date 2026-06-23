#!/usr/bin/env python3
"""
Тест 2.1: StopAction в дереве поведения.
Проверяет:
  - StopAction отправляет stop и ждёт ответ
  - StopAction возвращает SUCCESS
  - Дерево завершается корректно

Запуск: python3 /home/sh/test/test_2_stop_action.py
"""
import sys
sys.path.append('/home/sh')

import rclpy
from driver_lib import RobotDriver, LogAction, Sequence, StopAction

class StopActionTest(RobotDriver):
    def __init__(self):
        super().__init__('test_2_stop_action', tick_rate=5)
    
    def build_tree(self):
        return Sequence([
            LogAction("=== ТЕСТ 2.1: StopAction ==="),
            LogAction("Отправляю stop через StopAction..."),
            StopAction(name="TestStop", send_timeout=3.0),
            LogAction("StopAction вернул SUCCESS!"),
        ])

def main():
    rclpy.init()
    RobotDriver.spin(StopActionTest())

if __name__ == '__main__':
    main()
