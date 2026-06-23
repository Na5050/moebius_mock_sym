#!/usr/bin/env python3
"""
Тест 3.1: MoveTimeAction в дереве поведения.
Проверяет:
  - MoveTimeAction отправляет go_time
  - Ждёт ответ сервиса
  - Ждёт завершения движения (SUCCESS от автопилота)
  - Возвращает SUCCESS

Запуск: python3 /home/sh/test/test_3_move_time.py
"""
import sys
sys.path.append('/home/sh')

import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        MoveTimeAction, StopAction, Parallel, HeartbeatAction)

class MoveTimeTest(RobotDriver):
    def __init__(self):
        super().__init__('test_3_move_time', tick_rate=10)  # 10 Гц для более быстрой реакции
    
    def build_tree(self):
        return Parallel([
            HeartbeatAction(),  # Шлём heartbeat чтобы автопилот не отключился
            Sequence([
                LogAction("=== ТЕСТ 3.1: MoveTimeAction ==="),
                LogAction("Еду вперёд 1 сек со скоростью 0.2 м/с..."),
                MoveTimeAction(vx=0.2, vy=0.0, omega=0.0, duration=1.0,
                              name="Forward1s", 
                              send_timeout=3.0, 
                              exec_timeout=5.0),
                LogAction("MoveTimeAction завершился SUCCESS!"),
                LogAction("Делаю паузу 0.5 сек..."),
                MoveTimeAction(vx=0.0, vy=0.0, omega=0.0, duration=0.5,
                              name="Pause", exec_timeout=2.0),
                LogAction("Еду назад 0.5 сек..."),
                MoveTimeAction(vx=-0.15, vy=0.0, omega=0.0, duration=0.5,
                              name="Backward", exec_timeout=3.0),
                LogAction("=== ТЕСТ 3.1 ЗАВЕРШЁН ==="),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(MoveTimeTest())

if __name__ == '__main__':
    main()
