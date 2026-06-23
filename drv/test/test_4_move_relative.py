#!/usr/bin/env python3
"""
Тест 4.1: MoveRelativeAction в дереве поведения.
Проверяет:
  - Проезд вперёд на 0.5м
  - Поворот на 90°
  - Возврат в исходную позицию

Запуск: python3 /home/sh/test/test_4_move_relative.py
"""
import sys
sys.path.append('/home/sh')

import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        MoveRelativeAction, Parallel, HeartbeatAction)

class MoveRelativeTest(RobotDriver):
    def __init__(self):
        super().__init__('test_4_move_relative', tick_rate=10)
    
    def build_tree(self):
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=== ТЕСТ 4.1: MoveRelativeAction ==="),
                
                LogAction("1/4: Еду вперёд 0.5м..."),
                MoveRelativeAction(dx=0.5, dy=0.0, dyaw_deg=0.0,
                                  name="Forward05", exec_timeout=10.0),
                LogAction("   ✅ Проезд завершён"),
                
                LogAction("2/4: Поворот на 90°..."),
                MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0,
                                  name="Rotate90", exec_timeout=10.0),
                LogAction("   ✅ Поворот завершён"),
                
                LogAction("3/4: Еду вперёд 0.3м..."),
                MoveRelativeAction(dx=0.3, dy=0.0, dyaw_deg=0.0,
                                  name="Forward03", exec_timeout=10.0),
                LogAction("   ✅ Проезд завершён"),
                
                LogAction("4/4: Поворот на -90° (обратно)..."),
                MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=-90.0,
                                  name="RotateBack", exec_timeout=10.0),
                LogAction("   ✅ Возврат завершён"),
                
                LogAction("=== ТЕСТ 4.1 ЗАВЕРШЁН ==="),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(MoveRelativeTest())

if __name__ == '__main__':
    main()
