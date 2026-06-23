#!/usr/bin/env python3
"""
Тест 8: Fallback (селектор "ИЛИ").
Проверяет:
  - Первый ребёнок FAILURE → переход ко второму
  - Первый ребёнок SUCCESS → остановка (второй не выполняется)
  - Fallback возвращает FAILURE только если ВСЕ дети FAILURE

Сценарии:
  1. Condition (нет препятствия) → SUCCESS → Fallback завершается
  2. Condition (есть препятствие) → FAILURE → MoveTimeAction → SUCCESS

Запуск: python3 /home/sh/test/test_8_fallback.py
"""
import sys
sys.path.append('/home/sh')

import rclpy
from driver_lib import (RobotDriver, LogAction, Sequence, 
                        MoveTimeAction, Fallback, Condition, Parallel, HeartbeatAction)

class FallbackTest(RobotDriver):
    def __init__(self):
        super().__init__('test_8_fallback', tick_rate=5)
    
    def build_tree(self):
        # Сценарий 1: Препятствия нет → Fallback завершается на Condition
        def no_obstacle(driver):
            """Условие: впереди свободно (> 1м)"""
            front = driver.get_lidar_front(20)
            driver.get_logger().info(f"🔍 Condition: front={front:.2f}м > 1.0м? {front > 1.0}")
            return front > 1.0
        
        # Сценарий 2: Препятствие есть → FAILURE → едем вперёд
        def obstacle_present(driver):
            """Условие: впереди препятствие (< 0.5м)"""
            front = driver.get_lidar_front(20)
            driver.get_logger().info(f"🔍 Condition: front={front:.2f}м < 0.5м? {front < 0.5}")
            return front < 0.5
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=" * 50),
                LogAction("ТЕСТ 8.1: Fallback — нет препятствия"),
                LogAction("=" * 50),
                
                # Сценарий 1: Если нет препятствия → сразу SUCCESS
                Fallback([
                    Condition(no_obstacle),  # Если свободно → SUCCESS
                    MoveTimeAction(vx=0.2, vy=0.0, omega=0.0, duration=1.0,
                                 name="ShouldNotRun", exec_timeout=3.0),
                ]),
                
                LogAction("✅ Тест 8.1 пройден (Fallback завершился на Condition)"),
                
                LogAction("=" * 50),
                LogAction("ТЕСТ 8.2: Fallback — с препятствием"),
                LogAction("Поставь препятствие перед роботом на ~30см"),
                LogAction("=" * 50),
                
                # Сценарий 2: Если есть препятствие → FAILURE → запускаем движение
                Fallback([
                    Condition(obstacle_present),  # Если препятствие → SUCCESS
                    LogAction("⚠️ Препятствия нет (Condition FAILURE), еду вперёд 1 сек..."),
                    MoveTimeAction(vx=0.2, vy=0.0, omega=0.0, duration=1.0,
                                 name="AlternativeAction", exec_timeout=5.0),
                ]),
                
                LogAction("✅ Тест 8.2 пройден (Fallback отработал сценарий с препятствием)"),
                
                LogAction("=" * 50),
                LogAction("ТЕСТ 8.3: Fallback — все дети FAILURE"),
                LogAction("=" * 50),
                
                # Сценарий 3: Все условия ложны → FAILURE
                Fallback([
                    Condition(lambda d: False),  # Всегда FAILURE
                    Condition(lambda d: False),  # Всегда FAILURE
                ]),
                
                LogAction("❌ ЭТА СТРОКА НЕ ДОЛЖНА ПОЯВИТЬСЯ (Fallback вернул FAILURE)"),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(FallbackTest())

if __name__ == '__main__':
    main()
