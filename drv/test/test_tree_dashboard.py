#!/usr/bin/env python3
"""Тест TreeDashboardNode: дерево переключается минуту, робот стоит."""
import math
import time
import rclpy
from driver_lib import (RobotDriver, Sequence, Fallback, Parallel,
                         HeartbeatAction, EvalAction, LogAction, RepeatUntil,
                         Condition, MoveRelativeAction, TreeDashboardNode)


class TreeDashboardTest(RobotDriver):

    def __init__(self):
        super().__init__('tree_dashboard_test', tick_rate=2)
        self.add_node(TreeDashboardNode(self, port=8079))

    def build_tree(self):

        def fake_condition(driver):
            """Переключается каждые 5 секунд для демонстрации."""
            elapsed = time.time() - driver.bb.get('start_time', time.time())
            return (int(elapsed) // 5) % 2 == 0  # True 5с, False 5с

        def log_phase(driver):
            elapsed = time.time() - driver.bb.get('start_time', time.time())
            phase = (int(elapsed) // 5) % 2
            return f"Фаза {phase} (прошло {elapsed:.0f}с)"

        def minute_passed(driver):
            elapsed = time.time() - driver.bb.get('start_time', time.time())
            return elapsed >= 60.0

        return Sequence([
                EvalAction(lambda d: "🌳 Тест дашборда дерева (60 сек)", "Start"),
                EvalAction(lambda d: d.bb.set('start_time', time.time()) or "⏱️ Старт", "Init"),

                RepeatUntil(
                    child=Sequence([
                        EvalAction(log_phase, "Phase"),

                        Fallback([
                            # Ветка А: условие True
                            Sequence([
                                Condition(fake_condition),
                                LogAction("✅ Ветка А активна"),
                                EvalAction(lambda d: "  ├─ Подветка А1", "A1"),
                                EvalAction(lambda d: "  └─ Подветка А2", "A2"),
                            ]),
                            # Ветка Б: условие False
                            Sequence([
                                LogAction("❌ Ветка Б активна"),
                                EvalAction(lambda d: "  ├─ Подветка Б1", "B1"),
                                EvalAction(lambda d: "  ├─ Подветка Б2", "B2"),
                                EvalAction(lambda d: "  └─ Подветка Б3", "B3"),
                            ]),
                        ]),
                    ]),
                    stop_condition=minute_passed,
                    name="MainLoop"
                ),

                EvalAction(lambda d: "🏁 Тест завершён", "Finish"),
            ])


def main():
    rclpy.init()
    RobotDriver.spin(TreeDashboardTest())


if __name__ == '__main__':
    main()
