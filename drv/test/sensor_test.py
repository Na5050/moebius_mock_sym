#!/usr/bin/env python3
"""Тестирование геттеров лидара и камеры без движения."""
import sys
sys.path.append('/home/sh')
import rclpy
from driver_lib import RobotDriver, LogAction, EvalAction, Sequence

class SensorTest(RobotDriver):
    def __init__(self):
        super().__init__('sensor_test', tick_rate=2)

    def build_tree(self):
        return Sequence([
            LogAction("Тест лидара и камеры"),
            EvalAction(lambda d: f"Минимум: {d.get_lidar_min_distance():.2f} м", "LidarMin"),
            EvalAction(lambda d: f"Спереди: {d.get_lidar_front():.2f} м", "LidarFront"),
            EvalAction(lambda d: f"Слева:   {d.get_lidar_left():.2f} м", "LidarLeft"),
            EvalAction(lambda d: f"Справа:  {d.get_lidar_right():.2f} м", "LidarRight"),
            EvalAction(lambda d: f"Сзади:   {d.get_lidar_back():.2f} м", "LidarBack"),
            EvalAction(lambda d: f"Секторы: {d.get_lidar_sectors()}", "LidarSectors"),
            EvalAction(lambda d: f"Камера:  {'OK' if d.get_frame() is not None else 'нет кадра'}", "Camera"),
        ])

def main():
    rclpy.init()
    RobotDriver.spin(SensorTest())

if __name__ == '__main__':
    main()
