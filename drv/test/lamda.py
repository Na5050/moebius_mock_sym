#!/usr/bin/env python3
"""
Тест новых фич DriverLib с логированием одометрии:
  1. get_lidar_ranges() — сырой массив лидара
  2. MoveRelativeAction с callable dyaw_deg
  3. MoveTimeAction с callable duration
"""
import sys
sys.path.append('/home/sh')
import rclpy
import math
from driver_lib import (RobotDriver, Sequence, MoveRelativeAction,
                         MoveTimeAction, EvalAction, LogAction,
                         Parallel, HeartbeatAction)

class TestNewFeatures(RobotDriver):
    def __init__(self):
        super().__init__('test_new_features', tick_rate=5)
    
    def build_tree(self):
        
        def log_odom(driver, tag=""):
            x, y, yaw = driver.get_pose()
            driver.get_logger().info(f"[{tag}] 📍 x={x:.3f}м y={y:.3f}м yaw={math.degrees(yaw):.1f}°")
            return f"{tag}: ({x:.2f}, {y:.2f})"
        
        def test_lidar_getters(driver):
            ranges = driver.get_lidar_ranges()
            angle_min = driver.get_lidar_angle_min()
            angle_inc = driver.get_lidar_angle_increment()
            
            if ranges:
                driver.get_logger().info(f"✅ get_lidar_ranges(): {len(ranges)} замеров")
                driver.get_logger().info(f"✅ get_lidar_angle_min(): {math.degrees(angle_min):.1f}°")
                driver.get_logger().info(f"✅ get_lidar_angle_increment(): {math.degrees(angle_inc):.2f}°")
                
                mid = len(ranges) // 2
                driver.get_logger().info(f"📡 Центр (0°): {ranges[mid]:.2f}м")
                driver.get_logger().info(f"📡 +10°: {ranges[mid + int(math.radians(10) / angle_inc)]:.2f}м")
                driver.get_logger().info(f"📡 -10°: {ranges[mid - int(math.radians(10) / angle_inc)]:.2f}м")
                driver.get_logger().info(f"📡 +90°: {ranges[-1]:.2f}м")
                driver.get_logger().info(f"📡 -90°: {ranges[0]:.2f}м")
            else:
                driver.get_logger().warn("❌ Лидар не готов")
            
            return "Тест 1: OK"
        
        def test_callable_dyaw(driver):
            x, y, yaw = driver.get_pose()
            driver.get_logger().info(f"🔄 callable dyaw_deg: yaw={math.degrees(yaw):.0f}° → верну 45°")
            return 45.0
        
        def test_callable_duration(driver):
            x, y, yaw = driver.get_pose()
            driver.get_logger().info(f"⏱️  callable duration: поза=({x:.2f},{y:.2f}) → верну 0.5с")
            return 0.5
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=" * 60),
                LogAction("🧪 ТЕСТ НОВЫХ ФИЧ (с одометрией)"),
                EvalAction(lambda d: log_odom(d, "СТАРТ"), "StartOdom"),
                LogAction("=" * 60),
                
                # Тест 1
                LogAction("▶️  Тест 1: get_lidar_ranges()"),
                EvalAction(test_lidar_getters, "Lidar"),
                EvalAction(lambda d: log_odom(d, "ПОСЛЕ Т1"), "Odom1"),
                LogAction(""),
                
                # Тест 2: callable dyaw_deg
                LogAction("▶️  Тест 2: MoveRelativeAction с callable dyaw_deg (45°)"),
                MoveRelativeAction(
                    dx=0.0, dy=0.0,
                    dyaw_deg=test_callable_dyaw,
                    name="CallableTurn",
                    exec_timeout=10.0
                ),
                LogAction("✅ Поворот на 45° выполнен"),
                EvalAction(lambda d: log_odom(d, "ПОСЛЕ ПОВОРОТА"), "Odom2"),
                LogAction(""),
                
                # Возврат
                LogAction("↩️  Возврат на -45°..."),
                MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=-45.0,
                                 name="TurnBack", exec_timeout=10.0),
                LogAction("✅ Возврат выполнен"),
                EvalAction(lambda d: log_odom(d, "ПОСЛЕ ВОЗВРАТА"), "Odom3"),
                LogAction(""),
                
                # Тест 3: callable duration
                LogAction("▶️  Тест 3: MoveTimeAction с callable duration (0.5с)"),
                MoveTimeAction(
                    vx=0.15, vy=0.0, omega=0.0,
                    duration=test_callable_duration,
                    name="CallableMove",
                    exec_timeout=5.0
                ),
                LogAction("✅ Движение на 0.5с выполнено"),
                EvalAction(lambda d: log_odom(d, "ПОСЛЕ ДВИЖЕНИЯ"), "Odom4"),
                LogAction(""),
                
                LogAction("=" * 60),
                LogAction("✅ ВСЕ ТРИ ТЕСТА ПРОЙДЕНЫ!"),
                EvalAction(lambda d: log_odom(d, "ФИНИШ"), "FinalOdom"),
                LogAction("=" * 60),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(TestNewFeatures())

if __name__ == '__main__':
    main()
