#!/usr/bin/env python3
"""
Тест геттеров лидара с препятствием: поиск скачка дистанции.
Поставь коробку перед роботом на расстоянии ~0.5м.
"""
import sys
sys.path.append('/home/sh')
import rclpy
import math
from driver_lib import (RobotDriver, Sequence, StreamAction,
                         EvalAction, LogAction, Parallel, HeartbeatAction)

class TestLidarWithObstacle(RobotDriver):
    def __init__(self):
        super().__init__('test_lidar_obstacle', tick_rate=5)
    
    def build_tree(self):
        
        def log_odom(driver, tag=""):
            x, y, yaw = driver.get_pose()
            driver.get_logger().info(f"[{tag}] 📍 x={x:.2f}м y={y:.2f}м yaw={math.degrees(yaw):.0f}°")
            return tag
        
        def analyze_lidar(driver):
            """Полный анализ лидара с поиском скачков."""
            ranges = driver.get_lidar_ranges()
            angle_min = driver.get_lidar_angle_min()
            angle_inc = driver.get_lidar_angle_increment()
            
            if not ranges:
                driver.get_logger().warn("❌ Лидар не готов")
                return "Лидар не готов"
            
            front = driver.get_lidar_front(26)
            right = driver.get_lidar_right(20)
            left = driver.get_lidar_left(20)
            
            driver.get_logger().info(f"📡 front(26°)={front:.2f}м right(20°)={right:.2f}м left(20°)={left:.2f}м")
            driver.get_logger().info(f"📡 Всего замеров: {len(ranges)}, углы: {math.degrees(angle_min):.0f}°..{math.degrees(angle_min + len(ranges)*angle_inc):.0f}°")
            
            # Ищем ВСЕ скачки в данных
            jumps = []
            for i in range(0, len(ranges) - 1):
                if ranges[i] > 0.1 and ranges[i + 1] > 0.1:
                    diff = abs(ranges[i + 1] - ranges[i])
                    if diff > 0.3:  # Скачок > 30 см
                        angle = angle_min + i * angle_inc
                        jumps.append((math.degrees(angle), ranges[i], ranges[i+1], diff))
            
            if jumps:
                driver.get_logger().info(f"📡 Найдено скачков: {len(jumps)}")
                for angle, r1, r2, diff in jumps[:5]:  # Покажем первые 5
                    driver.get_logger().info(f"  📡 {angle:+.0f}°: {r1:.2f}м → {r2:.2f}м (Δ={diff:.2f}м)")
            else:
                driver.get_logger().info("📡 Скачков нет — всё ровно")
            
            # Покажем сектора по 10°
            driver.get_logger().info("📡 Сектора:")
            for deg in range(-90, 91, 15):
                idx = int((math.radians(deg) - angle_min) / angle_inc)
                if 0 <= idx < len(ranges):
                    bar = "█" * min(int(ranges[idx] * 5), 20)
                    driver.get_logger().info(f"  {deg:+4}°: {ranges[idx]:.2f}м {bar}")
            
            return "Анализ завершён"
        
        def drive_to_obstacle(driver):
            """Едем пока не увидим препятствие."""
            return (0.1, 0.0, 0.0, 0.3)
        
        def obstacle_nearby(driver):
            front = driver.get_lidar_front(20)
            if front <= 0.5:
                driver.get_logger().info(f"🛑 Препятствие на {front:.2f}м!")
                return True
            return False
        
        return Parallel([
            HeartbeatAction(),
            Sequence([
                LogAction("=" * 60),
                LogAction("🧪 ТЕСТ ГЕТТЕРОВ ЛИДАРА С ПРЕПЯТСТВИЕМ"),
                EvalAction(lambda d: log_odom(d, "СТАРТ"), "Start"),
                LogAction("Поставь коробку перед роботом на ~0.5м"),
                LogAction("=" * 60),
                
                # Анализ ДО движения
                LogAction("▶️  Анализ лидара ДО движения:"),
                EvalAction(analyze_lidar, "Before"),
                LogAction(""),
                
                # Едем к препятствию
                LogAction("▶️  Движение к препятствию..."),
                StreamAction(
                    command_func=drive_to_obstacle,
                    stop_condition=obstacle_nearby,
                    name="Approach"
                ),
                EvalAction(lambda d: log_odom(d, "У ПРЕПЯТСТВИЯ"), "AtObstacle"),
                LogAction(""),
                
                # Анализ ПОСЛЕ
                LogAction("▶️  Анализ лидара У ПРЕПЯТСТВИЯ:"),
                EvalAction(analyze_lidar, "After"),
                LogAction(""),
                
                LogAction("=" * 60),
                LogAction("✅ ТЕСТ ЗАВЕРШЁН"),
                LogAction("=" * 60),
            ]),
        ], required_successes=1)

def main():
    rclpy.init()
    RobotDriver.spin(TestLidarWithObstacle())

if __name__ == '__main__':
    main()
