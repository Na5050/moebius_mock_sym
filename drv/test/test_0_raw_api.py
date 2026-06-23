#!/usr/bin/env python3
"""
Тест 0.1: Прямой вызов API драйвера (с правильным ожиданием).
Исправление: ждём SUCCESS от TIME_DRIVE перед MoveRelative.

Запуск: python3 /home/sh/test/test_0_raw_api.py
"""
import sys
sys.path.append('/home/sh')

import rclpy
import time
from driver_lib import RobotDriver

class RawAPITest(RobotDriver):
    def __init__(self):
        super().__init__('test_0_raw_api')
        
    def run_test(self):
        logger = self.get_logger()
        all_passed = True
        
        # === ЧАСТЬ 1: Проверка соединения ===
        logger.info("=" * 50)
        logger.info("ЧАСТЬ 1: Проверка соединения")
        logger.info("=" * 50)
        
        logger.info("Ожидаю телеметрию...")
        timeout = 5.0
        start = time.time()
        while not self.is_connected() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.is_connected():
            logger.error("❌ ЧАСТЬ 1: Телеметрия не получена!")
            all_passed = False
        else:
            logger.info("✅ ЧАСТЬ 1: Телеметрия получена")
            x, y, yaw = self.get_pose()
            logger.info(f"   Поза: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
            logger.info(f"   Статус автопилота: {self.get_status()}")
        
        # === ЧАСТЬ 2: Heartbeat ===
        logger.info("=" * 50)
        logger.info("ЧАСТЬ 2: Heartbeat (10 сообщений)")
        logger.info("=" * 50)
        
        for i in range(10):
            self.send_heartbeat()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        
        rclpy.spin_once(self, timeout_sec=0.5)
        status = self.get_status()
        if "ABORTED" in status:
            logger.error(f"❌ ЧАСТЬ 2: Статус {status} (heartbeat не работает!)")
            all_passed = False
        else:
            logger.info(f"✅ ЧАСТЬ 2: Статус после heartbeat: {status}")
        
        # === ЧАСТЬ 3: Stop ===
        logger.info("=" * 50)
        logger.info("ЧАСТЬ 3: Stop")
        logger.info("=" * 50)
        
        logger.info("Отправляю stop...")
        future = self.stop()
        
        start = time.time()
        while not future.done() and time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        if not future.done():
            logger.error("❌ ЧАСТЬ 3: Stop — таймаут")
            all_passed = False
        else:
            response = future.result()
            if response is None or not response.success:
                logger.error(f"❌ ЧАСТЬ 3: Stop — отклонён")
                all_passed = False
            else:
                logger.info(f"✅ ЧАСТЬ 3: Stop — {response.message}")
        
        # === ВАЖНО: Ждём IDLE после stop ===
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # === ЧАСТЬ 4: MoveTime (ждём реального завершения) ===
        logger.info("=" * 50)
        logger.info("ЧАСТЬ 4: MoveTime — 1 сек вперёд (0.2 м/с)")
        logger.info("=" * 50)
        
        logger.info("Отправляю go_time...")
        future = self.move_time(vx=0.2, vy=0.0, omega=0.0, duration=1.0)
        
        start = time.time()
        while not future.done() and time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        if not future.done():
            logger.error("❌ ЧАСТЬ 4: MoveTime — нет ответа")
            all_passed = False
        else:
            response = future.result()
            if response is None or not response.success:
                logger.error(f"❌ ЧАСТЬ 4: MoveTime — отклонён")
                all_passed = False
            else:
                logger.info(f"✅ ЧАСТЬ 4: MoveTime — {response.status_message}")
                
                # Ждём пока автопилот реально завершит TIME_DRIVE
                logger.info("Ожидаю SUCCESS от автопилота...")
                start = time.time()
                move_completed = False
                while time.time() - start < 5.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    self.send_heartbeat()  # Шлём heartbeat чтобы не было ABORTED_TIMEOUT
                    status = self.get_status()
                    if status == "SUCCESS":
                        logger.info(f"✅ ЧАСТЬ 4: Автопилот завершил TIME_DRIVE (статус: {status})")
                        move_completed = True
                        break
                    elif status == "IDLE":
                        # Могло вытесниться — проверяем через 0.1 сек ещё раз
                        pass
                    time.sleep(0.1)
                
                if not move_completed:
                    logger.error(f"❌ ЧАСТЬ 4: Автопилот не завершил TIME_DRIVE за 5 сек (статус: {self.get_status()})")
                    all_passed = False
        
        # === Ждём стабилизации перед Частью 5 ===
        logger.info("Пауза 1 сек перед следующим тестом...")
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # === ЧАСТЬ 5: MoveRelative ===
        logger.info("=" * 50)
        logger.info("ЧАСТЬ 5: MoveRelative — 0.3м вперёд")
        logger.info("=" * 50)
        
        logger.info("Отправляю go_relative...")
        future = self.move_relative(dx=0.3, dy=0.0, dyaw_deg=0.0)
        
        start = time.time()
        while not future.done() and time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        if not future.done():
            logger.error("❌ ЧАСТЬ 5: MoveRelative — нет ответа")
            all_passed = False
        else:
            response = future.result()
            if response is None or not response.success:
                logger.error(f"❌ ЧАСТЬ 5: MoveRelative — отклонён")
                all_passed = False
            else:
                logger.info(f"✅ ЧАСТЬ 5: MoveRelative — {response.status_message}")
                
                logger.info("Ожидаю SUCCESS от автопилота...")
                start = time.time()
                move_completed = False
                while time.time() - start < 10.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    self.send_heartbeat()
                    status = self.get_status()
                    x, y, yaw = self.get_pose()
                    if status == "SUCCESS":
                        logger.info(f"✅ ЧАСТЬ 5: Автопилот завершил RELATIVE_MOVE (статус: {status})")
                        logger.info(f"   Финальная поза: x={x:.3f}, y={y:.3f}")
                        move_completed = True
                        break
                    elif status != "MOVING":
                        if int(time.time() * 2) % 4 == 0:
                            logger.info(f"   Статус: {status}, x={x:.3f}")
                    time.sleep(0.1)
                
                if not move_completed:
                    logger.error(f"❌ ЧАСТЬ 5: Автопилот не завершил RELATIVE_MOVE (статус: {self.get_status()})")
                    all_passed = False
        
        # === Финальный stop ===
        logger.info("=" * 50)
        self.stop()
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        logger.info(f"Финальный статус: {self.get_status()}")
        
        if all_passed:
            logger.info("✅ ВСЕ ТЕСТЫ ПРОЙДЕНЫ!")
        else:
            logger.error("❌ ТЕСТ ПРОВАЛЕН")
        logger.info("=" * 50)
        
        return all_passed

def main():
    rclpy.init()
    node = RawAPITest()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
