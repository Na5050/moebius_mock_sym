# Файл: roslessros/camera_viewer.py
#Copyright (c) 2026 Jurij Polikarpov
import sys
import os


# === ТОЧЕЧНОЕ ИСПРАВЛЕНИЕ ДЛЯ LINUX: Глушим варнинги шрифтов Qt ===
os.environ["QT_LOGGING_RULES"] = "*.warning=false;qt.qpa.fonts.warning=false"
# Или жесткий вариант подавления вывода Qt:
os.environ["QT_QUIET"] = "1"

import time
import queue  # Встроенная библиотека очередей

current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_dir)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraViewerNode(Node):
    # Явно указываем два аргумента: self и frame_queue
    def __init__(self, frame_queue):
        super().__init__('camera_viewer_node')
        self.queue = frame_queue
        self.frame_count = 0
        
        # Подписываемся на стандартный топик камеры
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        print("[Viewer] Сетевой слушатель камеры активирован. Ожидаю кадры...", flush=True)

# Отредактируйте метод image_callback в файле roslessros/camera_viewer.py

    def image_callback(self, msg):
        if not msg.data:
            return
            
        self.frame_count += 1
#         if self.frame_count % 30 == 0:
#             print(f"[Viewer] Кадров получено из сети: {self.frame_count}", flush=True)
            
        try:
            # Превращаем пришедшие байты в одномерный массив NumPy
            img_array = np.frombuffer(msg.data, dtype=np.uint8)
            
            # ОПРЕДЕЛЯЕМ РЕАЛЬНОЕ РАЗРЕШЕНИЕ:
            # Если симулятор прислал 0, берем стандартное разрешение вашей ноды Moebius (320x240)
            h = msg.height if msg.height > 0 else 240
            w = msg.width if msg.width > 0 else 320
            
            # --- ХАК АВТОДЕТЕКЦИИ СЖАТИЯ ---
            if len(msg.data) > 4 and msg.data[0:4] in [b'\xff\xd8\xff\xe0', b'\xff\xd8\xff\xe1', b'\x89PNG']:
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            else:
                # ВАРИАНТ 2: Сырой несжатый поток (теперь с гарантированной защитой от 0)
                img = img_array.reshape((h, w, 3))
                # Если симулятор шлет rgb8, переводим в понятный для Windows BGR
                if getattr(msg, 'encoding', '') == "rgb8" or msg.encoding == "":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            if img is not None:
                # Если разрешение кадра отличается от окна, принудительно масштабируем под 320x240
                if img.shape[0] != 240 or img.shape[1] != 320:
                    img = cv2.resize(img, (320, 240))
                    
                if self.queue.full():
                    try: self.queue.get_nowait()
                    except queue.Empty: pass
                self.queue.put_nowait(img)
                
        except Exception as e:
            print(f"[Viewer] Ошибка декодирования кадра: {e}. Размер байт: {len(msg.data)}, H:{msg.height}, W:{msg.width}", flush=True)


# Полностью замените функцию main() в файле roslessros/camera_viewer.py

def main():
    """Эта функция выполнится ТОЛЬКО если вы запустите файл напрямую: python camera_viewer.py"""
    rclpy.init()
    
    frame_queue = queue.Queue(maxsize=2)
    node = CameraViewerNode(frame_queue)
    
    print("[Viewer] Графический движок Windows активирован. Открываю окно...", flush=True)
    
    # Создаем стартовую серую картинку-заглушку, которая покажется ТОЛЬКО ОДИН РАЗ при запуске
    last_valid_frame = np.ones((240, 320, 3), dtype=np.uint8) * 100
    cv2.putText(last_valid_frame, "Waiting for first frame /camera/image_raw...", (10, 120), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            try:
                # Пробуем взять свежий кадр из сети брокера
                # Убираем nowait() и ставим микроскопический таймаут ожидания (5 мс)
                img = frame_queue.get(timeout=0.005)
                # Если кадр успешно пришел, запоминаем его как последний валидный
                last_valid_frame = img
            except queue.Empty:
                # Если в эту миллисекунду очередь пуста, мы НЕ РИСУЕМ серую заглушку,
                # а просто продолжаем крутить на экране последний удачно принятый кадр!
                pass
            
            # Всегда выводим актуальное изображение на экран Windows
            cv2.imshow("ROSLESSROS - Живой поток с камеры", last_valid_frame)
            
            # Обязательный триггер графических событий Windows (таймаут 1 мс)
            # 1 мс + 5 мс ожидания очереди дают идеальную частоту синхронизации под ваши 50 Гц!
            if cv2.waitKey(1) & 0xFF == 27:
                break
                
    except KeyboardInterrupt:
        print("\n[Viewer] Окно закрыто.", flush=True)
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
