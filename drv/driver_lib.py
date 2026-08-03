#!/usr/bin/env python3
#Copyright (c) 2026 Jurij Polikarpov
"""
Модуль: robot_driver.py
Содержит:
  - RobotDriver — базовый класс-клиент автопилота
  - Узлы дерева поведения (BTNode, Sequence, Fallback, Parallel, Condition,
    MoveRelativeAction, MoveTimeAction, StopAction, StreamAction,
    HeartbeatAction, LogAction, RetryAction)
  - MissionDispatcher — диспетчер миссий
  - AIDecisionAction — узел принятия решений ИИ
  - BehaviorTreeSquare — пример сценария «Квадрат» на дереве поведения

Версия с гарантированной доставкой команд:
  - Все Action-узлы проверяют ответ сервиса (Future + success)
  - Таймауты на отправку и выполнение
  - Обработка ABORTED-статусов и IDLE после приёма команды
  - RetryAction сбрасывает состояние дочернего узла при повторе
  - StreamAction для непрерывного управления по данным сенсоров
  - MissionDispatcher для переключения миссий
  - AIDecisionAction для принятия решений ИИ
"""

import rclpy, math
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
#import time
import threading
from ifra_mol_1.srv import AutopilotTime, AutopilotRelative
from ifra_mol_1.msg import AutopilotTelemetry
import socket
import http.server
import socketserver
from itertools import count

# ==============================================================================
# 3.2 BLACKBOARD — общая память между узлами
# ==============================================================================

class Blackboard:
    """Потокобезопасная общая память."""

    def __init__(self):
        self._data = {}
        self._lock = threading.Lock()

    def set(self, key: str, value):
        with self._lock:
            self._data[key] = value

    def get(self, key: str, default=None):
        with self._lock:
            return self._data.get(key, default)

    def has(self, key: str) -> bool:
        with self._lock:
            return key in self._data

    def delete(self, key: str):
        with self._lock:
            if key in self._data:
                del self._data[key]

    def clear(self):
        with self._lock:
            self._data.clear()

    def all(self) -> dict:
        with self._lock:
            return dict(self._data)

    def update(self, **kwargs):
        """Потокобезопасное обновление нескольких ключей."""
        with self._lock:
            self._data.update(kwargs)

class ImageProcessorNode(Node):
    """Базовый узел обработки изображений. Наследуй и переопредели process_frame()."""
    
    _counter = count(1)

    def __init__(self, driver, fps=30, node_name=None):
        if node_name is None:
            node_name = f'image_processor_{next(ImageProcessorNode._counter)}'
        super().__init__(node_name)
        
        self.driver = driver
        self._busy = False
        # Используем встроенный таймер ROS2
        self._timer = self.create_timer(1.0 / fps, self._process)
        self.get_logger().info(f"{node_name} готов ({fps} FPS)")

    def _process(self):
        # Если прошлый кадр еще обрабатывается, пропускаем текущий тик таймера
        if self._busy:
            return
            
        frame = self.driver.get_frame()
        if frame is None or frame.size == 0:
            return

        self._busy = True
        try:
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки кадра: {e}")
        finally:
            self._busy = False

    def process_frame(self, frame):
        """Переопредели в наследнике. frame — numpy array BGR."""
        raise NotImplementedError("Переопредели process_frame(frame)")

#import cv2
#import numpy as np
#from hailo_platform import (VDevice, HEF, ConfigureParams, HailoStreamInterface, 
#                            InputVStreamParams, OutputVStreamParams, InferVStreams)

import sys
import numpy as np
import cv2

try:
    # 1. Пытаемся импортировать реальный софт на роботе
    from hailo_platform import (VDevice, HEF, ConfigureParams, HailoStreamInterface,
                                InputVStreamParams, OutputVStreamParams, InferVStreams)
except ImportError:
    # 2. Если мы на ПК — на лету собираем виртуальную структуру hailo_platform
    print("[MOCK] Плата Hailo-8 не обнаружена. Инициализирую симуляцию нейросети для ПК...")

    class MockVStreamInfo:
        def __init__(self):
            self.name = "mock_input_layer_yolo"
            self.shape = (1, 640, 640, 3) # Имитируем стандартный вход YOLOv8/YOLOv5

    class MockHEF:
        def __init__(self, hef_path): pass
        def get_input_vstream_infos(self): return [MockVStreamInfo()]

    class MockContextManager:
        """Эмулирует вложенные контексты __enter__ и __exit__ чипа Hailo"""
        def __init__(self, *args, **kwargs): pass
        def __enter__(self): return self
        def __exit__(self, exc_type, exc_val, exc_tb): pass
        
        def infer(self, input_dict):
            """
            Имитация вывода нейросети на ПК.
            Возвращает фейковые пустые тензоры (словари), чтобы ваш handle_hailo_result не падал.
            """
            # Название выходного слоя должно соответствовать архитектуре вашей сети.
            # MagicMock просто вернет пустой массив. Вы можете заменить на реальные размерности:
            return {"mock_output_layer": np.zeros((1, 84, 8400), dtype=np.float32)}

    class MockParams:
        @classmethod
        def create_from_hef(cls, *args, **kwargs): return cls()
        @classmethod
        def make_from_network_group(cls, *args, **kwargs): return cls()

    class MockDevice:
        def __init__(self, *args, **kwargs): pass
        def configure(self, *args, **kwargs): return [MockNetworkGroup()]

    class MockNetworkGroup(MockContextManager):
        def activate(self): return MockContextManager()

    class MockStreamInterface:
        PCIe = "PCIe"

    # Создаем виртуальный модуль в системном кэше Python
    from types import ModuleType
    hailo_mod = ModuleType("hailo_platform")
    hailo_mod.VDevice = MockDevice
    hailo_mod.HEF = MockHEF
    hailo_mod.ConfigureParams = MockParams
    hailo_mod.HailoStreamInterface = MockStreamInterface
    hailo_mod.InputVStreamParams = MockParams
    hailo_mod.OutputVStreamParams = MockParams
    hailo_mod.InferVStreams = MockContextManager

    sys.modules["hailo_platform"] = hailo_mod
    
    # Повторяем импорт: теперь он заберет наши фейковые классы
    from hailo_platform import (VDevice, HEF, ConfigureParams, HailoStreamInterface,
                                InputVStreamParams, OutputVStreamParams, InferVStreams)


class HailoProcessorNode(ImageProcessorNode):
    """
    Новый базовый класс для любых ИИ-нод на чипе Hailo-8.
    Наследуется от стандартного ImageProcessorNode вашего робота.
    """

    def __init__(self, driver, hef_path, fps=30, node_name=None):
        # Инициализируем старый базовый класс (он создаст таймер и привяжет driver)
        super().__init__(driver, fps=fps, node_name=node_name)

        # 1. Аппаратная инициализация Hailo-8
        self._device = VDevice()
        self._hef = HEF(hef_path)

        self._configure_params = ConfigureParams.create_from_hef(
            self._hef, HailoStreamInterface.PCIe
        )
        self._network_groups = self._device.configure(self._hef, self._configure_params)
        self._network_group = self._network_groups[0]

        self._input_vstreams_params = InputVStreamParams.make_from_network_group(self._network_group)
        self._output_vstreams_params = OutputVStreamParams.make_from_network_group(self._network_group)

        # 2. Определение геометрии входного слоя модели
        self._input_info = self._hef.get_input_vstream_infos()[0]
        self._input_name = self._input_info.name
        self._input_size = self._input_info.shape

        if len(self._input_size) == 4:
            self._h_in = int(self._input_size[1])
            self._w_in = int(self._input_size[2])
        else:
            self._h_in = int(self._input_size[0])
            self._w_in = int(self._input_size[1])

        # 3. Фиксация вложенных контекстов активации в памяти
        self._activated_context = self._network_group.activate()
        self._activated_app = self._activated_context.__enter__()

        self._pipeline_context = InferVStreams(
            self._network_group,
            self._input_vstreams_params,
            self._output_vstreams_params
        )
        self._infer_pipeline = self._pipeline_context.__enter__()

        self.get_logger().info(f"Hailo-8 базовый класс готов. Слой: {self._input_name} ({self._w_in}x{self._h_in})")

    def process_frame(self, frame):
        display = frame.copy()

        # 1. Полноценный пропорциональный Letterbox под 640x640 с серым фоном 114
        img_h, img_w = frame.shape[:2]
        scale = min(self._w_in / img_w, self._h_in / img_h)
        new_w = int(img_w * scale)
        new_h = int(img_h * scale)
        
        resized = cv2.resize(frame, (new_w, new_h))
        
        # Важно: цвет холста 114 (серый стандарт для YOLO)
        letterbox_img = np.full((self._h_in, self._w_in, 3), 114, dtype=np.uint8)
        
        pad_x = (self._w_in - new_w) // 2
        pad_y = (self._h_in - new_h) // 2
        letterbox_img[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        
        # 2. Переводим в RGB (Официальный стандарт для Hailo Model Zoo)
        rgb = cv2.cvtColor(letterbox_img, cv2.COLOR_BGR2RGB)
        
        # 3. Добавляем батч-размерность (Формат NHWC: 1x640x640x3)
        batch_data = np.expand_dims(rgb, axis=0)
        
        # 4. Делаем массив непрерывным для Си-драйвера PCIe шины
        input_data = np.ascontiguousarray(batch_data)

        self._pad_info = (pad_x, pad_y, scale)

        t0 = self.get_clock().now()
        try:
            input_dict = {self._input_name: input_data}
            output = self._infer_pipeline.infer(input_dict)
            dt_ms = (self.get_clock().now() - t0).nanoseconds / 1e6  # миллисекунды

            self.handle_hailo_result(output, display, dt_ms)

        except Exception as e:
            self.get_logger().error(f"Ошибка инференса Hailo: {e}")

    def handle_hailo_result(self, output, display, dt_ms):
        """Этот метод ОБЯЗАТЕЛЬНО переопределяется в конечном скрипте миссии"""
        raise NotImplementedError("Переопредели handle_hailo_result(output, display, dt_ms) в наследнике")

    def __del__(self):
        if hasattr(self, '_pipeline_context') and self._pipeline_context:
            try:
                self._pipeline_context.__exit__(None, None, None)
            except Exception:
                pass
        if hasattr(self, '_activated_context') and self._activated_context:
            try:
                self._activated_context.__exit__(None, None, None)
            except Exception:
                pass
        if hasattr(self, '_device') and self._device:
            del self._device
        print("Ресурсы HailoProcessorNode успешно очищены.")


class HttpStreamerNode(Node):
    """HTTP видеопоток. Стримит JPEG из bb[key]."""
    _counter = 0
    def __init__(self, bb, key='stream_jpeg', port=8080, fps=30, node_name=None):
        if node_name is None:
            HttpStreamerNode._counter += 1
            node_name = f'http_streamer_{HttpStreamerNode._counter}'
        # Исправлено: передаем сгенерированный node_name в super().__init__
        super().__init__(node_name)
        self._bb = bb
        self._key = key
        self._httpd = None
        self._period = 1.0 / fps
        self._running = True # Флаг для остановки внутренних циклов клиентов
        self._start_server(port)
        self.get_logger().info(f"HTTP стример bb['{key}'] на http://localhost:{port} ({fps} FPS)")

    def _start_server(self, port):
        bb = self._bb
        key = self._key
        period = self._period
        node = self # Ссылка на саму ноду, чтобы проверять флаг остановки
        from time import sleep 

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass

            def do_GET(self):
                if self.path == '/' or self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    # Исправлено: проверяем флаг ноды node._running, чтобы выйти при дестрое
                    while node._running:
                        frame = bb.get(key)
                        if frame is not None:
                            try:
                                self.wfile.write(b'--frame\r\n')
                                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                                self.wfile.write(frame)
                                self.wfile.write(b'\r\n')
                            except (BrokenPipeError, ConnectionResetError, Exception):
                                break
                        sleep(period) 
                else:
                    self.send_response(404)
                    self.end_headers()

        # Исправлено: разрешаем повторное использование порта ДО создания сервера
        socketserver.TCPServer.allow_reuse_address = True
        self._httpd = socketserver.TCPServer(('0.0.0.0', port), Handler)
        
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def destroy_node(self):
        # 1. Меняем флаг, чтобы завершить отправку кадров во всех активных HTTP-клиентах
        self._running = False
        
        if self._httpd:
            # 2. Останавливаем цикл serve_forever фонового потока
            self._httpd.shutdown()
            # 3. Закрываем слушающий сокет
            self._httpd.server_close()
            self.get_logger().info("HTTP стример остановлен, порт свободен")
            
        super().destroy_node()


# Линия на :8080
#self.add_node(HttpStreamerNode(self.bb, key='stream_jpeg', port=8080))
# Каналы на :8081
#self.add_node(HttpStreamerNode(self.bb, key='channel_r', port=8081))
# HSV на :8082
#self.add_node(HttpStreamerNode(self.bb, key='hsv_jpeg', port=8082))

class TreeDashboardNode(Node):
    """HTTP дашборд с деревом поведения. Без видеопотока."""

    _counter = 0

    def __init__(self, driver, port=8079):
        TreeDashboardNode._counter += 1
        node_name = f'tree_dashboard_{TreeDashboardNode._counter}'
        super().__init__(node_name)
        self._driver = driver
        self._httpd = None
        self._running = True
        self._start_server(port)
        self.get_logger().info(f"Дашборд дерева на http://localhost:{port}")

    def _start_server(self, port):
        driver = self._driver
        node = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass

            def do_GET(self):
                if self.path == '/tree':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    tree_data = self._get_tree()
                    import json
                    self.wfile.write(json.dumps(tree_data).encode())

                elif self.path == '/' or self.path == '/dashboard':
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    self.wfile.write(self._html().encode())

                else:
                    self.send_response(404)
                    self.end_headers()

            def _get_tree(self):
                if driver is None or driver._bt_root is None:
                    return {"name": "Нет данных", "children": [], "state": ""}
                return self._node_to_dict(driver._bt_root)

            def _node_to_dict(self, bt_node):
                name = getattr(bt_node, 'name', type(bt_node).__name__)
                state = getattr(bt_node, '_state', '')
                
                children = []
                if hasattr(bt_node, 'child'):
                    children.append(self._node_to_dict(bt_node.child))
                elif hasattr(bt_node, 'children'):
                    for child in bt_node.children:
                        children.append(self._node_to_dict(child))
                
                return {"name": name, "state": state or "IDLE", "children": children}

            def _html(self):
                return """<!DOCTYPE html>
<html><head><title>Tree Dashboard</title>
<meta charset="utf-8">
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: 'Courier New', monospace; background: #0a0a1a; color: #0f0; padding: 20px; }
  h1 { font-size: 18px; margin-bottom: 15px; }
  .tree { font-size: 13px; line-height: 1.0; }
  .node { padding: 1px 0; }
  .RUNNING { color: #ff0; }
  .SUCCESS { color: #0f0; }
  .FAILURE { color: #f00; }
  .IDLE { color: #666; }
  .NO-STATE { color: #444; }
  #legend { margin-top: 20px; font-size: 11px; color: #888; }
  #legend span { margin-right: 15px; }
  #time { font-size: 12px; color: #888; margin-left: 10px; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 4px; }
  .dot.RUNNING { background: #ff0; box-shadow: 0 0 6px #ff0; }
  .dot.SUCCESS { background: #0f0; }
  .dot.FAILURE { background: #f00; }
  .dot.IDLE, .dot.NO-STATE { background: #444; }
</style></head><body>
<h1>🌳 Древо жизни <span id="time"></span></h1>
<div class="tree" id="tree"></div>
<div id="legend">
  <span><span class="dot RUNNING"></span> RUNNING</span>
  <span><span class="dot SUCCESS"></span> SUCCESS</span>
  <span><span class="dot FAILURE"></span> FAILURE</span>
  <span><span class="dot NO-STATE"></span> IDLE</span>
</div>
<script>
  var flatNodes = [];
const STATE_MAP = {
    'START': 'IDLE',
    'WAITING_RESPONSE': 'RUNNING',
    'MOVING': 'RUNNING',
    'IDLE': 'IDLE',
    'RUNNING': 'RUNNING',
    'SUCCESS': 'SUCCESS',
    'FAILURE': 'FAILURE',
    'ABORTED_STUCK': 'FAILURE',
    'ABORTED_TIMEOUT': 'FAILURE',
};
  function flatten(node, depth) {
    depth = depth || 0;
    var result = [];
    result.push({name: node.name, state: node.state || 'NO-STATE', depth: depth});
    if (node.children) {
      node.children.forEach(function(child) {
        result = result.concat(flatten(child, depth + 1));
      });
    }
    return result;
  }

  function updateTree() {
    fetch('/tree')
      .then(r => r.json())
      .then(data => {
        var container = document.getElementById('tree');
        var newFlat = flatten(data);
        
        var changed = flatNodes.length !== newFlat.length;
        if (!changed) {
          for (var i = 0; i < newFlat.length; i++) {
            if (newFlat[i].state !== flatNodes[i].state) {
              changed = true;
              break;
            }
          }
        }
        
        if (changed) {
          container.innerHTML = '';
          newFlat.forEach(function(n) {
            var div = document.createElement('div');
            var displayState = STATE_MAP[n.state] || n.state || 'IDLE';
            div.className = 'node ' + displayState;
            var indent = '│  '.repeat(n.depth);
            var prefix = n.depth > 0 ? '├─ ' : '';
            var dot = '<span class="dot ' + n.state + '"></span>';
            div.innerHTML = indent + prefix + dot + n.name;
            container.appendChild(div);
          });
          flatNodes = newFlat;
        }
        
        document.getElementById('time').textContent = new Date().toLocaleTimeString();
      })
      .catch(e => {});
  }

  setInterval(updateTree, 500);
  updateTree();
</script>
</body></html>"""

        socketserver.TCPServer.allow_reuse_address = True
        self._httpd = socketserver.TCPServer(('0.0.0.0', port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def destroy_node(self):
        self._running = False
        if self._httpd:
            self._httpd.socket.close()
            self._httpd.server_close()
            self.get_logger().info("Дашборд дерева остановлен")
        super().destroy_node()

# ==============================================================================
# 1. БАЗОВЫЙ КЛАСС-КЛИЕНТ АВТОПИЛОТА (DRIVER)
# ==============================================================================


class BTNode:
    """Абстрактный узел дерева поведения."""

    def tick(self, driver: 'RobotDriver') -> str:
        raise NotImplementedError("Переопределите tick в наследнике")

    def reset(self):
        """Сброс внутреннего состояния. По умолчанию ничего не делает."""
        pass

class RobotDriver(Node):
    """
    Универсальный робот-драйвер.
    Два режима работы:
      1. Старый: ручное управление _bt_root и таймером.
      2. Новый: переопределите build_tree() и вызовите spin().
    Предоставляет интерфейс к автопилоту, геттеры для лидара и камеры.
    """

    def __init__(self, node_name: str = 'robot_driver', tick_rate: float = None):
        super().__init__(node_name)

        # Сервис-клиенты
        self._cli_time = self.create_client(AutopilotTime, 'autopilot/go_time')
        self._cli_relative = self.create_client(AutopilotRelative, 'autopilot/go_relative')
        self._cli_stop = self.create_client(Trigger, 'autopilot/stop')

        # Телеметрия
        self._telemetry_sub = self.create_subscription(
            AutopilotTelemetry, '/autopilot/telemetry', self._on_telemetry, 10)

        # Heartbeat
        self._hb_pub = self.create_publisher(Empty, '/autopilot/heartbeat', 10)

        # Blackboard — единая потокобезопасная память
        self.bb = Blackboard()
        self.bb.update(
            current_status="IDLE",
            current_x=0.0, current_y=0.0, current_yaw=0.0,
            has_telemetry=False,
            lidar_ranges=[],
            lidar_angle_min=0.0, lidar_angle_increment=0.0,
            lidar_ready=False,
            lidar_offset_x=0.0, lidar_offset_y=0.0,
            lidar_offset_ready=False,
            tf_tree={},
            last_frame=None,
            camera_ready=False,
            line_offset=0.0, line_angle=0.0, line_detected=False,
            object_x=0.0, object_y=0.0, object_detected=False,
            marker_ids=[], marker_poses={}, markers_detected=False,
            flow_dx=0.0, flow_dy=0.0, flow_ready=False,
            detections=[], detections_ready=False,
            extra_nodes=[],
        )

        # Сенсоры
        self._init_lidar()
        self._init_camera()
        self._init_vision()
        self._init_neural()

        # Режим запуска
        self._active = False
        self._debug = False

        if tick_rate is not None:
            # Новый режим: авто-запуск через build_tree()
            self._mission_root = self.build_tree()
            self._bt_root = Parallel([
                HeartbeatAction(),
                self._mission_root,
            ], required_successes=1, name="Цель")
            self._ticker = self.create_timer(1.0 / tick_rate, self._loop)
            self.get_logger().info(f"Миссия готова, тик {tick_rate} Гц")
        else:
            # Старый режим: наследник сам создаст _bt_root и таймер
            self._bt_root = None
            self.get_logger().info("Драйвер готов (ручной режим)")

        # Ожидание автопилота
        self.get_logger().info("Ожидание автопилота...")
        while not self._cli_time.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info("Повторное ожидание...")
        self.get_logger().info("Автопилот подключён.")

    # =====================================================================
    # МИССИЯ (переопределите в наследнике для нового режима)
    # =====================================================================

    def build_tree(self) -> BTNode:
        raise NotImplementedError("Переопределите build_tree()")

    # =====================================================================
    # ЗАПУСК
    # =====================================================================

    @property
    def debug(self) -> bool:
        return self._debug

    @debug.setter
    def debug(self, value: bool):
        self._debug = value

    def _loop(self):
        #print(f"\033[91m[sim loop 1\033[0m", flush=True)
        if not self.is_connected():
            return
        #print(f"\033[91m[sim loop 2\033[0m", flush=True)
        if not self._active:
            print(f"\nРобот готов к миссии '{self.__class__.__name__}'.")
            #input("Нажмите Enter для старта...")
            self.get_logger().info(f"Лидар offset: x={self.bb.get('lidar_offset_x'):.3f}, y={self.bb.get('lidar_offset_y'):.3f}, ready={self.bb.get('lidar_offset_ready')}")
            self.get_logger().info("Запуск миссии...")
            self._active = True
            return
        status = self._bt_root.tick(self)
        if self._debug:
            self._print_tree()
        if status == "SUCCESS":
            self.get_logger().info("Миссия выполнена успешно!")
            self.stop()
            self.create_timer(0.5, lambda: rclpy.shutdown())
        elif status == "FAILURE":
            self.get_logger().error("Миссия провалена. Аварийная остановка.")
            self.stop()
            self.create_timer(0.5, lambda: rclpy.shutdown())

    def _print_tree(self, node=None, indent=0):
        if node is None:
            node = self._bt_root
        prefix = "  " * indent + ("→ " if indent > 0 else "")
        name = getattr(node, 'name', node.__class__.__name__)
        state = getattr(node, '_state', '')
        state_str = f" [{state}]" if state else ""
        self.get_logger().info(f"{prefix}{name}{state_str}")
        for child in getattr(node, 'children', []):
            self._print_tree(child, indent + 1)

    @staticmethod
    def spin(node: 'RobotDriver'):
        """Запуск с многопоточным исполнителем."""
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        for extra in node.bb.get('extra_nodes', []):
            executor.add_node(extra)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
            for extra in node.bb.get('extra_nodes', []):
                extra.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def add_node(self, node):
        """Добавить узел в пул параллельного выполнения."""
        nodes = self.bb.get('extra_nodes', [])
        nodes.append(node)
        self.bb.set('extra_nodes', nodes)
        self.get_logger().info(f"Добавлен узел: {node.get_name()}")

    # =====================================================================
    # ИНТЕРФЕЙС АВТОПИЛОТА
    # =====================================================================

    def _on_telemetry(self, msg: AutopilotTelemetry):
        self.bb.update(
            current_status=msg.status,
            current_x=msg.x,
            current_y=msg.y,
            current_yaw=msg.yaw,
            has_telemetry=True,
        )

    def get_status(self) -> str:
        return self.bb.get('current_status', "IDLE")

    def get_pose(self) -> tuple:
        return (
            self.bb.get('current_x', 0.0),
            self.bb.get('current_y', 0.0),
            self.bb.get('current_yaw', 0.0),
        )

    def is_connected(self) -> bool:
        return self.bb.get('has_telemetry', False)

    def send_heartbeat(self):
        self._hb_pub.publish(Empty())

    def move_time(self, vx: float, vy: float, omega: float, duration: float):
        req = AutopilotTime.Request()
        req.vx = vx
        req.vy = vy
        req.omega = omega
        req.duration = duration
        return self._cli_time.call_async(req)

    def move_relative(self, dx: float, dy: float, dyaw_deg: float):
        req = AutopilotRelative.Request()
        req.dx = dx
        req.dy = dy
        req.dyaw_deg = dyaw_deg
        return self._cli_relative.call_async(req)

    def stop(self):
        req = Trigger.Request()
        return self._cli_stop.call_async(req)

    # =====================================================================
    # СЕНСОРЫ: ЛИДАР
    # =====================================================================

    def _init_lidar(self):
        from sensor_msgs.msg import LaserScan
        self.create_subscription(LaserScan, '/scan', self._on_lidar, 10)

    def _update_lidar_offset(self):
        if self.bb.get('lidar_offset_ready') or hasattr(self, '_tf_static_sub'):
            return
        try:
            from tf2_msgs.msg import TFMessage
            from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

            static_qos = QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST
            )

            self._tf_static_sub = self.create_subscription(
                TFMessage, '/tf_static', self._on_tf_static, static_qos)
            self.get_logger().info("Подписка на /tf_static создана")
        except Exception as e:
            self.get_logger().warn(f"Не удалось подписаться на /tf_static: {e}")

    def _on_tf_static(self, msg):
        if self.bb.get('lidar_offset_ready'):
            return

        tf_tree = self.bb.get('tf_tree')
        for t in msg.transforms:
            tf_tree[t.child_frame_id] = (t.header.frame_id, t.transform.translation.x, t.transform.translation.y)
        self.bb.set('tf_tree', tf_tree)

        if 'lidar_link' in tf_tree:
            x, y = 0.0, 0.0
            frame = 'lidar_link'
            max_depth = 20
            depth = 0

            while frame in tf_tree and depth < max_depth:
                depth += 1
                parent, dx, dy = tf_tree[frame]
                x += dx
                y += dy
                frame = parent

                if frame == 'base_link':
                    self.bb.update(lidar_offset_x=x, lidar_offset_y=y, lidar_offset_ready=True)
                    self.get_logger().info(f"Лидар смещён: x={x:.3f}, y={y:.3f}")
                    self.destroy_subscription(self._tf_static_sub)
                    return

    def _transform_lidar_distance(self, angle_rad: float, measured: float) -> float:
        if measured >= float('inf') or not self.bb.get('lidar_offset_ready'):
            return measured
        offset_proj = self.bb.get('lidar_offset_x') * math.cos(angle_rad) + self.bb.get('lidar_offset_y') * math.sin(angle_rad)
        return max(0.0, measured - offset_proj)

    def _on_lidar(self, msg):
        self.bb.update(
            lidar_ranges=list(msg.ranges),
        )
        if not self.bb.get('lidar_ready'):
            self.bb.update(
                lidar_angle_min=msg.angle_min,
                lidar_angle_increment=msg.angle_increment,
                lidar_ready=True,
            )
        self._update_lidar_offset()

    def get_lidar_ranges(self) -> list:
        """Вернуть сырой массив замеров лидара."""
        return self.bb.get('lidar_ranges')

    def get_lidar_angle_min(self) -> float:
        """Минимальный угол лидара (радиан)."""
        return self.bb.get('lidar_angle_min')

    def get_lidar_angle_increment(self) -> float:
        """Шаг угла лидара (радиан)."""
        return self.bb.get('lidar_angle_increment')

    def _lidar_index(self, angle_rad: float) -> int:
        while angle_rad < self.bb.get('lidar_angle_min'):
            angle_rad += 2 * math.pi
        while angle_rad >= self.bb.get('lidar_angle_min') + 2 * math.pi:
            angle_rad -= 2 * math.pi
        return int((angle_rad - self.bb.get('lidar_angle_min')) / self.bb.get('lidar_angle_increment'))

    def _lidar_distance_at(self, angle_rad: float, default=float('inf')) -> float:
        if not self.bb.get('lidar_ready'):
            return default
        idx = self._lidar_index(angle_rad)
        if 0 <= idx < len(self.bb.get('lidar_ranges')):
            val = self.bb.get('lidar_ranges')[idx]
            return val if val > 0.01 else default
        return default

    def _lidar_min_in_range(self, start_rad: float, end_rad: float) -> float:
        if not self.bb.get('lidar_ready'):
            return float('inf')
        start_idx = self._lidar_index(start_rad)
        end_idx = self._lidar_index(end_rad) + 1
        if start_idx <= end_idx:
            vals = [v for v in self.bb.get('lidar_ranges')[start_idx:end_idx] if v > 0.01]
        else:
            vals = [v for v in self.bb.get('lidar_ranges')[start_idx:] + self.bb.get('lidar_ranges')[:end_idx] if v > 0.01]
        return min(vals) if vals else float('inf')

    def get_lidar_sector(self, center_rad: float, width_deg: float = 20.0) -> float:
        """
        Минимальное расстояние в секторе.
        center_rad — центр сектора в радианах (0 — вперёд, ±π/2 — бока, ±π — назад).
        width_deg — ширина сектора в градусах.
        """
        w = math.radians(width_deg / 2)
        raw = self._lidar_min_in_range(center_rad - w, center_rad + w)
        return self._transform_lidar_distance(center_rad, raw)

    def get_lidar_front(self, width_deg: float = 30.0) -> float:
        return self.get_lidar_sector(0.0, width_deg)

    def get_lidar_right(self, width_deg: float = 20.0) -> float:
        return self.get_lidar_sector(-math.pi/2, width_deg)

    def get_lidar_left(self, width_deg: float = 20.0) -> float:
        return self.get_lidar_sector(math.pi/2, width_deg)

    def get_lidar_back(self, width_deg: float = 20.0) -> float:
        return self.get_lidar_sector(math.pi, width_deg)

    def get_lidar_min_distance(self) -> float:
        if not self.bb.get('lidar_ready'):
            return float('inf')
        vals = [v for v in self.bb.get('lidar_ranges') if v > 0.01]
        return self._transform_lidar_distance(0.0, min(vals)) if vals else float('inf')

    def get_lidar_sectors(self, front_width=30, side_width=20) -> dict:
        return {
            "front": self.get_lidar_front(front_width),
            "left": self.get_lidar_left(side_width),
            "right": self.get_lidar_right(side_width),
            "back": self.get_lidar_back(side_width),
        }

    # =====================================================================
    # СЕНСОРЫ: КАМЕРА
    # =====================================================================

    def _init_camera(self):
        from sensor_msgs.msg import Image
        import cv2
        from cv_bridge import CvBridge
        self._cv_bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self._on_camera, 10)

    def _on_camera(self, msg):
        import cv2
        try:
            # Выводим кодировку кадра, чтобы увидеть, не затерлась ли она в сети!
            #enc = getattr(msg, 'encoding', 'NOT_FOUND')
            #self.get_logger().error(f"camera: 1, Encoding in MSG: '{enc}'")
            #self.get_logger().error(f"camera: 1")
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #self.get_logger().error(f"camera: 2")
            self.bb.update(last_frame=frame, camera_ready=True)
            #self.get_logger().error(f"camera: 3")
        except Exception as e:
            self.get_logger().error(f"camera: {e}")
            pass

    def get_frame(self):
        return self.bb.get('last_frame')

    def save_frame(self, filename: str = None) -> str:
        import cv2
        frame = self.bb.get('last_frame')
        if frame is None:
            return ""
        if filename is None:
            from datetime import datetime
            filename = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        cv2.imwrite(filename, frame)
        self.get_logger().info(f"Кадр сохранён: {filename}")
        return filename

    # =====================================================================
    # СЕНСОРЫ: КОМПЬЮТЕРНОЕ ЗРЕНИЕ (OpenCV)
    # =====================================================================

    def _init_vision(self):
        """Поля для результатов компьютерного зрения."""
        pass

    def set_line_info(self, offset: float, angle: float, detected: bool = True):
        self.bb.update(line_offset=offset, line_angle=angle, line_detected=detected)

    def get_line_position(self) -> tuple:
        return (self.bb.get('line_offset', 0.0), self.bb.get('line_angle', 0.0), self.bb.get('line_detected', False))

    def set_object_position(self, x: float, y: float, detected: bool = True):
        self.bb.update(object_x=x, object_y=y, object_detected=detected)

    def get_object_position(self) -> tuple:
        return (self.bb.get('object_x', 0.0), self.bb.get('object_y', 0.0), self.bb.get('object_detected', False))

    def set_marker_info(self, ids: list, poses: dict):
        self.bb.update(marker_ids=ids, marker_poses=poses, markers_detected=len(ids) > 0)

    def get_marker_info(self) -> tuple:
        return (self.bb.get('marker_ids', []), self.bb.get('marker_poses', {}), self.bb.get('markers_detected', False))

    def get_marker_id(self):
        ids = self.bb.get('marker_ids', [])
        return ids[0] if ids else None

    def set_optical_flow(self, dx: float, dy: float):
        self.bb.update(flow_dx=dx, flow_dy=dy, flow_ready=True)

    def get_optical_flow(self) -> tuple:
        return (self.bb.get('flow_dx', 0.0), self.bb.get('flow_dy', 0.0), self.bb.get('flow_ready', False))

    # =====================================================================
    # СЕНСОРЫ: НЕЙРОСЕТИ (PyTorch / Hailo-8)
    # =====================================================================

    def _init_neural(self):
        pass

    def set_detections(self, detections: list):
        self.bb.update(detections=detections, detections_ready=True)

    def get_detections(self) -> list:
        return self.bb.get('detections', [])

    def has_detections(self) -> bool:
        return self.bb.get('detections_ready', False) and len(self.bb.get('detections', [])) > 0

    def is_class_detected(self, class_name: str) -> bool:
        return any(d[0] == class_name for d in self.bb.get('detections', []))

    def is_person_detected(self) -> bool:
        return self.is_class_detected("person")

    def get_detections_by_class(self, class_name: str) -> list:
        return [d for d in self.bb.get('detections', []) if d[0] == class_name]

# ==============================================================================
# 2. ЯДРО ДЕРЕВА ПОВЕДЕНИЯ
# ==============================================================================



class Sequence(BTNode):
    """Последовательное «И»: все дети должны завершиться SUCCESS."""

    def __init__(self, children: list, name: str = "Sequence"):
        self.children = children
        self._idx = 0
        self._state = "IDLE"
        self.name = name

    def reset(self):
        self._idx = 0
        self._state = "IDLE"
        for child in self.children:
            child.reset()

    def tick(self, driver: RobotDriver) -> str:
        if self._idx >= len(self.children):
            self._state = "SUCCESS"
            return "SUCCESS"

        status = self.children[self._idx].tick(driver)
        self._state = status

        if status == "RUNNING":
            return "RUNNING"
        elif status == "FAILURE":
            return "FAILURE"
        elif status == "SUCCESS":
            driver.get_logger().info(
                f"[Sequence] Узел {self._idx + 1}/{len(self.children)} завершён"
            )
            self._idx += 1
            return "RUNNING"


class Fallback(BTNode):
    """Селектор «ИЛИ»: первый SUCCESS останавливает узел."""

    def __init__(self, children: list, name: str = "Fallback"):
        self.children = children
        self._idx = 0
        self._state = "IDLE"
        self.name = name

    def reset(self):
        self._idx = 0
        self._state = "IDLE"
        for child in self.children:
            child.reset()

    def tick(self, driver: RobotDriver) -> str:
        if self._idx >= len(self.children):
            self._state = "FAILURE"
            return "FAILURE"

        status = self.children[self._idx].tick(driver)
        self._state = status

        if status == "RUNNING":
            return "RUNNING"
        elif status == "SUCCESS":
            return "SUCCESS"
        elif status == "FAILURE":
            self._idx += 1
            self._state = "RUNNING"
            return "RUNNING"


class Parallel(BTNode):
    """Параллельный узел: все дети тикаются одновременно."""

    def __init__(self, children: list, required_successes: int = None, name: str = "Parallel"):
        self.children = children
        self.required = required_successes if required_successes is not None else len(children)
        self._child_statuses = ["RUNNING"] * len(children)
        self._state = "IDLE"
        self.name = name

    def reset(self):
        self._child_statuses = ["RUNNING"] * len(self.children)
        self._state = "IDLE"
        for child in self.children:
            child.reset()

    def tick(self, driver: RobotDriver) -> str:
        successes = 0
        failures = 0
        for i, child in enumerate(self.children):
            if self._child_statuses[i] == "RUNNING":
                self._child_statuses[i] = child.tick(driver)
            if self._child_statuses[i] == "SUCCESS":
                successes += 1
            elif self._child_statuses[i] == "FAILURE":
                failures += 1

        if successes >= self.required:
            self._state = "SUCCESS"
            return "SUCCESS"
        if failures > 0:
            self._state = "FAILURE"
            return "FAILURE"
        self._state = "RUNNING"
        return "RUNNING"


class Condition(BTNode):
    """Проверка условия — мгновенный SUCCESS или FAILURE."""

    def __init__(self, check_func, name: str = "Condition"):
        self._check = check_func
        self.name = name
        self._state = "IDLE"

    def tick(self, driver: RobotDriver) -> str:
        result = self._check(driver)
        self._state = "SUCCESS" if result else "FAILURE"
        return "SUCCESS" if result else "FAILURE"

    def reset(self):
        self._state = "IDLE"


# ==============================================================================
# 3. БИБЛИОТЕКА УЗЛОВ-ДЕЙСТВИЙ
# ==============================================================================

class MoveRelativeAction(BTNode):
    """
    Относительное перемещение через сервис go_relative.
    Гарантирует проверку ответа сервиса и таймауты.
    Состояния: START → WAITING_RESPONSE → MOVING → SUCCESS / FAILURE
    """

    def __init__(self, dx: float = 0.0, dy: float = 0.0, dyaw_deg: float = 0.0,
                 name: str = "",
                 send_timeout: float = 2.0,
                 exec_timeout: float = 30.0):
        self.dx = dx
        self.dy = dy
        self.dyaw_deg = dyaw_deg
        self.name = name or f"Move(dx={dx},dy={dy},yaw={dyaw_deg})"
        self._send_timeout = send_timeout
        self._exec_timeout = exec_timeout
        self._state = "START"
        self._future = None
        self._state_start_time = None

    def _log(self, driver, msg: str):
        driver.get_logger().info(f"[BT] {self.name}: {msg}")

    def _warn(self, driver, msg: str):
        driver.get_logger().warn(f"[BT] {self.name}: {msg}")

    def _error(self, driver, msg: str):
        driver.get_logger().error(f"[BT] {self.name}: {msg}")

    def tick(self, driver: RobotDriver) -> str:
        now = driver.get_clock().now().nanoseconds * 1e-9

        if self._state == "START":
            self._log(driver, "Отправка go_relative...")
            dx = self.dx(driver) if callable(self.dx) else self.dx
            dy = self.dy(driver) if callable(self.dy) else self.dy
            dyaw = self.dyaw_deg(driver) if callable(self.dyaw_deg) else self.dyaw_deg
            self._future = driver.move_relative(dx, dy, dyaw)
            self._state = "WAITING_RESPONSE"
            self._state_start_time = now
            return "RUNNING"

        elif self._state == "WAITING_RESPONSE":
            if now - self._state_start_time > self._send_timeout:
                self._error(driver, f"Сервис не ответил за {self._send_timeout}с")
                self.reset()
                return "FAILURE"

            if self._future is None:
                self._error(driver, "Future is None")
                self.reset()
                return "FAILURE"

            if not self._future.done():
                return "RUNNING"

            response = self._future.result()
            if response is None:
                self._error(driver, "Ответ: None")
                self.reset()
                return "FAILURE"

            if not response.success:
                self._error(driver, f"Отклонено: {response.status_message}")
                self.reset()
                return "FAILURE"

            self._log(driver, f"Принято: {response.status_message}")
            self._state = "MOVING"
            self._state_start_time = now
            return "RUNNING"

        elif self._state == "MOVING":
            if now - self._state_start_time > self._exec_timeout:
                self._error(driver, f"Таймаут выполнения ({self._exec_timeout}с)")
                driver.stop()
                self.reset()
                return "FAILURE"

            status = driver.get_status()

            if status == "MOVING":
                return "RUNNING"
            elif status == "SUCCESS":
                self._log(driver, "Успешно")
                self.reset()
                return "SUCCESS"
            elif status in ("ABORTED_STUCK", "ABORTED_TIMEOUT"):
                self._error(driver, f"Статус: {status}")
                self.reset()
                return "FAILURE"
            elif status == "IDLE":
                if now - self._state_start_time > 3.0:
                    self._warn(driver, "IDLE > 3 сек после приёма")
                    self.reset()
                    return "FAILURE"
                return "RUNNING"

            return "RUNNING"

        return "RUNNING"

    def reset(self):
        self._state = "START"
        self._future = None
        self._state_start_time = None


class MoveTimeAction(BTNode):
    """
    Движение по времени через сервис go_time.
    Гарантирует проверку ответа сервиса и таймауты.
    Состояния: START → WAITING_RESPONSE → MOVING → SUCCESS / FAILURE
    """

    def __init__(self, vx: float = 0.0, vy: float = 0.0, omega: float = 0.0,
                 duration: float = 1.0, name: str = "",
                 send_timeout: float = 2.0, exec_timeout: float = 30.0):
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.duration = duration
        self.name = name or f"Time(vx={vx},vy={vy},omega={omega},{duration}s)"
        self._send_timeout = send_timeout
        self._exec_timeout = exec_timeout
        self._state = "START"
        self._future = None
        self._state_start_time = None

    def _log(self, driver, msg: str):
        driver.get_logger().info(f"[BT] {self.name}: {msg}")

    def _warn(self, driver, msg: str):
        driver.get_logger().warn(f"[BT] {self.name}: {msg}")

    def _error(self, driver, msg: str):
        driver.get_logger().error(f"[BT] {self.name}: {msg}")

    def tick(self, driver: RobotDriver) -> str:
        now = driver.get_clock().now().nanoseconds * 1e-9

        if self._state == "START":
            self._log(driver, "Отправка go_time...")
            vx = self.vx(driver) if callable(self.vx) else self.vx
            vy = self.vy(driver) if callable(self.vy) else self.vy
            omega = self.omega(driver) if callable(self.omega) else self.omega
            duration = self.duration(driver) if callable(self.duration) else self.duration
            self._future = driver.move_time(vx, vy, omega, duration)
            self._state = "WAITING_RESPONSE"
            self._state_start_time = now
            return "RUNNING"

        elif self._state == "WAITING_RESPONSE":
            if now - self._state_start_time > self._send_timeout:
                self._error(driver, f"Сервис не ответил за {self._send_timeout}с")
                self.reset()
                return "FAILURE"

            if self._future is None:
                self._error(driver, "Future is None")
                self.reset()
                return "FAILURE"

            if not self._future.done():
                return "RUNNING"

            response = self._future.result()
            if response is None:
                self._error(driver, "Ответ: None")
                self.reset()
                return "FAILURE"

            if not response.success:
                self._error(driver, f"Отклонено: {response.status_message}")
                self.reset()
                return "FAILURE"

            self._log(driver, f"Принято: {response.status_message}")
            self._state = "MOVING"
            self._state_start_time = now
            return "RUNNING"

        elif self._state == "MOVING":
            if now - self._state_start_time > self._exec_timeout:
                self._error(driver, f"Таймаут выполнения ({self._exec_timeout}с)")
                driver.stop()
                self.reset()
                return "FAILURE"

            status = driver.get_status()

            if status == "MOVING":
                return "RUNNING"
            elif status == "SUCCESS":
                self._log(driver, "Успешно")
                self.reset()
                return "SUCCESS"
            elif status in ("ABORTED_STUCK", "ABORTED_TIMEOUT"):
                self._error(driver, f"Статус: {status}")
                self.reset()
                return "FAILURE"
            elif status == "IDLE":
                if now - self._state_start_time > 3.0:
                    self._warn(driver, "IDLE > 3 сек после приёма")
                    self.reset()
                    return "FAILURE"
                return "RUNNING"

            return "RUNNING"

        return "RUNNING"

    def reset(self):
        self._state = "START"
        self._future = None
        self._state_start_time = None


class StopAction(BTNode):
    """
    Экстренная остановка через сервис stop.
    Гарантирует проверку ответа сервиса.
    Состояния: START → WAITING_RESPONSE → SUCCESS / FAILURE
    """

    def __init__(self, name: str = "Stop", send_timeout: float = 2.0):
        self.name = name
        self._send_timeout = send_timeout
        self._state = "START"
        self._future = None
        self._state_start_time = None

    def _log(self, driver, msg: str):
        driver.get_logger().info(f"[BT] {self.name}: {msg}")

    def _error(self, driver, msg: str):
        driver.get_logger().error(f"[BT] {self.name}: {msg}")

    def tick(self, driver: RobotDriver) -> str:
        now = driver.get_clock().now().nanoseconds * 1e-9

        if self._state == "START":
            self._log(driver, "Отправка stop...")
            self._future = driver.stop()
            self._state = "WAITING_RESPONSE"
            self._state_start_time = now
            return "RUNNING"

        elif self._state == "WAITING_RESPONSE":
            if now - self._state_start_time > self._send_timeout:
                self._error(driver, f"Сервис не ответил за {self._send_timeout}с")
                self.reset()
                return "FAILURE"

            if self._future is None:
                self._error(driver, "Future is None")
                self.reset()
                return "FAILURE"

            if not self._future.done():
                return "RUNNING"

            response = self._future.result()
            if response is None:
                self._error(driver, "Ответ: None")
                self.reset()
                return "FAILURE"

            if not response.success:
                self._error(driver, f"Отклонено: {response.message}")
                self.reset()
                return "FAILURE"

            self._log(driver, "Остановка подтверждена")
            self.reset()
            return "SUCCESS"

        return "RUNNING"

    def reset(self):
        self._state = "START"
        self._future = None
        self._state_start_time = None


class HeartbeatAction(BTNode):
    """Бесконечная отправка heartbeat. RUNNING пока всё живо."""

    def __init__(self):
        self._state = "RUNNING"
        self.name = "Пульс"

    def tick(self, driver: RobotDriver) -> str:
        if not rclpy.ok():
            self._state = "FAILURE"
            return "FAILURE"
        driver.send_heartbeat()
        self._state = "RUNNING"
        return "RUNNING"

    def reset(self):
        self._state = "RUNNING"


class EvalAction(BTNode):
    """Вычисляет переданную функцию и логирует результат. Мгновенный SUCCESS."""
    def __init__(self, func, name="Eval"):
        self._func = func
        self.name = name
        self._done = False
        self._state = "IDLE"

    def tick(self, driver) -> str:
        if not self._done:
            result = self._func(driver)
            driver.get_logger().info(f"[BT] {self.name}: {result}")
            self._done = True
            self._state = "SUCCESS"
        return "SUCCESS"

    def reset(self):
        self._done = False
        self._state = "IDLE"

class LogAction(BTNode):
    """Печать сообщения в лог. Мгновенный SUCCESS."""

    def __init__(self, message: str, name: str = "Log"):
        self._msg = message
        self._done = False
        self.name = name
        self._state = "IDLE"

    def tick(self, driver: RobotDriver) -> str:
        if not self._done:
            driver.get_logger().info(f"[BT] {self._msg}")
            self._done = True
            self._state = "SUCCESS"
        return "SUCCESS"

    def reset(self):
        self._done = False
        self._state = "IDLE"


class RetryAction(BTNode):
    """Повторить дочерний узел до N раз при FAILURE."""

    def __init__(self, child: BTNode, max_attempts: int = 3, name: str = "Retry"):
        self.child = child
        self.max_attempts = max_attempts
        self._attempt = 0
        self.name = name
        self._state = "IDLE"

    def tick(self, driver: RobotDriver) -> str:
        status = self.child.tick(driver)

        if status == "SUCCESS":
            self._attempt = 0
            self._state = "SUCCESS"
            return "SUCCESS"
        elif status == "FAILURE":
            self._attempt += 1
            driver.get_logger().warning(
                f"[Retry] Попытка {self._attempt}/{self.max_attempts} провалена"
            )
            self.child.reset()
            if self._attempt >= self.max_attempts:
                self._attempt = 0
                self._state = "FAILURE"
                return "FAILURE"
            self._state = "RUNNING"
            return "RUNNING"
        self._state = "RUNNING"
        return "RUNNING"

    def reset(self):
        self._attempt = 0
        self._state = "IDLE"
        self.child.reset()

class StreamAction(BTNode):
    """
    Потоковое действие: на каждом тике заново вызывает go_time
    с обновлёнными параметрами на основе данных сенсоров.
    Контроллер перезаписывает цель при повторном вызове.
    Возвращает RUNNING пока условие завершения не выполнено.
    
    Если одометрия не меняется за 1 секунду, но условие остановки 
    не выполнено — выводит предупреждение (не чаще 1 раза в секунду).
    """

    def __init__(self, command_func, stop_condition=None, 
                 name="Stream", 
                 stuck_xy_threshold=0.02,    # 2 см за секунду
                 stuck_yaw_threshold=0.05):  # ~3° за секунду
        """
        command_func(driver) -> (vx, vy, omega, duration)
        stop_condition(driver) -> bool (опционально, если None — бесконечное)
        stuck_xy_threshold: мин. изменение позиции (м) за секунду
        stuck_yaw_threshold: мин. изменение угла (рад) за секунду
        """
        self._command_func = command_func
        self._stop_condition = stop_condition
        self.name = name
        self._stuck_xy_threshold = stuck_xy_threshold
        self._stuck_yaw_threshold = stuck_yaw_threshold
        self._started = False
        self._last_warn_time = 0.0
        self._last_check_time = 0.0
        self._check_start_x = 0.0
        self._check_start_y = 0.0
        self._check_start_yaw = 0.0
        self._check_interval = 1.0  # Проверяем раз в секунду
        self._state = "IDLE"

    def _log(self, driver, msg: str):
        driver.get_logger().info(f"[BT] {self.name}: {msg}")

    def _warn(self, driver, msg: str):
        driver.get_logger().warning(f"[BT] {self.name}: {msg}")

    def tick(self, driver: RobotDriver) -> str:
        if self._started and self._stop_condition and self._stop_condition(driver):
            self._log(driver, "Условие завершения выполнено")
            driver.stop()
            self._state = "SUCCESS"
            return "SUCCESS"

        current_x, current_y, current_yaw = driver.get_pose()
        now = driver.get_clock().now().nanoseconds * 1e-9
        
        # Инициализация точки отсчёта
        if not self._started:
            self._check_start_x = current_x
            self._check_start_y = current_y
            self._check_start_yaw = current_yaw
            self._last_check_time = now
        
        # Проверка раз в секунду
        if self._started and self._stop_condition and now - self._last_check_time >= self._check_interval:
            dx = abs(current_x - self._check_start_x)
            dy = abs(current_y - self._check_start_y)
            dyaw = abs(current_yaw - self._check_start_yaw)
            
            if (dx < self._stuck_xy_threshold and 
                dy < self._stuck_xy_threshold and 
                dyaw < self._stuck_yaw_threshold):
                if now - self._last_warn_time >= 1.0:
                    self._warn(driver, 
                        f"Одометрия не меняется за {self._check_interval}с: "
                        f"dx={dx:.3f}м, dy={dy:.3f}м, dyaw={dyaw:.3f}рад "
                        f"(пороги: xy={self._stuck_xy_threshold}м, yaw={self._stuck_yaw_threshold}рад). "
                        f"Робот мог застрять."
                    )
                    self._last_warn_time = now
            
            # Сброс точки отсчёта
            self._check_start_x = current_x
            self._check_start_y = current_y
            self._check_start_yaw = current_yaw
            self._last_check_time = now
        
        vx, vy, omega, duration = self._command_func(driver)
        driver.move_time(vx, vy, omega, duration)
        self._started = True
        self._state = "RUNNING"
        return "RUNNING"

    def reset(self):
        self._started = False
        self._last_warn_time = 0.0
        self._last_check_time = 0.0
        self._state = "IDLE"

# ==============================================================================
# 3.1 ВСПОМОГАТЕЛЬНЫЕ УЗЛЫ (WaitCondition, RepeatUntil, TimerAction)
# ==============================================================================

class WaitCondition(BTNode):
    """
    Ждать пока условие не станет истиной.
    Возвращает RUNNING пока ждёт, SUCCESS когда условие выполнено,
    FAILURE если истёк таймаут.
    """

    def __init__(self, check_func, timeout: float = 10.0, name: str = "Wait"):
        self._check = check_func       # (driver) -> bool
        self._timeout = timeout
        self.name = name
        self._start_time = None
        self._state = "IDLE"

    def _log(self, driver, msg: str):
        driver.get_logger().info(f"[BT] {self.name}: {msg}")

    def _warn(self, driver, msg: str):
        driver.get_logger().warning(f"[BT] {self.name}: {msg}")

    def tick(self, driver: RobotDriver) -> str:
        if self._start_time is None:
            self._start_time = driver.get_clock().now().nanoseconds * 1e-9
            self._state = "RUNNING"

        if driver.get_clock().now().nanoseconds * 1e-9 - self._start_time > self._timeout:
            self._warn(driver, f"Таймаут {self._timeout}с истёк")
            self._start_time = None
            self._state = "FAILURE"
            return "FAILURE"

        if self._check(driver):
            self._log(driver, "Условие выполнено")
            self._start_time = None
            self._state = "SUCCESS"
            return "SUCCESS"

        self._state = "RUNNING"

        return "RUNNING"

    def reset(self):
        self._start_time = None
        self._state = "IDLE"


class RepeatUntil(BTNode):
    """
    Повторяет дочерний узел пока условие не станет истиной.
    Когда условие выполнено — возвращает SUCCESS.
    Если дочерний узел вернул FAILURE — возвращает FAILURE.
    """

    def __init__(self, child: BTNode, stop_condition, name: str = "RepeatUntil"):
        self.child = child
        self._stop = stop_condition     # (driver) -> bool
        self.name = name
        self._state = "IDLE"

    def _log(self, driver, msg: str):
        driver.get_logger().info(f"[BT] {self.name}: {msg}")

    def tick(self, driver: RobotDriver) -> str:
        if self._stop(driver):
            self._state = "SUCCESS"
            self._log(driver, "Условие остановки выполнено")
            return "SUCCESS"

        status = self.child.tick(driver)
        self._state = status
        if status == "FAILURE":
            return "FAILURE"
        if status == "SUCCESS":
            self.child.reset()  # <-- ВОТ ЭТО НУЖНО ДОБАВИТЬ
            self._state = "RUNNING"
        return "RUNNING"

    def reset(self):
        self._state = "IDLE"
        self.child.reset()


class TimerAction(BTNode):
    """
    Выполняет дочерний узел заданное время, затем возвращает SUCCESS.
    Если дочерний узел вернул FAILURE — возвращает FAILURE досрочно.
    """

    def __init__(self, child: BTNode, duration: float, name: str = "Timer"):
        self.child = child
        self.duration = duration
        self.name = name
        self._start_time = None
        self._state = "IDLE"

    def _log(self, driver, msg: str):
        driver.get_logger().info(f"[BT] {self.name}: {msg}")

    def tick(self, driver: RobotDriver) -> str:
        if self._start_time is None:
            self._start_time = driver.get_clock().now().nanoseconds * 1e-9
            self._state = "RUNNING"

        if driver.get_clock().now().nanoseconds * 1e-9 - self._start_time > self.duration:
            self._log(driver, f"Время {self.duration}с истекло")
            self._state = "SUCCESS"
            return "SUCCESS"

        status = self.child.tick(driver)
        if status == "FAILURE":
            self._state = "FAILURE"
            return "FAILURE"
        self._state = "RUNNING"
        return "RUNNING"

    def reset(self):
        self._start_time = None
        self._state = "IDLE"
        self.child.reset()




# ==============================================================================
# 3.3 SUBTREE — переиспользуемый фрагмент дерева
# ==============================================================================

class SubTree(BTNode):
    """Ссылка на именованный фрагмент дерева."""

    def __init__(self, name: str = "SubTree"):
        self.name = name
        self._root: BTNode = None
        self._state = "IDLE"

    def set_root(self, root: BTNode):
        self._root = root

    def tick(self, driver: RobotDriver) -> str:
        if self._root is None:
            driver.get_logger().error(f"[BT] {self.name}: корень не установлен")
            self._state = "FAILURE"
            return "FAILURE"
        status = self._root.tick(driver)
        self._state = status
        return status

    def reset(self):
        self._state = "IDLE"
        if self._root is not None:
            self._root.reset()

# ==============================================================================
# 4. ДИСПЕТЧЕР МИССИЙ И УЗЕЛ ПРИНЯТИЯ РЕШЕНИЙ ИИ
# ==============================================================================

class MissionDispatcher(RobotDriver):
    """
    Диспетчер миссий.
    Хранит реестр миссий, запускает и прерывает их.
    Может использоваться как самостоятельно, так и внутри AIDecisionAction.
    """

    def __init__(self, node_name: str = 'mission_dispatcher', name: str = "Mission dispatcher"):
        super().__init__(node_name)

        self._missions: dict = {}
        self._current_mission_name: str = "IDLE"
        self._current_mission_root: BTNode = None
        self._state = "IDLE"
        self.name = name

        self._ticker = self.create_timer(0.2, self._mission_loop)
        self.get_logger().info("Диспетчер миссий готов")

    def register_mission(self, name: str, root_node: BTNode):
        self._missions[name] = root_node
        self.get_logger().info(f"Миссия зарегистрирована: {name}")

    def start_mission(self, name: str):
        if name not in self._missions:
            self.get_logger().error(f"Миссия не найдена: {name}")
            return

        if self._current_mission_name == name:
            return

        self.stop()
        self.get_logger().info(f"Запуск миссии: {self._current_mission_name} → {name}")
        self._current_mission_name = name
        self._current_mission_root = self._missions[name]
        self._state = "RUNNING"

    def abort_mission(self):
        self.get_logger().info(f"Прерывание миссии: {self._current_mission_name}")
        self.stop()
        self._current_mission_name = "IDLE"
        self._current_mission_root = None
        self._state = "IDLE"

    def get_current_mission(self) -> str:
        return self._current_mission_name

    def _mission_loop(self):
        #self.send_heartbeat()

        self._state = "RUNNING"
        if self._current_mission_root is None:
            self._state = "IDLE"
            return

        status = self._current_mission_root.tick(self)
        self._state = status

        if status == "SUCCESS":
            self.get_logger().info(f"Миссия {self._current_mission_name} завершена успешно")
            self._current_mission_name = "IDLE"
            self._current_mission_root = None
        elif status == "FAILURE":
            self.get_logger().error(f"Миссия {self._current_mission_name} провалена")
            self.stop()
            self._current_mission_name = "IDLE"
            self._current_mission_root = None

class AIDecisionAction(BTNode):
    """Узел принятия решений."""

    def __init__(self, decision_func, name: str = "AI"):
        self._decision_func = decision_func
        self.name = name
        self._current_mission_name = None
        self._current_mission_root = None
        self._missions = {}
        self._state = "IDLE"

    def register_mission(self, name: str, root_node: BTNode):
        self._missions[name] = root_node

    def _switch_mission(self, driver: MissionDispatcher, new_name: str):
        if self._current_mission_name == new_name:
            return

        driver.stop()
        driver.get_logger().info(f"[AI] Смена миссии: {self._current_mission_name} → {new_name}")
        self._current_mission_name = new_name
        self._current_mission_root = self._missions.get(new_name)

    def tick(self, driver: MissionDispatcher) -> str:
        new_mission = self._decision_func(driver)
        self._switch_mission(driver, new_mission)

        if self._current_mission_root is not None:
            status = self._current_mission_root.tick(driver)
            if status == "FAILURE":
                driver.get_logger().error(f"[AI] Миссия {self._current_mission_name} провалена")
                self._current_mission_name = None
                self._current_mission_root = None
                self._state = "FAILURE"
                return "FAILURE"
            self._state = "RUNNING"
            return "RUNNING"

        self._state = "RUNNING"
        return "RUNNING"

    def reset(self):
        self._current_mission_name = None
        self._current_mission_root = None
        self._state = "IDLE"

# ==============================================================================
# 5. ПРИМЕР СЦЕНАРИЯ — КВАДРАТ НА ДЕРЕВЕ ПОВЕДЕНИЯ
# ==============================================================================

class BehaviorTreeSquare(RobotDriver):
    """
    Робот проезжает квадрат со стороной side_length метров.
    Использует дерево поведения: Sequence из 8 MoveRelativeAction,
    Heartbeat в параллель, Retry на прямолинейных участках.
    """

    def __init__(self, side_length: float = 1.0):
        super().__init__('bt_square')

        self._bt_root = Parallel(
            [
                HeartbeatAction(),
                Sequence([
                    LogAction("Старт миссии: Квадрат"),
                    RetryAction(MoveRelativeAction(dx=side_length, dy=0.0, dyaw_deg=0.0,
                                                   name="Вперёд 1")),
                    MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0, name="Поворот 1"),
                    RetryAction(MoveRelativeAction(dx=side_length, dy=0.0, dyaw_deg=0.0,
                                                   name="Вперёд 2")),
                    MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0, name="Поворот 2"),
                    RetryAction(MoveRelativeAction(dx=side_length, dy=0.0, dyaw_deg=0.0,
                                                   name="Вперёд 3")),
                    MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0, name="Поворот 3"),
                    RetryAction(MoveRelativeAction(dx=side_length, dy=0.0, dyaw_deg=0.0,
                                                   name="Вперёд 4")),
                    MoveRelativeAction(dx=0.0, dy=0.0, dyaw_deg=90.0, name="Поворот 4"),
                    LogAction("Миссия завершена успешно!"),
                ]),
            ],
            required_successes=1
        )

        self._active = False
        self._ticker = self.create_timer(0.2, self._loop)
        self.get_logger().info(f"[BT Square] Дерево готово. Сторона: {side_length} м")

    def _loop(self):
        if not self.is_connected():
            return

        if not self._active:
            print("\nРобот готов к миссии 'Квадрат'.")
            input("Нажмите Enter для старта...")
            self._active = True
            return

        status = self._bt_root.tick(self)

        if status == "SUCCESS":
            self.get_logger().info("Дерево выполнено успешно!")
            self.stop()
            self.create_timer(0.5, lambda: rclpy.shutdown())
        elif status == "FAILURE":
            self.get_logger().error("Дерево вернуло FAILURE. Аварийная остановка.")
            self.stop()
            self.create_timer(0.5, lambda: rclpy.shutdown())


# ==============================================================================
# 6. ТОЧКА ВХОДА
# ==============================================================================

def main():
    import sys
    rclpy.init()
    side = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    node = BehaviorTreeSquare(side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
