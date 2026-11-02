#Copyright (c) 2026 Jurij Polikarpov
import sys
import os
import time
import math
import xml.etree.ElementTree as ET
import pygame

current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_dir)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

class MapOdomViewerNode(Node):
    def __init__(self):
        super().__init__('map_odom_viewer_node')
        
        self.win_width = 800
        self.win_height = 600
        self.scale = 150.0  
        self.min_scale = 1.0
        self.max_scale = 600.0
        
        self.map_data = None
        self.scan_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Геометрические параметры по умолчанию (перезапишутся из URDF)
        self.geometry = {
            'chassis': {'size_x': 0.27, 'size_y': 0.15},
            'camera': {'size_x': 0.04, 'size_y': 0.04, 'x': 0.13, 'y': 0.0},
            'lidar': {'radius': 0.025, 'x': -0.035, 'y': 0.0},
            'wheels': [] 
        }
        
        # Подписываемся на стандартный robot_description и сенсоры
        self.create_subscription(String, '/robot_description', self.description_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # --- ИСПРАВЛЕНИЕ ДЛЯ LINUX: Инициализируем графику строго в Main Thread ---
        pygame.init()
        self.screen = pygame.display.set_mode((self.win_width, self.win_height))
        pygame.display.set_caption("POND Viewer - Dynamic URDF Ego-View")
        self.clock = pygame.time.Clock()
        
        # Вместо threading.Thread создаем пассивный ROS 2 таймер на 30 Гц (~0.033 сек)
        self.create_timer(1.0 / 30.0, self.gui_timer_callback)
        print("[Viewer] Окно Ego-View успешно инициализировано в Main Thread.", flush=True)

    def map_callback(self, msg): self.map_data = msg
    def scan_callback(self, msg): self.scan_data = msg

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def description_callback(self, msg):
        try:
            self.parse_urdf_string(msg.data)
            print("[Viewer] Геометрия робота успешно обновлена из URDF топика.", flush=True)
        except Exception as e:
            print(f"[Viewer] Ошибка парсинга URDF: {e}", flush=True)

    def parse_urdf_string(self, urdf_str):
        root = ET.fromstring(urdf_str)
        new_wheels = []
        joints = {}
        for joint in root.findall('joint'):
            name = joint.get('name')
            child = joint.find('child').get('link')
            origin = joint.find('origin')
            xyz = [0.0, 0.0, 0.0]
            if origin is not None and origin.get('xyz'):
                xyz = [float(x) for x in origin.get('xyz').split()]
            joints[child] = {'xyz': xyz}

        for link in root.findall('link'):
            link_name = link.get('name')
            visual = link.find('visual')
            if visual is None: continue
            geom = visual.find('geometry')
            if geom is None: continue

            if link_name == 'chassis_link':
                box = geom.find('box')
                if box is not None:
                    size = [float(s) for s in box.get('size').split()]
                    self.geometry['chassis']['size_x'] = size[0]
                    self.geometry['chassis']['size_y'] = size[1]

            elif 'wheel' in link_name:
                cylinder = geom.find('cylinder')
                if cylinder is not None and link_name in joints:
                    r = float(cylinder.get('radius'))
                    length = float(cylinder.get('length'))
                    xyz = joints[link_name]['xyz']
                    new_wheels.append({'x': xyz[0], 'y': xyz[1], 'radius': r, 'length': length})

            elif link_name == 'camera_link' and link_name in joints:
                box = geom.find('box')
                if box is not None:
                    size = [float(s) for s in box.get('size').split()]
                    xyz = joints[link_name]['xyz']
                    self.geometry['camera'] = {'size_x': size[0], 'size_y': size[1], 'x': xyz[0], 'y': xyz[1]}

            elif link_name == 'lidar_link' and link_name in joints:
                cylinder = geom.find('cylinder')
                if cylinder is not None:
                    r = float(cylinder.get('radius'))
                    xyz = joints[link_name]['xyz']
                    self.geometry['lidar'] = {'radius': r, 'x': xyz[0], 'y': xyz[1]}

        if new_wheels:
            self.geometry['wheels'] = new_wheels
    def gui_timer_callback(self):
        """ Один шаг рендеринга графики Pygame, вызываемый экзекутором """
        # 1. МАСШТАБИРОВАНИЕ КОЛЕСИКОМ И ОБРАБОТКА КРЕСТИКА
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:  # Вверх (Приближение)
                    self.scale = min(self.max_scale, self.scale * 1.1)
                elif event.button == 5:  # Вниз (Отдаление)
                    self.scale = max(self.min_scale, self.scale / 1.1)

        # Фон RViz Grey
        self.screen.fill((50, 50, 50))  
        center_x = self.win_width // 2
        center_y = self.win_height // 2

#         # 2. ОТРИСОВКА КАРТЫ ОТ ПЕРВОГО ЛИЦА
#         if self.map_data and hasattr(self.map_data, 'data') and self.map_data.data:
#             w = self.map_data.info.width
#             h = self.map_data.info.height
#             res = self.map_data.info.resolution
#             
#             orig_x = self.map_data.info.origin.position.x
#             orig_y = self.map_data.info.origin.position.y
#             cell_px = res * self.scale
#             
#             for y in range(h):
#                 for x in range(w):
#                     idx = x + y * w
#                     val = self.map_data.data[idx]
#                     
#                     if val == -1: 
#                         continue
#                         
#                     cell_world_x = orig_x + (x + 0.5) * res
#                     cell_world_y = orig_y + (y + 0.5) * res
#                     
#                     dx = cell_world_x - self.robot_x
#                     dy = cell_world_y - self.robot_y
#                     
#                     rx = dx * math.cos(self.robot_yaw) + dy * math.sin(self.robot_yaw)
#                     ry = -dx * math.sin(self.robot_yaw) + dy * math.cos(self.robot_yaw)
#                     
#                     scr_x = int(center_x - ry * self.scale)
#                     scr_y = int(center_y - rx * self.scale)
#                     
#                     if -cell_px <= scr_x <= self.win_width + cell_px and -cell_px <= scr_y <= self.win_height + cell_px:
#                         rect = pygame.Rect(int(scr_x - cell_px/2), int(scr_y - cell_px/2), math.ceil(cell_px), math.ceil(cell_px))
#                         if val == 100:
#                             pygame.draw.rect(self.screen, (200, 200, 200), rect) # Стена
#                         elif val == 0:
#                             pygame.draw.rect(self.screen, (30, 30, 30), rect)    # Свободно
#             
#         else:
#             # === ИСПРАВЛЕНИЕ ДЛЯ LINUX: Убираем SysFont, чтобы исключить падение таймера ===
#             # Просто рисуем небольшой индикаторный круг в углу экрана, показывающий, что окно ЖИВО
#             pygame.draw.circle(self.screen, (100, 0, 0), (20, 20), 8)
                    # 2. ОТРИСОВКА КАРТЫ ОТ ПЕРВОГО ЛИЦА (С ЧЕСТНЫМ ВРАЩЕНИЕМ КВАДРАТОВ)
        # 2. ОТРИСОВКА КАРТЫ ОТ ПЕРВОГО ЛИЦА (ЧЕСТНЫЙ МАТЕМАТИЧЕСКИЙ ПОВОРОТ УГЛОВ)
        if self.map_data and hasattr(self.map_data, 'data') and self.map_data.data:
            w = self.map_data.info.width
            h = self.map_data.info.height
            res = self.map_data.info.resolution
            
            orig_x = self.map_data.info.origin.position.x
            orig_y = self.map_data.info.origin.position.y
            
            half_res = res / 2.0
            cos_yaw = math.cos(self.robot_yaw)
            sin_yaw = math.sin(self.robot_yaw)
            
            # Дельты 4 углов ячейки в метрах мира относительно её центра
            corner_offsets = [
                (-half_res,  half_res),  # Верхний левый угол
                ( half_res,  half_res),  # Верхний правый угол
                ( half_res, -half_res),  # Нижний правый угол
                (-half_res, -half_res)   # Нижний левый угол
            ]
            
            for y in range(h):
                for x in range(w):
                    idx = x + y * w
                    val = self.map_data.data[idx]
                    
                    if val == -1: 
                        continue
                        
                    # Центр ячейки в абсолютных координатах мира
                    cell_world_x = orig_x + (x + 0.5) * res
                    cell_world_y = orig_y + (y + 0.5) * res
                    
                    # Проверка на вылет за экран по центру (для оптимизации)
                    dx_c = cell_world_x - self.robot_x
                    dy_c = cell_world_y - self.robot_y
                    rx_c = dx_c * cos_yaw + dy_c * sin_yaw
                    ry_c = -dx_c * sin_yaw + dy_c * cos_yaw
                    scr_x_c = center_x - ry_c * self.scale
                    scr_y_c = center_y - rx_c * self.scale
                    
                    pad = res * self.scale * 2.0
                    if scr_x_c < -pad or scr_x_c > self.win_width + pad or scr_y_c < -pad or scr_y_c > self.win_height + pad:
                        continue
                    
                    # Считаем экранные координаты для КАЖДОГО из 4 углов ячейки индивидуально!
                    points = []
                    for ox, oy in corner_offsets:
                        # Абсолютные координаты угла в мире
                        corner_world_x = cell_world_x + ox
                        corner_world_y = cell_world_y + oy
                        
                        # Смещение угла относительно робота
                        dx = corner_world_x - self.robot_x
                        dy = corner_world_y - self.robot_y
                        
                        # ВРАЩЕНИЕ УГЛА: Переводим в локальную систему робота (Ego-View)
                        rx = dx * cos_yaw + dy * sin_yaw
                        ry = -dx * sin_yaw + dy * cos_yaw
                        
                        # Перевод локальных координат в пиксели экрана
                        scr_x = int(center_x - ry * self.scale)
                        scr_y = int(center_y - rx * self.scale)
                        points.append((scr_x, scr_y))
                    
                    # Отрисовываем честный, повернутый под нужным углом четырехугольник!
                    if val == 100:
                        pygame.draw.polygon(self.screen, (200, 200, 200), points) # Стена
                        pygame.draw.polygon(self.screen, (140, 140, 140), points, 1) # Тонкая сетка блоков
                    elif val == 0:
                        pygame.draw.polygon(self.screen, (30, 30, 30), points)    # Свободно
        else:
            pygame.draw.circle(self.screen, (100, 0, 0), (20, 20), 8)


        # 3. ОТРИСОВКА СКАНИРОВАНИЯ ЛИДАРА (/scan)
        if self.scan_data and hasattr(self.scan_data, 'ranges'):
            angle_min = self.scan_data.angle_min
            angle_increment = self.scan_data.angle_increment
            
            lidar_offset_x = self.geometry['lidar']['x']
            lidar_offset_y = self.geometry['lidar']['y']
            
            lidar_scr_x = int(center_x - lidar_offset_y * self.scale)
            lidar_scr_y = int(center_y - lidar_offset_x * self.scale)
            
            for i, r in enumerate(self.scan_data.ranges):
                if math.isnan(r) or math.isinf(r) or r <= 0.0:
                    continue
                    
                beam_angle = angle_min + i * angle_increment
                lx = r * math.cos(beam_angle)
                ly = r * math.sin(beam_angle)
                
                rx = lx + lidar_offset_x
                ry = ly + lidar_offset_y
                
                scan_scr_x = int(center_x - ry * self.scale)
                scan_scr_y = int(center_y - rx * self.scale)
                
                if 0 <= scan_scr_x < self.win_width and 0 <= scan_scr_y < self.win_height:
                    pygame.draw.line(self.screen, (0, 70, 0), (lidar_scr_x, lidar_scr_y), (scan_scr_x, scan_scr_y), 1)
                    pygame.draw.circle(self.screen, (0, 255, 0), (scan_scr_x, scan_scr_y), 2)

        # 4. ДИНАМИЧЕСКИЙ РЕНДЕР РОБОТА ПО ДАННЫМ ИЗ URDF
        if self.geometry['wheels']:
            for wh in self.geometry['wheels']:
                w_len_px = int(wh['radius'] * 2 * self.scale)
                w_wid_px = int(wh['length'] * self.scale)
                
                w_off_x = wh['x']
                w_off_y = wh['y']
                
                wh_scr_x = int(center_x - w_off_y * self.scale)
                wh_scr_y = int(center_y - w_off_x * self.scale)
                
                wheel_rect = pygame.Rect(
                    wh_scr_x - int(w_wid_px / 2),
                    wh_scr_y - int(w_len_px / 2),
                    w_wid_px,
                    w_len_px
                )
                pygame.draw.rect(self.screen, (20, 20, 20), wheel_rect)
                pygame.draw.rect(self.screen, (120, 120, 120), wheel_rect, 1)
        
        ch = self.geometry['chassis']
        b_len_px = int(ch['size_x'] * self.scale)
        b_wid_px = int(ch['size_y'] * self.scale)
        
        body_rect = pygame.Rect(
            center_x - int(b_wid_px / 2),
            center_y - int(b_len_px / 2),
            b_wid_px,
            b_len_px
        )
        pygame.draw.rect(self.screen, (0, 90, 220), body_rect)
        pygame.draw.rect(self.screen, (255, 255, 255), body_rect, 2)

        cam = self.geometry['camera']
        cam_len_px = int(cam['size_x'] * self.scale)
        cam_wid_px = int(cam['size_y'] * self.scale)
        
        cam_scr_x = int(center_x - cam['y'] * self.scale)
        cam_scr_y = int(center_y - cam['x'] * self.scale)
        
        camera_rect = pygame.Rect(
            cam_scr_x - int(cam_wid_px / 2),
            cam_scr_y - int(cam_len_px / 2),
            cam_wid_px,
            cam_len_px
        )
        pygame.draw.rect(self.screen, (30, 30, 30), camera_rect)
        pygame.draw.rect(self.screen, (0, 255, 255), camera_rect, 1)

        lid = self.geometry['lidar']
        lidar_r_px = max(4, int(lid['radius'] * self.scale))
        
        lid_scr_x = int(center_x - lid['y'] * self.scale)
        lid_scr_y = int(center_y - lid['x'] * self.scale)
        
        pygame.draw.circle(self.screen, (240, 190, 0), (lid_scr_x, lid_scr_y), lidar_r_px)
        pygame.draw.circle(self.screen, (0, 0, 0), (lid_scr_x, lid_scr_y), lidar_r_px, 2)
        pygame.draw.circle(self.screen, (255, 0, 255), (lid_scr_x, lid_scr_y), 3)

        pygame.draw.line(self.screen, (255, 0, 0), (center_x, center_y), (center_x, center_y - int(ch['size_x']/2 * self.scale) - 15), 2)

        #font_ui = pygame.font.SysFont(None, 20)
        #ui_text = font_ui.render(f"Scale: {self.scale:.1f} px/m | URDF Ego-Mode", True, (255, 255, 0))
        #self.screen.blit(ui_text, (10, self.win_height - 25))
            
        pygame.display.flip()
        self.clock.tick(30)

def main():
    # Наш executors.py на Linux перехватит этот вызов и выполнит gui_timer_callback в Main-потоке!
    rclpy.init()
    node = MapOdomViewerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
