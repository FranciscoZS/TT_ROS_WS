# #!/usr/bin/env python3
# """
# Nodo RRT Planner Simplificado
# - Recibe waypoints desde ROS2
# - Al cerrar (Ctrl+C), genera ruta RRT y mapa
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# import math
# import random
# import matplotlib.pyplot as plt
# from datetime import datetime
# import json
# import os
# import sys

# class SimpleRRTPlanner(Node):
#     def __init__(self):
#         super().__init__('simple_rrt_planner')
        
#         self.get_logger().info("🔄 Inicializando RRT Planner Simplificado...")
        
#         # Parámetros básicos
#         self.step_size = 0.40  # metros
#         self.max_iterations = 300
#         self.goal_bias = 0.70   # 20% probabilidad de ir al objetivo
        
#         # Almacenamiento
#         self.waypoints = []  # Lista de (x, y)
#         self.planned_path = []
        

#         # Suscriptor
#         self.waypoint_sub = self.create_subscription(
#             Float32MultiArray,
#             '/waypoints',
#             self.waypoint_callback,
#             10
#         )
        
#         self.get_logger().info("✅ RRT Planner listo. Esperando waypoints...")
#         self.get_logger().info("   Presiona Ctrl+C para generar ruta y mapa")
    
#     def waypoint_callback(self, msg):
#         """Recibe waypoints del nodo de monitoreo"""
#         if len(msg.data) >= 3:
#             x, y, theta = msg.data[0], msg.data[1], msg.data[2]
            
#             # Señal especial de limpieza
#             if x == -1.0 and y == -1.0 and theta == -1.0:
#                 self.waypoints = []
#                 self.get_logger().info("🗑️ Waypoints limpiados")
#                 return
            
#             self.waypoints.append((x, y))
            
#             self.get_logger().info(
#                 f"📍 Waypoint {len(self.waypoints)}: ({x:.3f}, {y:.3f})"
#             )
            
#             # Guardar automáticamente
#             self.save_waypoints()
    
#     def save_waypoints(self):
#         """Guarda waypoints en archivo"""
#         try:
#             data = {
#                 'waypoints': self.waypoints,
#                 'timestamp': datetime.now().isoformat(),
#                 'count': len(self.waypoints)
#             }
            
#             with open(f'/home/orangepi/rrt_{self.timestamp}_waypoints.json', 'w') as f:
#                 json.dump(data, f, indent=2)
                
#         except Exception as e:
#             self.get_logger().error(f"Error guardando waypoints: {e}")
    
#     def generate_rrt_path(self):
#         """Genera ruta RRT simple que conecta waypoints en orden"""
#         if len(self.waypoints) < 2:
#             self.get_logger().warn("⚠️ Se necesitan al menos 2 waypoints")
#             return []
        
#         self.get_logger().info(f"🎯 Generando ruta RRT para {len(self.waypoints)} waypoints...")
        
#         complete_path = []
        
#         # Conectar waypoints en orden
#         for i in range(len(self.waypoints) - 1):
#             start = self.waypoints[i]
#             goal = self.waypoints[i + 1]
            
#             segment = self.rrt_connect(start, goal)
            
#             if complete_path:
#                 # Evitar duplicar el primer punto
#                 complete_path.extend(segment[1:])
#             else:
#                 complete_path.extend(segment)
            
#             self.get_logger().info(
#                 f"  Segmento {i+1}: {len(segment)} puntos "
#                 f"(distancia: {self.distance(start, goal):.2f}m)"
#             )
        
#         self.planned_path = complete_path
#         return complete_path
    
#     def rrt_connect(self, start, goal):
#         """RRT simple para conectar dos puntos"""
#         nodes = [start]
#         parents = [-1]
        
#         for iteration in range(self.max_iterations):
#             # Punto aleatorio con bias hacia el goal
#             if random.random() < self.goal_bias:
#                 rand_point = goal
#             else:
#                 # Punto aleatorio en área razonable
#                 rand_point = (
#                     random.uniform(min(start[0], goal[0]) - 2, max(start[0], goal[0]) + 2),
#                     random.uniform(min(start[1], goal[1]) - 2, max(start[1], goal[1]) + 2)
#                 )
            
#             # Encontrar nodo más cercano
#             nearest_idx = self.find_nearest(nodes, rand_point)
#             nearest_node = nodes[nearest_idx]
            
#             # Extender hacia el punto aleatorio
#             new_node = self.steer(nearest_node, rand_point)
            
#             # Añadir al árbol
#             nodes.append(new_node)
#             parents.append(nearest_idx)
            
#             # Verificar si alcanzamos el goal
#             if self.distance(new_node, goal) < self.step_size:
#                 # Reconstruir ruta
#                 path = [goal]
#                 current_idx = len(nodes) - 1
                
#                 while current_idx != -1:
#                     path.append(nodes[current_idx])
#                     current_idx = parents[current_idx]
                
#                 path.reverse()
#                 return path
        
#         # Si no se encontró ruta, devolver línea recta
#         return [start, goal]
    
#     def steer(self, from_node, to_node):
#         """Mueve desde from_node hacia to_node con paso máximo"""
#         dist = self.distance(from_node, to_node)
#         if dist <= self.step_size:
#             return to_node
        
#         ratio = self.step_size / dist
#         new_x = from_node[0] + ratio * (to_node[0] - from_node[0])
#         new_y = from_node[1] + ratio * (to_node[1] - from_node[1])
        
#         return (new_x, new_y)
    
#     def find_nearest(self, nodes, point):
#         """Encuentra el nodo más cercano"""
#         min_dist = float('inf')
#         nearest_idx = 0
        
#         for i, node in enumerate(nodes):
#             dist = self.distance(node, point)
#             if dist < min_dist:
#                 min_dist = dist
#                 nearest_idx = i
        
#         return nearest_idx
    
#     def distance(self, p1, p2):
#         """Distancia euclidiana"""
#         return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
#     def create_map(self):
#         """Crea y guarda mapa con waypoints y ruta"""
#         if not self.waypoints:
#             self.get_logger().warn("⚠️ No hay waypoints para crear mapa")
#             return None
        
#         try:
#             timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
#             # Crear figura
#             fig, ax = plt.subplots(figsize=(10, 8))
            
#             # Calcular límites automáticos
#             all_points = self.waypoints
#             if self.planned_path:
#                 all_points = all_points + self.planned_path
            
#             xs = [p[0] for p in all_points]
#             ys = [p[1] for p in all_points]
            
#             x_min, x_max = min(xs), max(xs)
#             y_min, y_max = min(ys), max(ys)
            
#             # Añadir margen
#             margin = max((x_max - x_min), (y_max - y_min)) * 0.2
#             if margin == 0:
#                 margin = 1.0
            
#             x_min -= margin
#             x_max += margin
#             y_min -= margin
#             y_max += margin
            
#             # Configurar gráfico
#             ax.set_xlim(x_min, x_max)
#             ax.set_ylim(y_min, y_max)
#             ax.set_aspect('equal')
#             ax.grid(True, alpha=0.3, linestyle='--')
#             ax.set_xlabel('X (metros)', fontsize=12)
#             ax.set_ylabel('Y (metros)', fontsize=12)
#             ax.set_title(f'Mapa de Navegación - {timestamp}', fontsize=14, fontweight='bold')
            
#             # Dibujar waypoints
#             if self.waypoints:
#                 waypoints_x = [p[0] for p in self.waypoints]
#                 waypoints_y = [p[1] for p in self.waypoints]
                
#                 ax.scatter(waypoints_x, waypoints_y, c='blue', s=200, 
#                           marker='o', label='Waypoints', zorder=5, edgecolors='black', linewidth=2)
                
#                 # Numerar waypoints
#                 for i, (x, y) in enumerate(self.waypoints):
#                     ax.text(x, y + margin/15, f'W{i+1}', 
#                            fontsize=11, ha='center', va='bottom', 
#                            fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', 
#                                                        facecolor='yellow', alpha=0.7))
            
#             # Dibujar ruta planeada
#             if self.planned_path and len(self.planned_path) > 1:
#                 path_x = [p[0] for p in self.planned_path]
#                 path_y = [p[1] for p in self.planned_path]
                
#                 ax.plot(path_x, path_y, 'g-', linewidth=3, label='Ruta RRT', zorder=3, alpha=0.7)
#                 ax.scatter(path_x, path_y, c='green', s=20, alpha=0.5, zorder=4)
                
#                 # Marcar inicio y fin de ruta
#                 ax.scatter(path_x[0], path_y[0], c='orange', s=300, 
#                           marker='*', label='Inicio', zorder=6, edgecolors='black', linewidth=2)
#                 ax.scatter(path_x[-1], path_y[-1], c='red', s=300, 
#                           marker='*', label='Fin', zorder=6, edgecolors='black', linewidth=2)
            
#             # Leyenda
#             ax.legend(loc='upper right', fontsize=10)
            
#             # Añadir información estadística
#             info_text = f'Waypoints: {len(self.waypoints)}\n'
#             if self.planned_path:
#                 path_length = sum(self.distance(self.planned_path[i], self.planned_path[i+1]) 
#                                  for i in range(len(self.planned_path)-1))
#                 info_text += f'Longitud ruta: {path_length:.2f}m\n'
#                 info_text += f'Puntos ruta: {len(self.planned_path)}'
            
#             ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
#                    fontsize=10, verticalalignment='top',
#                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
#             # Guardar como PNG
#             png_filename = f"/home/orangepi/map_test/mapa_navegacion_{timestamp}.png"
#             plt.tight_layout()
#             plt.savefig(png_filename, dpi=150, bbox_inches='tight')
            
#             # Guardar como BMP (más ligero)
#             #bmp_filename = f"/home/orangepi/map_test/mapa_navegacion_{timestamp}.bmp"
#             #plt.savefig(bmp_filename, dpi=100, format='bmp', bbox_inches='tight')
            
#             plt.close(fig)
            
#             self.get_logger().info(f"🗺️  Mapa guardado como PNG: {png_filename}")
#             #self.get_logger().info(f"🗺️  Mapa guardado como BMP: {bmp_filename}")
            
#             # Guardar datos de la ruta
#             self.save_path_data(png_filename.replace('.png', '_ruta.json'))

#             instr = self.generate_movement_instructions(self.planned_path)

#             self.save_instructions(instr,self.waypoints)
            
#             return png_filename#, bmp_filename
            
#         except Exception as e:
#             self.get_logger().error(f"❌ Error creando mapa: {e}")
#             return None, None
    
#     def save_path_data(self, filename):
#         """Guarda datos detallados de la ruta"""
#         try:
#             data = {
#                 'waypoints': self.waypoints,
#                 'planned_path': self.planned_path,
#                 'timestamp': datetime.now().isoformat(),
#                 'path_length': self.calculate_path_length(),
#                 'waypoint_count': len(self.waypoints),
#                 'path_point_count': len(self.planned_path)
#             }
            
#             with open(filename, 'w') as f:
#                 json.dump(data, f, indent=2)
                
#             self.get_logger().info(f"💾 Datos de ruta guardados: {filename}")
            
#         except Exception as e:
#             self.get_logger().error(f"Error guardando datos de ruta: {e}")
    
#     def calculate_path_length(self):
#         """Calcula longitud total de la ruta"""
#         if len(self.planned_path) < 2:
#             return 0.0
        
#         total = 0.0
#         for i in range(len(self.planned_path) - 1):
#             total += self.distance(self.planned_path[i], self.planned_path[i+1])
        
#         return total
    
#     def print_summary(self):
#         """Imprime resumen de la planificación"""
#         self.get_logger().info("\n" + "="*50)
#         self.get_logger().info("📊 RESUMEN DE PLANIFICACIÓN")
#         self.get_logger().info("="*50)
#         self.get_logger().info(f"Waypoints registrados: {len(self.waypoints)}")
        
#         if self.waypoints:
#             self.get_logger().info("\nLista de waypoints:")
#             for i, (x, y) in enumerate(self.waypoints):
#                 self.get_logger().info(f"  W{i+1}: ({x:.3f}, {y:.3f})")
        
#         if self.planned_path:
#             path_length = self.calculate_path_length()
#             self.get_logger().info(f"\nRuta generada: {len(self.planned_path)} puntos")
#             self.get_logger().info(f"Longitud total: {path_length:.2f} metros")
#             self.get_logger().info(f"Primer punto: ({self.planned_path[0][0]:.3f}, {self.planned_path[0][1]:.3f})")
#             self.get_logger().info(f"Último punto: ({self.planned_path[-1][0]:.3f}, {self.planned_path[-1][1]:.3f})")
        
#         self.get_logger().info("="*50 + "\n")

#     def generate_movement_instructions(self, path):
#         """Convierte la ruta RRT en instrucciones de movimiento simples"""
#         if len(path) < 2:
#             return []
        
#         instructions = []
#         current_theta = 0.0  # Asumimos que el robot comienza orientado a 0°
        
#         for i in range(len(path) - 1):
#             current_point = np.array(path[i])
#             next_point = np.array(path[i + 1])
            
#             # Calcular vector de dirección
#             direction_vector = next_point - current_point
#             distance = np.linalg.norm(direction_vector)
            
#             # Calcular ángulo necesario
#             target_theta = math.atan2(direction_vector[1], direction_vector[0])
            
#             # Normalizar ángulo
#             angle_diff = target_theta - current_theta
#             angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi
            
#             # Si hay rotación necesaria
#             if abs(angle_diff) > 0.01:  # Tolerancia de 0.57°
#                 instructions.append({
#                     'id': len(instructions) + 1,
#                     'type': 'ROTATE',
#                     'angle': math.degrees(abs(angle_diff)),
#                     'direction': 'CW' if angle_diff < 0 else 'CCW',
#                     'target_theta': target_theta,
#                     'speed_percent': 40
#                 })
#                 current_theta = target_theta
            
#             # Añadir movimiento hacia adelante
#             instructions.append({
#                 'id': len(instructions) + 1,
#                 'type': 'FORWARD',
#                 'distance': distance,
#                 'target_x': float(next_point[0]),
#                 'target_y': float(next_point[1]),
#                 'speed_percent': 50
#             })
        
#         return instructions

#     def save_instructions(self, instructions, path_points, filename):
#         """Guarda instrucciones en archivo JSON"""
#         try:
#             data = {
#                 'metadata': {
#                     'timestamp': datetime.now().isoformat(),
#                     'total_points': len(path_points),
#                     'total_distance': self.calculate_path_length(),
#                     'waypoint_count': len(self.waypoints),
#                     'instruction_count': len(instructions)
#                 },
#                 'instructions': instructions,
#                 'path_points': path_points
#             }
            
#             with open(filename, 'w') as f:
#                 json.dump(data, f, indent=2)
                
#             self.get_logger().info(f"💾 Instrucciones guardadas: {filename}")
#             return True
            
#         except Exception as e:
#             self.get_logger().error(f"Error guardando instrucciones: {e}")
#             return False

# def main(args=None):
#     rclpy.init(args=args)
    
#     planner = SimpleRRTPlanner()
    
#     try:
#         # Mantener el nodo activo
#         rclpy.spin(planner)
        
#     except KeyboardInterrupt:
#         planner.get_logger().info("\n🛑 Interrupción recibida. Generando ruta y mapa...")
        
#         # Generar ruta RRT
#         planner.generate_rrt_path()
        
#         # Crear y guardar mapa
#         planner.create_map()
        
#         # Mostrar resumen
#         planner.print_summary()
        
#         planner.get_logger().info("✅ Proceso completado. Mapas guardados en /home/orangepi/map_test")
        
#     finally:
#         planner.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Nodo RRT Planner Simplificado - VERSIÓN CORREGIDA Y SIMPLIFICADA
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import random
import matplotlib.pyplot as plt
from datetime import datetime
import json
import os

class SimpleRRTPlanner(Node):
    def __init__(self):
        super().__init__('simple_rrt_planner')
        
        self.get_logger().info("🔄 Inicializando RRT Planner...")
        
        # Parámetros básicos
        self.step_size = 0.40
        self.max_iterations = 300
        self.goal_bias = 0.70
        
        # Almacenamiento
        self.waypoints = []
        self.planned_path = []
        
        # Directorio de salida
        self.output_dir = "/home/orangepi/map_test"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Suscriptor
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray,
            '/waypoints',
            self.waypoint_callback,
            10
        )
        
        self.get_logger().info("✅ RRT Planner listo. Esperando waypoints...")
        self.get_logger().info("   Presiona Ctrl+C para generar ruta y mapa")
    
    def waypoint_callback(self, msg):
        """Recibe waypoints del nodo de monitoreo"""
        if len(msg.data) >= 3:
            x, y, theta = msg.data[0], msg.data[1], msg.data[2]
            
            # Señal especial de limpieza
            if x == -1.0 and y == -1.0 and theta == -1.0:
                self.waypoints = []
                self.get_logger().info("🗑️ Waypoints limpiados")
                return
            
            self.waypoints.append((x, y, theta))
            self.get_logger().info(f"📍 Waypoint {len(self.waypoints)}: ({x:.3f}, {y:.3f})")
    
    def generate_rrt_path(self):
        """Genera ruta RRT que conecta waypoints en orden"""
        if len(self.waypoints) < 2:
            self.get_logger().warn("⚠️ Se necesitan al menos 2 waypoints")
            return []
        
        self.get_logger().info(f"🎯 Generando ruta RRT para {len(self.waypoints)} waypoints...")
        
        complete_path = []
        
        # Conectar waypoints en orden
        for i in range(len(self.waypoints) - 1):
            start = (self.waypoints[i][0], self.waypoints[i][1])  # Solo (x, y)
            goal = (self.waypoints[i + 1][0], self.waypoints[i + 1][1])  # Solo (x, y)
            
            segment = self.rrt_connect(start, goal)
            
            if complete_path:
                # Evitar duplicar el primer punto
                complete_path.extend(segment[1:])
            else:
                complete_path.extend(segment)
            
            self.get_logger().info(f"  Segmento {i+1}: {len(segment)} puntos")
        
        self.planned_path = complete_path
        return complete_path
    
    def rrt_connect(self, start, goal):
        """RRT simple para conectar dos puntos"""
        nodes = [start]
        parents = [-1]
        
        for iteration in range(self.max_iterations):
            # Punto aleatorio con bias hacia el goal
            if random.random() < self.goal_bias:
                rand_point = goal
            else:
                # Punto aleatorio en área razonable
                rand_point = (
                    random.uniform(min(start[0], goal[0]) - 2, max(start[0], goal[0]) + 2),
                    random.uniform(min(start[1], goal[1]) - 2, max(start[1], goal[1]) + 2)
                )
            
            # Encontrar nodo más cercano
            nearest_idx = self.find_nearest(nodes, rand_point)
            nearest_node = nodes[nearest_idx]
            
            # Extender hacia el punto aleatorio
            new_node = self.steer(nearest_node, rand_point)
            
            # Añadir al árbol
            nodes.append(new_node)
            parents.append(nearest_idx)
            
            # Verificar si alcanzamos el goal
            if self.distance(new_node, goal) < self.step_size:
                # Reconstruir ruta
                path = [goal]
                current_idx = len(nodes) - 1
                
                while current_idx != -1:
                    path.append(nodes[current_idx])
                    current_idx = parents[current_idx]
                
                path.reverse()
                return path
        
        # Si no se encontró ruta, devolver línea recta
        return [start, goal]
    
    def steer(self, from_node, to_node):
        """Mueve desde from_node hacia to_node con paso máximo"""
        dist = self.distance(from_node, to_node)
        if dist <= self.step_size:
            return to_node
        
        ratio = self.step_size / dist
        new_x = from_node[0] + ratio * (to_node[0] - from_node[0])
        new_y = from_node[1] + ratio * (to_node[1] - from_node[1])
        
        return (new_x, new_y)
    
    def find_nearest(self, nodes, point):
        """Encuentra el nodo más cercano"""
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, node in enumerate(nodes):
            dist = self.distance(node, point)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx
    
    def distance(self, p1, p2):
        """Distancia euclidiana"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def create_simple_instructions(self, path):
        """Crea instrucciones simples para el path follower"""
        if len(path) < 2:
            return []
        
        instructions = []
        current_angle = 0.0
        
        for i in range(len(path) - 1):
            current_point = path[i]
            next_point = path[i + 1]
            
            # Calcular ángulo necesario
            dx = next_point[0] - current_point[0]
            dy = next_point[1] - current_point[1]
            target_angle = math.atan2(dy, dx)
            
            # Calcular diferencia de ángulo
            angle_diff = target_angle - current_angle
            angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi  # Normalizar
            
            # Si hay rotación necesaria (> 5 grados)
            if abs(math.degrees(angle_diff)) > 5:
                if angle_diff > 0:
                    instructions.append({
                        'type': 'ROTATE',
                        'direction': 'CCW',
                        'angle': abs(angle_diff),
                        'angle_deg': math.degrees(abs(angle_diff)),
                        'speed': 40
                    })
                else:
                    instructions.append({
                        'type': 'ROTATE',
                        'direction': 'CW',
                        'angle': abs(angle_diff),
                        'angle_deg': math.degrees(abs(angle_diff)),
                        'speed': 40
                    })
                current_angle = target_angle
            
            # Añadir movimiento hacia adelante
            distance = self.distance(current_point, next_point)
            if distance > 0.01:  # Ignorar movimientos muy pequeños
                instructions.append({
                    'type': 'FORWARD',
                    'distance': distance,
                    'speed': 50,
                    'from': [float(current_point[0]), float(current_point[1])],
                    'to': [float(next_point[0]), float(next_point[1])]
                })
        
        return instructions
    
    def create_map_simple(self):
        """Crea y guarda mapa SIMPLE - versión corregida"""
        if not self.waypoints:
            self.get_logger().warn("⚠️ No hay waypoints para crear mapa")
            return
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Crear figura simple
            plt.figure(figsize=(10, 8))
            
            # Dibujar waypoints
            waypoints_x = [wp[0] for wp in self.waypoints]
            waypoints_y = [wp[1] for wp in self.waypoints]
            plt.scatter(waypoints_x, waypoints_y, c='blue', s=200, marker='o', label='Waypoints', zorder=5)
            
            # Numerar waypoints
            for i, (x, y, _) in enumerate(self.waypoints):
                plt.text(x, y + 0.1, f'W{i+1}', fontsize=12, ha='center', fontweight='bold')
            
            # Dibujar ruta planeada
            if self.planned_path and len(self.planned_path) > 1:
                path_x = [p[0] for p in self.planned_path]
                path_y = [p[1] for p in self.planned_path]
                plt.plot(path_x, path_y, 'g-', linewidth=3, label='Ruta RRT', zorder=3)
                
                # Marcar inicio y fin
                plt.scatter(path_x[0], path_y[0], c='orange', s=300, marker='*', label='Inicio', zorder=6)
                plt.scatter(path_x[-1], path_y[-1], c='red', s=300, marker='*', label='Fin', zorder=6)
            
            # Configurar gráfico
            plt.xlabel('X (metros)')
            plt.ylabel('Y (metros)')
            plt.title(f'Ruta RRT - {timestamp}')
            plt.grid(True, alpha=0.3)
            plt.legend()
            plt.axis('equal')
            
            # Añadir información
            info_text = f'Waypoints: {len(self.waypoints)}\n'
            if self.planned_path:
                path_length = sum(self.distance(self.planned_path[i], self.planned_path[i+1]) 
                                for i in range(len(self.planned_path)-1))
                info_text += f'Longitud ruta: {path_length:.2f}m\n'
                info_text += f'Puntos ruta: {len(self.planned_path)}'
            
            plt.figtext(0.02, 0.98, info_text, fontsize=10, verticalalignment='top',
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            # Guardar como PNG
            png_filename = f"{self.output_dir}/ruta_rrt_{timestamp}.png"
            plt.tight_layout()
            plt.savefig(png_filename, dpi=150, bbox_inches='tight')
            plt.close()
            
            self.get_logger().info(f"🗺️  Mapa PNG guardado: {png_filename}")
            
            # Guardar archivo JSON simple
            json_filename = f"{self.output_dir}/ruta_simple_{timestamp}.json"
            self.save_simple_json(json_filename)
            
            # También guardar copia fija para pruebas
            fixed_json = f"{self.output_dir}/ruta_grabada.json"
            self.save_simple_json(fixed_json)
            
            self.get_logger().info(f"💾 Archivo JSON guardado: {json_filename}")
            self.get_logger().info(f"💾 Copia fija: {fixed_json}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error creando mapa: {str(e)}")
    
    def save_simple_json(self, filename):
        """Guarda ruta en formato JSON simple"""
        try:
            # Generar instrucciones
            instructions = self.create_simple_instructions(self.planned_path) if self.planned_path else []
            
            data = {
                'metadata': {
                    'timestamp': datetime.now().isoformat(),
                    'waypoints': len(self.waypoints),
                    'route_points': len(self.planned_path) if self.planned_path else 0,
                    'instructions': len(instructions)
                },
                'waypoints': [
                    {
                        'id': i+1,
                        'x': float(wp[0]),
                        'y': float(wp[1]),
                        'theta': float(wp[2])
                    }
                    for i, wp in enumerate(self.waypoints)
                ],
                'route': [
                    [float(p[0]), float(p[1])] 
                    for p in self.planned_path
                ] if self.planned_path else [],
                'instructions': instructions
            }
            
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error guardando JSON: {e}")
            return False
    
    def print_summary(self):
        """Imprime resumen"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("📊 RESUMEN DE PLANIFICACIÓN")
        self.get_logger().info("="*50)
        
        self.get_logger().info(f"Waypoints: {len(self.waypoints)}")
        if self.waypoints:
            for i, (x, y, theta) in enumerate(self.waypoints):
                self.get_logger().info(f"  W{i+1}: ({x:.3f}, {y:.3f}) - {math.degrees(theta):.1f}°")
        
        if self.planned_path:
            path_length = sum(self.distance(self.planned_path[i], self.planned_path[i+1]) 
                            for i in range(len(self.planned_path)-1))
            self.get_logger().info(f"Ruta: {len(self.planned_path)} puntos")
            self.get_logger().info(f"Longitud: {path_length:.2f}m")
        
        self.get_logger().info("="*50)
        self.get_logger().info(f"Archivos guardados en: {self.output_dir}")
        self.get_logger().info("="*50)

def main(args=None):
    rclpy.init(args=args)
    
    planner = SimpleRRTPlanner()
    
    try:
        rclpy.spin(planner)
        
    except KeyboardInterrupt:
        planner.get_logger().info("\n🛑 Generando ruta y mapa...")
        
        try:
            # Generar ruta
            planner.generate_rrt_path()
            
            # Crear mapa e instrucciones
            planner.create_map_simple()
            
            # Mostrar resumen
            planner.print_summary()
            
            planner.get_logger().info("✅ Proceso completado exitosamente")
            
        except Exception as e:
            planner.get_logger().error(f"❌ Error durante el cierre: {e}")
        
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()