# #!/usr/bin/env python3
# """
# Nodo RRT Planner - VERSIÓN OPTIMIZADA
# Genera rutas bajo demanda mediante comandos ROS2
# NUEVO: Optimiza instrucciones combinando movimientos rectos
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, String
# import numpy as np
# import math
# import random
# import matplotlib.pyplot as plt
# from datetime import datetime
# import json
# import os
# import subprocess

# class SimpleRRTPlanner(Node):
#     def __init__(self):
#         super().__init__('simple_rrt_planner')
        
#         self.get_logger().info("🔄 Inicializando RRT Planner Optimizado...")
        
#         # Parámetros básicos RRT
#         self.step_size = 0.40
#         self.max_iterations = 300
#         self.goal_bias = 0.80
        
#         # Almacenamiento
#         self.waypoints = []
#         self.planned_path = []
        
#         # Directorio de salida
#         self.output_dir = "/home/orangepi/map_test"
#         os.makedirs(self.output_dir, exist_ok=True)
        
#         # Configuración servidor (para envío opcional de archivos)
#         self.server_user = "cesaramj"
#         self.server_ip = "192.168.0.200"
#         self.server_path = "/home/cesaramj/"
        
#         # SUSCRIPTOR: Recibe waypoints
#         self.waypoint_sub = self.create_subscription(
#             Float32MultiArray,
#             '/waypoints',
#             self.waypoint_callback,
#             10
#         )
        
#         # SUSCRIPTOR: Recibe comandos de control
#         self.control_sub = self.create_subscription(
#             String,
#             '/rrt_control',
#             self.control_callback,
#             10
#         )
        
#         # PUBLICADOR: Notifica cuando la ruta está lista
#         self.status_pub = self.create_publisher(
#             String,
#             '/rrt_status',
#             10
#         )
        
#         self.get_logger().info("✅ RRT Planner listo")
#         self.get_logger().info("   📍 Esperando waypoints en /waypoints")
#         self.get_logger().info("   🎮 Esperando comandos en /rrt_control")
#         self.get_logger().info("   Comandos disponibles:")
#         self.get_logger().info("     - GENERATE_PATH: Genera ruta RRT")
#         self.get_logger().info("     - CLEAR_WAYPOINTS: Limpia waypoints")
#         self.get_logger().info("     - STATUS: Muestra estado actual")
    
#     def waypoint_callback(self, msg):
#         """Recibe waypoints del nodo de monitoreo"""
#         if len(msg.data) >= 3:
#             x, y, theta = msg.data[0], msg.data[1], msg.data[2]
            
#             # Señal especial de limpieza
#             if x == -1.0 and y == -1.0 and theta == -1.0:
#                 self.waypoints = []
#                 self.get_logger().info("🗑️ Waypoints limpiados")
#                 return
            
#             self.waypoints.append((x, y, theta))
#             self.get_logger().info(
#                 f"📍 Waypoint {len(self.waypoints)}: "
#                 f"({x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°)"
#             )
    
#     def control_callback(self, msg):
#         """
#         Maneja comandos de control para el planner
        
#         Comandos soportados:
#         - GENERATE_PATH: Genera ruta RRT con los waypoints actuales
#         - CLEAR_WAYPOINTS: Limpia todos los waypoints
#         - STATUS: Publica estado actual
#         """
#         command = msg.data.strip().upper()
        
#         self.get_logger().info(f"🎮 Comando recibido: {command}")
        
#         if command == "GENERATE_PATH":
#             self.handle_generate_path()
        
#         elif command == "CLEAR_WAYPOINTS":
#             self.waypoints = []
#             self.planned_path = []
#             self.get_logger().info("🗑️ Waypoints y ruta limpiados")
#             self.publish_status("CLEARED")
        
#         elif command == "STATUS":
#             self.print_status()
#             self.publish_status("IDLE")
        
#         else:
#             self.get_logger().warning(f"⚠️ Comando desconocido: {command}")
    
#     def handle_generate_path(self):
#         """
#         Maneja la generación completa de ruta RRT.
#         Este es el método principal que se ejecuta cuando se recibe GENERATE_PATH.
#         """
#         self.get_logger().info("\n" + "="*60)
#         self.get_logger().info("🚀 INICIANDO GENERACIÓN DE RUTA RRT")
#         self.get_logger().info("="*60)
        
#         try:
#             # 1. Validar waypoints
#             if len(self.waypoints) < 2:
#                 self.get_logger().error("❌ Se necesitan al menos 2 waypoints")
#                 self.publish_status("ERROR: Waypoints insuficientes")
#                 return
            
#             self.get_logger().info(f"✅ Waypoints válidos: {len(self.waypoints)}")
            
#             # 2. Generar ruta RRT
#             self.get_logger().info("🗺️ Generando ruta RRT...")
#             self.publish_status("GENERATING")
            
#             success = self.generate_rrt_path()
            
#             if not success or not self.planned_path:
#                 self.get_logger().error("❌ Falló generación de ruta")
#                 self.publish_status("ERROR: Generación fallida")
#                 return
            
#             self.get_logger().info(f"✅ Ruta generada: {len(self.planned_path)} puntos")
            
#             # 3. Crear mapa visual
#             self.get_logger().info("🎨 Creando mapa visual...")
#             map_file = self.create_map_simple()
            
#             if map_file:
#                 self.get_logger().info(f"✅ Mapa guardado: {map_file}")
            
#             # 4. Guardar archivos JSON
#             self.get_logger().info("💾 Guardando archivos JSON...")
#             timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
#             # Archivo timestamped
#             json_file = f"{self.output_dir}/ruta_simple_{timestamp}.json"
#             self.save_simple_json(json_file)
            
#             # Archivo fijo para path follower
#             fixed_json = f"{self.output_dir}/ruta_grabada.json"
#             self.save_simple_json(fixed_json)
            
#             self.get_logger().info(f"✅ JSON guardado: {json_file}")
#             self.get_logger().info(f"✅ Copia fija: {fixed_json}")

#             # 5. Mostrar resumen
#             self.print_summary()
            
#             # 6. Publicar éxito
#             self.publish_status("COMPLETED")
            
#             self.get_logger().info("="*60)
#             self.get_logger().info("✅ GENERACIÓN DE RUTA COMPLETADA EXITOSAMENTE")
#             self.get_logger().info("="*60 + "\n")
            
#         except Exception as e:
#             self.get_logger().error(f"❌ Error en generación de ruta: {e}")
#             self.publish_status(f"ERROR: {str(e)}")
    
#     def generate_rrt_path(self):
#         """Genera ruta RRT que conecta waypoints en orden"""
#         if len(self.waypoints) < 2:
#             return False
        
#         self.get_logger().info(
#             f"🎯 Conectando {len(self.waypoints)} waypoints con RRT..."
#         )
        
#         complete_path = []
        
#         # Conectar waypoints consecutivos
#         for i in range(len(self.waypoints) - 1):
#             start = (self.waypoints[i][0], self.waypoints[i][1])
#             goal = (self.waypoints[i + 1][0], self.waypoints[i + 1][1])
            
#             self.get_logger().info(
#                 f"  Segmento {i+1}/{len(self.waypoints)-1}: "
#                 f"({start[0]:.2f}, {start[1]:.2f}) → "
#                 f"({goal[0]:.2f}, {goal[1]:.2f})"
#             )
            
#             segment = self.rrt_connect(start, goal)
            
#             if complete_path:
#                 complete_path.extend(segment[1:])  # Evitar duplicar puntos
#             else:
#                 complete_path.extend(segment)
            
#             self.get_logger().info(f"    ✓ {len(segment)} puntos generados")
        
#         self.planned_path = complete_path
#         return True
    
#     def rrt_connect(self, start, goal):
#         """Algoritmo RRT para conectar dos puntos"""
#         nodes = [start]
#         parents = [-1]
        
#         for iteration in range(self.max_iterations):
#             # Punto aleatorio con bias hacia goal
#             if random.random() < self.goal_bias:
#                 rand_point = goal
#             else:
#                 rand_point = (
#                     random.uniform(
#                         min(start[0], goal[0]) - 2, 
#                         max(start[0], goal[0]) + 2
#                     ),
#                     random.uniform(
#                         min(start[1], goal[1]) - 2, 
#                         max(start[1], goal[1]) + 2
#                     )
#                 )
            
#             # Encontrar nodo más cercano
#             nearest_idx = self.find_nearest(nodes, rand_point)
#             nearest_node = nodes[nearest_idx]
            
#             # Extender hacia punto aleatorio
#             new_node = self.steer(nearest_node, rand_point)
            
#             # Añadir al árbol
#             nodes.append(new_node)
#             parents.append(nearest_idx)
            
#             # Verificar si alcanzamos goal
#             if self.distance(new_node, goal) < self.step_size:
#                 # Reconstruir ruta
#                 path = [goal]
#                 current_idx = len(nodes) - 1
                
#                 while current_idx != -1:
#                     path.append(nodes[current_idx])
#                     current_idx = parents[current_idx]
                
#                 path.reverse()
#                 return path
        
#         # Si no se encontró ruta, línea recta
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
    
#     def create_simple_instructions(self, path):
#         """Crea instrucciones de movimiento para el path follower"""
#         if len(path) < 2:
#             return []
        
#         instructions = []
#         current_angle = 0.0
        
#         for i in range(len(path) - 1):
#             current_point = path[i]
#             next_point = path[i + 1]
            
#             # Calcular ángulo necesario
#             dx = next_point[0] - current_point[0]
#             dy = next_point[1] - current_point[1]
#             target_angle = math.atan2(dy, dx)
            
#             # Diferencia de ángulo
#             angle_diff = target_angle - current_angle
#             angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi
            
#             # Rotación si es necesaria (> 5°)
#             if abs(math.degrees(angle_diff)) > 5:
#                 instructions.append({
#                     'type': 'ROTATE',
#                     'direction': 'CCW' if angle_diff > 0 else 'CW',
#                     'angle': abs(angle_diff),
#                     'angle_deg': math.degrees(abs(angle_diff)),
#                     'speed': 50
#                 })
#                 current_angle = target_angle
            
#             # Movimiento hacia adelante
#             distance = self.distance(current_point, next_point)
#             if distance > 0.01:
#                 instructions.append({
#                     'type': 'FORWARD',
#                     'distance': distance,
#                     'speed': 50,
#                     'from': [float(current_point[0]), float(current_point[1])],
#                     'to': [float(next_point[0]), float(next_point[1])]
#                 })
        
#         # 🆕 OPTIMIZACIÓN: Combinar FORWARDs consecutivos
#         optimized = self.optimize_instructions(instructions)
        
#         return optimized
    
#     def optimize_instructions(self, instructions):
#         """
#         🆕 NUEVA FUNCIÓN: Optimiza instrucciones combinando FORWARDs consecutivos
        
#         Combina múltiples instrucciones FORWARD seguidas (sin ROTATE entre ellas)
#         en una sola instrucción FORWARD con la distancia total.
        
#         Esto elimina los micro-pasos cuando el robot va en línea recta.
#         """
#         if not instructions:
#             return []
        
#         optimized = []
#         i = 0
        
#         while i < len(instructions):
#             current = instructions[i]
            
#             # Si es ROTATE, añadirla directamente
#             if current['type'] == 'ROTATE':
#                 optimized.append(current)
#                 i += 1
#                 continue
            
#             # Si es FORWARD, buscar cuántos FORWARDs consecutivos hay
#             if current['type'] == 'FORWARD':
#                 # Acumular distancia y puntos
#                 total_distance = current['distance']
#                 start_point = current['from']
#                 end_point = current['to']
#                 forward_count = 1
                
#                 # Mirar hacia adelante para encontrar más FORWARDs consecutivos
#                 j = i + 1
#                 while j < len(instructions) and instructions[j]['type'] == 'FORWARD':
#                     total_distance += instructions[j]['distance']
#                     end_point = instructions[j]['to']
#                     forward_count += 1
#                     j += 1
                
#                 # Crear instrucción combinada
#                 combined = {
#                     'type': 'FORWARD',
#                     'distance': total_distance,
#                     'speed': current['speed'],
#                     'from': start_point,
#                     'to': end_point
#                 }
                
#                 # Si se combinaron varios, añadir info adicional
#                 if forward_count > 1:
#                     combined['combined_steps'] = forward_count
#                     combined['note'] = f'Combinado de {forward_count} micro-pasos'
                
#                 optimized.append(combined)
                
#                 # Saltar todos los FORWARDs que acabamos de combinar
#                 i = j
#             else:
#                 # Tipo desconocido, añadir tal cual
#                 optimized.append(current)
#                 i += 1
        
#         # Log de optimización
#         original_count = len(instructions)
#         optimized_count = len(optimized)
#         if original_count != optimized_count:
#             self.get_logger().info(
#                 f"🔧 Optimización: {original_count} instrucciones → "
#                 f"{optimized_count} instrucciones "
#                 f"(reducción: {original_count - optimized_count})"
#             )
        
#         return optimized
    
#     def create_map_simple(self):
#         """Crea y guarda mapa visual PNG"""
#         if not self.waypoints:
#             return None
        
#         try:
#             timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
#             plt.figure(figsize=(12, 10))
            
#             # Waypoints
#             waypoints_x = [wp[0] for wp in self.waypoints]
#             waypoints_y = [wp[1] for wp in self.waypoints]
#             plt.scatter(
#                 waypoints_x, waypoints_y, 
#                 c='blue', s=250, marker='o', 
#                 label='Waypoints', zorder=5, 
#                 edgecolors='black', linewidth=2
#             )
            
#             # Numerar waypoints
#             for i, (x, y, _) in enumerate(self.waypoints):
#                 plt.text(
#                     x, y + 0.15, f'W{i+1}', 
#                     fontsize=13, ha='center', 
#                     fontweight='bold',
#                     bbox=dict(
#                         boxstyle='round,pad=0.4', 
#                         facecolor='yellow', 
#                         alpha=0.8
#                     )
#                 )
            
#             # Ruta RRT
#             if self.planned_path and len(self.planned_path) > 1:
#                 path_x = [p[0] for p in self.planned_path]
#                 path_y = [p[1] for p in self.planned_path]
#                 plt.plot(
#                     path_x, path_y, 
#                     'g-', linewidth=3, 
#                     label='Ruta RRT', 
#                     zorder=3, alpha=0.7
#                 )
                
#                 # Inicio y fin
#                 plt.scatter(
#                     path_x[0], path_y[0], 
#                     c='orange', s=350, marker='*', 
#                     label='Inicio', zorder=6,
#                     edgecolors='black', linewidth=2
#                 )
#                 plt.scatter(
#                     path_x[-1], path_y[-1], 
#                     c='red', s=350, marker='*', 
#                     label='Fin', zorder=6,
#                     edgecolors='black', linewidth=2
#                 )
            
#             # Configuración
#             plt.xlabel('X (metros)', fontsize=13)
#             plt.ylabel('Y (metros)', fontsize=13)
#             plt.title(
#                 f'Ruta de Navegación RRT - {timestamp}', 
#                 fontsize=15, fontweight='bold'
#             )
#             plt.grid(True, alpha=0.3, linestyle='--')
#             plt.legend(fontsize=11, loc='best')
#             plt.axis('equal')
            
#             # Información estadística
#             if self.planned_path:
#                 path_length = sum(
#                     self.distance(self.planned_path[i], self.planned_path[i+1]) 
#                     for i in range(len(self.planned_path)-1)
#                 )
#                 info_text = (
#                     f'Waypoints: {len(self.waypoints)}\n'
#                     f'Puntos ruta: {len(self.planned_path)}\n'
#                     f'Longitud: {path_length:.2f}m'
#                 )
#                 plt.figtext(
#                     0.02, 0.98, info_text, 
#                     fontsize=11, verticalalignment='top',
#                     bbox=dict(
#                         boxstyle='round', 
#                         facecolor='wheat', 
#                         alpha=0.9
#                     )
#                 )
            
#             # Guardar
#             png_file = f"{self.output_dir}/ruta_rrt_{timestamp}.png"
#             plt.tight_layout()
#             plt.savefig(png_file, dpi=150, bbox_inches='tight')
#             plt.close()
            
#             return png_file
            
#         except Exception as e:
#             self.get_logger().error(f"Error creando mapa: {e}")
#             return None
    
#     def save_simple_json(self, filename):
#         """Guarda ruta en formato JSON"""
#         try:
#             instructions = self.create_simple_instructions(self.planned_path)
            
#             # Contar instrucciones por tipo
#             rotate_count = sum(1 for i in instructions if i['type'] == 'ROTATE')
#             forward_count = sum(1 for i in instructions if i['type'] == 'FORWARD')
#             combined_count = sum(1 for i in instructions if i.get('combined_steps', 0) > 1)
            
#             data = {
#                 'metadata': {
#                     'timestamp': datetime.now().isoformat(),
#                     'generator': 'RRT Planner',
#                     'waypoints_count': len(self.waypoints),
#                     'route_points': len(self.planned_path),
#                     'instructions_count': len(instructions),
#                     'rotate_instructions': rotate_count,
#                     'forward_instructions': forward_count,
#                     'optimized_forwards': combined_count,
#                     'optimization_enabled': True
#                 },
#                 'waypoints': [
#                     {
#                         'id': i+1,
#                         'x': float(wp[0]),
#                         'y': float(wp[1]),
#                         'theta': float(wp[2]),
#                         'theta_deg': float(math.degrees(wp[2]))
#                     }
#                     for i, wp in enumerate(self.waypoints)
#                 ],
#                 'route': [
#                     [float(p[0]), float(p[1])] 
#                     for p in self.planned_path
#                 ],
#                 'instructions': instructions
#             }
            
#             with open(filename, 'w') as f:
#                 json.dump(data, f, indent=2)
            
#             return True
            
#         except Exception as e:
#             self.get_logger().error(f"Error guardando JSON: {e}")
#             return False
    
#     def publish_status(self, status):
#         """Publica estado del planner"""
#         msg = String()
#         msg.data = status
#         self.status_pub.publish(msg)
    
#     def print_status(self):
#         """Imprime estado actual"""
#         self.get_logger().info("\n" + "="*50)
#         self.get_logger().info("📊 ESTADO ACTUAL DEL PLANNER")
#         self.get_logger().info("="*50)
#         self.get_logger().info(f"Waypoints: {len(self.waypoints)}")
#         self.get_logger().info(f"Ruta generada: {'Sí' if self.planned_path else 'No'}")
#         if self.planned_path:
#             self.get_logger().info(f"Puntos en ruta: {len(self.planned_path)}")
#         self.get_logger().info("="*50)
    
#     def print_summary(self):
#         """Imprime resumen detallado"""
#         self.get_logger().info("\n" + "="*60)
#         self.get_logger().info("📊 RESUMEN DE PLANIFICACIÓN")
#         self.get_logger().info("="*60)
        
#         if self.waypoints:
#             self.get_logger().info(f"\n📍 Waypoints ({len(self.waypoints)}):")
#             for i, (x, y, theta) in enumerate(self.waypoints):
#                 self.get_logger().info(
#                     f"  W{i+1}: ({x:.3f}, {y:.3f}) - {math.degrees(theta):.1f}°"
#                 )
        
#         if self.planned_path:
#             path_length = sum(
#                 self.distance(self.planned_path[i], self.planned_path[i+1]) 
#                 for i in range(len(self.planned_path)-1)
#             )
            
#             # Generar instrucciones para mostrar estadísticas
#             instructions = self.create_simple_instructions(self.planned_path)
#             rotate_count = sum(1 for i in instructions if i['type'] == 'ROTATE')
#             forward_count = sum(1 for i in instructions if i['type'] == 'FORWARD')
#             combined_count = sum(1 for i in instructions if i.get('combined_steps', 0) > 1)
            
#             self.get_logger().info(f"\n🗺️ Ruta RRT:")
#             self.get_logger().info(f"  Puntos: {len(self.planned_path)}")
#             self.get_logger().info(f"  Longitud: {path_length:.2f}m")
#             self.get_logger().info(
#                 f"  Inicio: ({self.planned_path[0][0]:.3f}, "
#                 f"{self.planned_path[0][1]:.3f})"
#             )
#             self.get_logger().info(
#                 f"  Fin: ({self.planned_path[-1][0]:.3f}, "
#                 f"{self.planned_path[-1][1]:.3f})"
#             )
            
#             self.get_logger().info(f"\n🎯 Instrucciones:")
#             self.get_logger().info(f"  Total: {len(instructions)}")
#             self.get_logger().info(f"  Rotaciones: {rotate_count}")
#             self.get_logger().info(f"  Movimientos: {forward_count}")
#             if combined_count > 0:
#                 self.get_logger().info(f"  🔧 Optimizados: {combined_count} (tramos rectos combinados)")
        
#         self.get_logger().info(f"\n💾 Archivos: {self.output_dir}")
#         self.get_logger().info("="*60 + "\n")



# def main(args=None):
#     rclpy.init(args=args)
    
#     planner = SimpleRRTPlanner()
    
#     try:
#         rclpy.spin(planner)
        
#     except KeyboardInterrupt:
#         planner.get_logger().info("\n🛑 Deteniendo planner...")
        
#     finally:
#         planner.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Nodo RRT Planner - VERSIÓN OPTIMIZADA CORRECTA
Genera rutas que PASAN POR TODOS LOS WAYPOINTS
Solo optimiza los segmentos entre waypoints para eliminar zigzags
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import math
import random
import matplotlib.pyplot as plt
from datetime import datetime
import json
import os

class OptimizedRRTPlanner(Node):
    def __init__(self):
        super().__init__('optimized_rrt_planner')
        
        self.get_logger().info("🚀 Inicializando RRT Planner Optimizado...")
        
        # Parámetros RRT optimizados para rutas más rectas
        self.step_size = 0.6  # Pasos más grandes
        self.max_iterations = 500
        self.goal_bias = 0.95  # Muy alto bias hacia objetivo
        
        # Parámetros de optimización de segmentos
        self.douglas_peucker_epsilon = 0.05  # Tolerancia para simplificación
        self.angle_threshold_deg = 8.0  # Umbral para considerar giro significativo
        
        # Almacenamiento
        self.waypoints = []
        self.planned_path = []
        self.loaded_route_data = None  # Para RELOAD
        
        # Directorio de salida
        self.output_dir = "/home/orangepi/map_test"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Configuración servidor
        self.server_user = "cesaramj"
        self.server_ip = "192.168.0.200"
        self.server_path = "/home/cesaramj/"
        
        # SUSCRIPTOR: Recibe waypoints
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray,
            '/waypoints',
            self.waypoint_callback,
            10
        )
        
        # SUSCRIPTOR: Recibe comandos de control
        self.control_sub = self.create_subscription(
            String,
            '/rrt_control',
            self.control_callback,
            10
        )
        
        # PUBLICADOR: Notifica cuando la ruta está lista
        self.status_pub = self.create_publisher(
            String,
            '/rrt_status',
            10
        )
        
        self.get_logger().info("✅ RRT Planner Optimizado listo")
        self.get_logger().info("   📍 Esperando waypoints en /waypoints")
        self.get_logger().info("   🎮 Comandos disponibles:")
        self.get_logger().info("      - GENERATE_PATH: Genera ruta RRT")
        self.get_logger().info("      - RELOAD: Recarga última ruta guardada")
        self.get_logger().info("      - CLEAR_WAYPOINTS: Limpia waypoints")
        self.get_logger().info("      - STATUS: Muestra estado")
    
    def waypoint_callback(self, msg):
        """Recibe waypoints del nodo de monitoreo"""
        if len(msg.data) >= 3:
            x, y, theta = msg.data[0], msg.data[1], msg.data[2]
            
            if x == -1.0 and y == -1.0 and theta == -1.0:
                self.waypoints = []
                self.get_logger().info("🗑️ Waypoints limpiados")
                return
            
            self.waypoints.append((x, y, theta))
            self.get_logger().info(
                f"📍 Waypoint {len(self.waypoints)}: "
                f"({x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°)"
            )
    
    def control_callback(self, msg):
        """Maneja comandos de control para el planner"""
        command = msg.data.strip().upper()
        self.get_logger().info(f"🎮 Comando recibido: {command}")
        
        if command == "GENERATE_PATH":
            self.handle_generate_path()
        
        elif command == "RELOAD":
            self.handle_reload_route()
        
        elif command == "CLEAR_WAYPOINTS":
            self.waypoints = []
            self.planned_path = []
            self.get_logger().info("🗑️ Waypoints y ruta limpiados")
            self.publish_status("CLEARED")
        
        elif command == "STATUS":
            self.print_status()
            self.publish_status("IDLE")
        
        else:
            self.get_logger().warning(f"⚠️ Comando desconocido: {command}")
    
    def handle_reload_route(self):
        """
        🔄 Recarga la última ruta guardada desde ruta_grabada.json
        """
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🔄 RECARGANDO RUTA DESDE ARCHIVO")
        self.get_logger().info("="*60)
        
        try:
            json_file = f"{self.output_dir}/ruta_grabada.json"
            
            if not os.path.exists(json_file):
                self.get_logger().error(f"❌ Archivo no encontrado: {json_file}")
                self.publish_status("ERROR: Archivo no encontrado")
                return
            
            # Cargar JSON
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            # Reconstruir waypoints
            self.waypoints = []
            for wp in data['waypoints']:
                self.waypoints.append((wp['x'], wp['y'], wp['theta']))
            
            # Reconstruir ruta
            self.planned_path = [tuple(point) for point in data['route']]
            
            # Guardar datos completos
            self.loaded_route_data = data
            
            self.get_logger().info(f"✅ Ruta recargada exitosamente")
            self.get_logger().info(f"   Waypoints: {len(self.waypoints)}")
            self.get_logger().info(f"   Puntos en ruta: {len(self.planned_path)}")
            self.get_logger().info(f"   Instrucciones: {len(data['instructions'])}")
            
            # Mostrar resumen
            self.print_summary()
            
            self.publish_status("RELOADED")
            
            self.get_logger().info("="*60)
            self.get_logger().info("✅ RECARGA COMPLETADA")
            self.get_logger().info("="*60 + "\n")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error recargando ruta: {e}")
            self.publish_status(f"ERROR: {str(e)}")
    
    def handle_generate_path(self):
        """Maneja la generación completa de ruta optimizada"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🚀 GENERANDO RUTA OPTIMIZADA")
        self.get_logger().info("="*60)
        
        try:
            # 1. Validar waypoints
            if len(self.waypoints) < 2:
                self.get_logger().error("❌ Se necesitan al menos 2 waypoints")
                self.publish_status("ERROR: Waypoints insuficientes")
                return
            
            self.get_logger().info(f"✅ Waypoints válidos: {len(self.waypoints)}")
            
            # 2. Generar ruta que pasa por TODOS los waypoints
            self.get_logger().info("🗺️ Generando ruta por todos los waypoints...")
            self.publish_status("GENERATING")
            
            success = self.generate_optimized_path()
            
            if not success or not self.planned_path:
                self.get_logger().error("❌ Falló generación de ruta")
                self.publish_status("ERROR: Generación fallida")
                return
            
            self.get_logger().info(f"✅ Ruta generada: {len(self.planned_path)} puntos")
            
            # 3. Crear mapa visual
            self.get_logger().info("🎨 Creando mapa visual...")
            map_file = self.create_map_visual()
            
            if map_file:
                self.get_logger().info(f"✅ Mapa guardado: {map_file}")
            
            # 4. Guardar archivos JSON
            self.get_logger().info("💾 Guardando archivos JSON...")
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            json_file = f"{self.output_dir}/ruta_optimizada_{timestamp}.json"
            self.save_route_json(json_file)
            
            fixed_json = f"{self.output_dir}/ruta_grabada.json"
            self.save_route_json(fixed_json)
            
            self.get_logger().info(f"✅ JSON guardado: {json_file}")
            self.get_logger().info(f"✅ Copia fija: {fixed_json}")

            # 5. Mostrar resumen
            self.print_summary()
            
            # 6. Publicar éxito
            self.publish_status("COMPLETED")
            
            self.get_logger().info("="*60)
            self.get_logger().info("✅ GENERACIÓN COMPLETADA")
            self.get_logger().info("="*60 + "\n")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
            self.publish_status(f"ERROR: {str(e)}")
    
    def generate_optimized_path(self):
        """
        Genera ruta que PASA POR TODOS LOS WAYPOINTS en orden
        Optimiza solo los segmentos entre waypoints
        """
        if len(self.waypoints) < 2:
            return False
        
        self.get_logger().info(f"🎯 Conectando {len(self.waypoints)} waypoints en orden...")
        
        complete_path = []
        
        # Conectar waypoints consecutivos
        for i in range(len(self.waypoints) - 1):
            start = (self.waypoints[i][0], self.waypoints[i][1])
            goal = (self.waypoints[i + 1][0], self.waypoints[i + 1][1])
            
            self.get_logger().info(
                f"  Segmento {i+1}/{len(self.waypoints)-1}: "
                f"W{i+1} → W{i+2}"
            )
            
            # Generar segmento con RRT
            raw_segment = self.rrt_connect(start, goal)
            
            # 🔧 OPTIMIZAR solo este segmento (sin eliminar waypoints)
            optimized_segment = self.optimize_segment(raw_segment)
            
            self.get_logger().info(
                f"    Puntos: {len(raw_segment)} → {len(optimized_segment)} "
                f"(optimizado)"
            )
            
            # Agregar a ruta completa
            if complete_path:
                # Evitar duplicar el punto de conexión
                complete_path.extend(optimized_segment[1:])
            else:
                complete_path.extend(optimized_segment)
        
        self.planned_path = complete_path
        
        # Verificar que la ruta pasa por todos los waypoints
        self.verify_waypoints_in_path()
        
        return True
    
    def optimize_segment(self, segment):
        """
        Optimiza UN segmento entre dos waypoints
        NO elimina los extremos (waypoints)
        """
        if len(segment) < 3:
            return segment
        
        # Siempre mantener primer y último punto (waypoints)
        start = segment[0]
        end = segment[-1]
        
        # Optimizar puntos intermedios con Douglas-Peucker
        optimized = self.douglas_peucker(segment, self.douglas_peucker_epsilon)
        
        # Asegurar que los extremos se mantienen
        if optimized[0] != start:
            optimized.insert(0, start)
        if optimized[-1] != end:
            optimized.append(end)
        
        return optimized
    
    def verify_waypoints_in_path(self):
        """
        Verifica que todos los waypoints estén en la ruta
        """
        self.get_logger().info("\n🔍 Verificando waypoints en ruta:")
        
        for i, (wx, wy, wtheta) in enumerate(self.waypoints):
            # Buscar waypoint en ruta (con tolerancia de 0.01m)
            found = False
            for px, py in self.planned_path:
                if abs(px - wx) < 0.01 and abs(py - wy) < 0.01:
                    found = True
                    break
            
            if found:
                self.get_logger().info(f"  ✅ W{i+1} encontrado en ruta")
            else:
                self.get_logger().warning(f"  ⚠️ W{i+1} NO encontrado en ruta!")
    
    def douglas_peucker(self, points, epsilon):
        """
        Simplificación Douglas-Peucker
        Elimina puntos que están casi en línea recta
        """
        if len(points) < 3:
            return points
        
        # Encontrar punto con mayor distancia perpendicular
        dmax = 0
        index = 0
        end = len(points) - 1
        
        for i in range(1, end):
            d = self.perpendicular_distance(points[i], points[0], points[end])
            if d > dmax:
                index = i
                dmax = d
        
        # Si la distancia máxima es mayor que epsilon, dividir
        if dmax > epsilon:
            rec1 = self.douglas_peucker(points[:index+1], epsilon)
            rec2 = self.douglas_peucker(points[index:], epsilon)
            result = rec1[:-1] + rec2
        else:
            # Todos los puntos cerca de línea recta
            result = [points[0], points[end]]
        
        return result
    
    def perpendicular_distance(self, point, line_start, line_end):
        """Calcula distancia perpendicular de punto a línea"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            return self.distance(point, line_start)
        
        t = max(0, min(1, ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)))
        
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        
        return math.sqrt((x0 - proj_x)**2 + (y0 - proj_y)**2)
    
    def rrt_connect(self, start, goal):
        """RRT optimizado con alto bias hacia objetivo"""
        nodes = [start]
        parents = [-1]
        
        for iteration in range(self.max_iterations):
            # Alto bias hacia objetivo para rutas más directas
            if random.random() < self.goal_bias:
                rand_point = goal
            else:
                # Exploración limitada cerca de la línea start-goal
                margin = 0.5
                rand_point = (
                    random.uniform(
                        min(start[0], goal[0]) - margin, 
                        max(start[0], goal[0]) + margin
                    ),
                    random.uniform(
                        min(start[1], goal[1]) - margin, 
                        max(start[1], goal[1]) + margin
                    )
                )
            
            nearest_idx = self.find_nearest(nodes, rand_point)
            nearest_node = nodes[nearest_idx]
            new_node = self.steer(nearest_node, rand_point)
            
            nodes.append(new_node)
            parents.append(nearest_idx)
            
            # Si llegamos cerca del objetivo
            if self.distance(new_node, goal) < self.step_size:
                # Reconstruir ruta
                path = [goal]
                current_idx = len(nodes) - 1
                
                while current_idx != -1:
                    path.append(nodes[current_idx])
                    current_idx = parents[current_idx]
                
                path.reverse()
                return path
        
        # Si no se encontró, línea recta
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
        return min(range(len(nodes)), key=lambda i: self.distance(nodes[i], point))
    
    def distance(self, p1, p2):
        """Distancia euclidiana"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def create_simple_instructions(self, path):
        """Crea instrucciones de movimiento optimizadas"""
        if len(path) < 2:
            return []
        
        instructions = []
        current_angle = 0.0
        
        for i in range(len(path) - 1):
            current_point = path[i]
            next_point = path[i + 1]
            
            dx = next_point[0] - current_point[0]
            dy = next_point[1] - current_point[1]
            target_angle = math.atan2(dy, dx)
            
            angle_diff = target_angle - current_angle
            angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi
            
            # Rotación solo si es significativa
            if abs(math.degrees(angle_diff)) > self.angle_threshold_deg:
                instructions.append({
                    'type': 'ROTATE',
                    'direction': 'CCW' if angle_diff > 0 else 'CW',
                    'angle': abs(angle_diff),
                    'angle_deg': math.degrees(abs(angle_diff)),
                    'speed': 50
                })
                current_angle = target_angle
            
            distance = self.distance(current_point, next_point)
            if distance > 0.01:
                instructions.append({
                    'type': 'FORWARD',
                    'distance': distance,
                    'speed': 50,
                    'from': [float(current_point[0]), float(current_point[1])],
                    'to': [float(next_point[0]), float(next_point[1])]
                })
        
        # Combinar FORWARDs consecutivos
        optimized = self.optimize_instructions(instructions)
        
        return optimized
    
    def optimize_instructions(self, instructions):
        """Combina instrucciones FORWARD consecutivas"""
        if not instructions:
            return []
        
        optimized = []
        i = 0
        
        while i < len(instructions):
            current = instructions[i]
            
            if current['type'] == 'ROTATE':
                optimized.append(current)
                i += 1
                continue
            
            if current['type'] == 'FORWARD':
                total_distance = current['distance']
                start_point = current['from']
                end_point = current['to']
                forward_count = 1
                
                j = i + 1
                while j < len(instructions) and instructions[j]['type'] == 'FORWARD':
                    total_distance += instructions[j]['distance']
                    end_point = instructions[j]['to']
                    forward_count += 1
                    j += 1
                
                combined = {
                    'type': 'FORWARD',
                    'distance': total_distance,
                    'speed': current['speed'],
                    'from': start_point,
                    'to': end_point
                }
                
                if forward_count > 1:
                    combined['combined_steps'] = forward_count
                
                optimized.append(combined)
                i = j
            else:
                optimized.append(current)
                i += 1
        
        return optimized
    
    def create_map_visual(self):
        """Crea mapa visual mostrando todos los waypoints"""
        if not self.waypoints:
            return None
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            plt.figure(figsize=(14, 10))
            
            # Waypoints
            waypoints_x = [wp[0] for wp in self.waypoints]
            waypoints_y = [wp[1] for wp in self.waypoints]
            plt.scatter(waypoints_x, waypoints_y, c='blue', s=300, marker='o',
                       label='Waypoints', zorder=5, edgecolors='black', linewidth=2)
            
            # Numerar waypoints
            for i, (x, y, _) in enumerate(self.waypoints):
                plt.text(x, y + 0.15, f'W{i+1}', fontsize=13, ha='center',
                        fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.4', facecolor='yellow', alpha=0.8))
            
            # Ruta optimizada
            if self.planned_path and len(self.planned_path) > 1:
                path_x = [p[0] for p in self.planned_path]
                path_y = [p[1] for p in self.planned_path]
                plt.plot(path_x, path_y, 'g-', linewidth=3,
                        label='Ruta', zorder=3, alpha=0.7)
                
                # Marcar puntos de la ruta
                plt.scatter(path_x, path_y, c='lightgreen', s=50, 
                          marker='o', zorder=4, alpha=0.5)
                
                # Inicio y fin
                plt.scatter(path_x[0], path_y[0], c='orange', s=400, marker='*',
                          label='Inicio', zorder=6, edgecolors='black', linewidth=2)
                plt.scatter(path_x[-1], path_y[-1], c='red', s=400, marker='*',
                          label='Fin', zorder=6, edgecolors='black', linewidth=2)
            
            # Configuración
            plt.xlabel('X (metros)', fontsize=14)
            plt.ylabel('Y (metros)', fontsize=14)
            plt.title(f'Ruta\n{timestamp}', 
                     fontsize=15, fontweight='bold')
            plt.grid(True, alpha=0.3, linestyle='--')
            plt.legend(fontsize=12, loc='best')
            plt.axis('equal')
            
            # Información estadística
            if self.planned_path:
                path_length = sum(
                    self.distance(self.planned_path[i], self.planned_path[i+1]) 
                    for i in range(len(self.planned_path)-1)
                )
                
                instructions = self.create_simple_instructions(self.planned_path)
                rotate_count = sum(1 for i in instructions if i['type'] == 'ROTATE')
                forward_count = sum(1 for i in instructions if i['type'] == 'FORWARD')
                
                info_text = (
                    f'Waypoints: {len(self.waypoints)}\n'
                    f'Puntos ruta: {len(self.planned_path)}\n'
                    f'Longitud: {path_length:.2f}m\n'
                    f'Rotaciones: {rotate_count}\n'
                    f'Movimientos: {forward_count}'
                )
                plt.figtext(0.02, 0.98, info_text, fontsize=11, 
                          verticalalignment='top',
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
            
            # Guardar
            png_file = f"{self.output_dir}/ruta_rtt_{timestamp}.png"
            plt.tight_layout()
            plt.savefig(png_file, dpi=150, bbox_inches='tight')
            plt.close()
            
            return png_file
            
        except Exception as e:
            self.get_logger().error(f"Error creando mapa: {e}")
            return None
    
    def save_route_json(self, filename):
        """Guarda ruta en formato JSON"""
        try:
            instructions = self.create_simple_instructions(self.planned_path)
            
            rotate_count = sum(1 for i in instructions if i['type'] == 'ROTATE')
            forward_count = sum(1 for i in instructions if i['type'] == 'FORWARD')
            combined_count = sum(1 for i in instructions if i.get('combined_steps', 0) > 1)
            
            data = {
                'metadata': {
                    'timestamp': datetime.now().isoformat(),
                    'generator': 'RRT Planner Optimizado',
                    'waypoints_count': len(self.waypoints),
                    'route_points': len(self.planned_path),
                    'instructions_count': len(instructions),
                    'rotate_instructions': rotate_count,
                    'forward_instructions': forward_count,
                    'optimized_forwards': combined_count,
                    'optimization_enabled': True
                },
                'waypoints': [
                    {
                        'id': i+1,
                        'x': float(wp[0]),
                        'y': float(wp[1]),
                        'theta': float(wp[2]),
                        'theta_deg': float(math.degrees(wp[2]))
                    }
                    for i, wp in enumerate(self.waypoints)
                ],
                'route': [
                    [float(p[0]), float(p[1])] 
                    for p in self.planned_path
                ],
                'instructions': instructions
            }
            
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error guardando JSON: {e}")
            return False
    
    def publish_status(self, status):
        """Publica estado del planner"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def print_status(self):
        """Imprime estado actual"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("📊 ESTADO ACTUAL")
        self.get_logger().info("="*50)
        self.get_logger().info(f"Waypoints: {len(self.waypoints)}")
        self.get_logger().info(f"Ruta generada: {'Sí' if self.planned_path else 'No'}")
        if self.planned_path:
            self.get_logger().info(f"Puntos en ruta: {len(self.planned_path)}")
        self.get_logger().info("="*50)
    
    def print_summary(self):
        """Imprime resumen detallado"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📊 RESUMEN DE PLANIFICACIÓN")
        self.get_logger().info("="*60)
        
        if self.waypoints:
            self.get_logger().info(f"\n📍 Waypoints ({len(self.waypoints)}):")
            for i, (x, y, theta) in enumerate(self.waypoints):
                self.get_logger().info(
                    f"  W{i+1}: ({x:.3f}, {y:.3f}) - {math.degrees(theta):.1f}°"
                )
        
        if self.planned_path:
            path_length = sum(
                self.distance(self.planned_path[i], self.planned_path[i+1]) 
                for i in range(len(self.planned_path)-1)
            )
            
            instructions = self.create_simple_instructions(self.planned_path)
            rotate_count = sum(1 for i in instructions if i['type'] == 'ROTATE')
            forward_count = sum(1 for i in instructions if i['type'] == 'FORWARD')
            combined_count = sum(1 for i in instructions if i.get('combined_steps', 0) > 1)
            
            self.get_logger().info(f"\n🗺️ Ruta Optimizada:")
            self.get_logger().info(f"  Puntos: {len(self.planned_path)}")
            self.get_logger().info(f"  Longitud: {path_length:.2f}m")
            
            self.get_logger().info(f"\n🎯 Instrucciones:")
            self.get_logger().info(f"  Total: {len(instructions)}")
            self.get_logger().info(f"  Rotaciones: {rotate_count}")
            self.get_logger().info(f"  Movimientos: {forward_count}")
            if combined_count > 0:
                self.get_logger().info(f"  🔧 Combinados: {combined_count}")
        
        self.get_logger().info(f"\n💾 Archivos: {self.output_dir}")
        self.get_logger().info("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    planner = OptimizedRRTPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("\n🛑 Deteniendo planner...")
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()