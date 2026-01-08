# from library_opi.audio_player import AudioPlayer
# from library_opi.sdc40_sensor import SCD40_SMBus    
# from library_opi.ads1115 import ADS1115
# from library_opi.pca9685_leds import SimpleRGBPCA9685
# from library_opi.lidarpoint import TauLidarLogger



# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32MultiArray
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Point
# import math
# import json
# import os
# import subprocess
# from pathlib import Path

# import wiringpi
# from wiringpi import GPIO
# from datetime import datetime
# from nav_autonoma.configRobot import Config
# import time


# class Monitoreo_RobotEvents():
#     def __init__(self):
#         wiringpi.wiringPiSetup()
#         self.sensorC02 = SCD40_SMBus()
#         self.sensorC02.start_periodic_measurement()
#         self.audio = AudioPlayer()
#         self.adc = ADS1115()
#         self.lamps = 0
#         self.motors = 1
#         self.status_lamps = False
#         wiringpi.pinMode(self.lamps, GPIO.OUTPUT)
#         wiringpi.digitalWrite(self.lamps,GPIO.LOW)
#         wiringpi.pinMode(self.motors, GPIO.OUTPUT)
#         wiringpi.digitalWrite(self.motors,GPIO.LOW)

#         # Publicador para control del path follower (se inicializará después)
#         self.path_control_pub = None
#         self.path_follower_running = False
        
#         # Leds
#         self.leds = SimpleRGBPCA9685()
#         self.leds.set_color(r=100,g=100,b=100)
        
#         # Variables para almacenar la odometría
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_theta = 0.0
#         self.has_odom_data = False
#         self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         # Publicadores para waypoints
#         self.waypoint_pub = None
#         self.waypoints_list = []
#         self.waypoints_file = f"/home/orangepi/map_test/waypoints_{self.timestamp}.json"
        
#         # Contador de waypoints
#         self.waypoint_counter = 0

#         self.lidar = TauLidarLogger()
#         self.map_active = False
#         self.lidar.setup()
#         print("LiDAR OK . . .")

#         # Configuración para transferencia de archivos
#         self.server_user = "cesaramj"
#         self.server_ip = "192.168.0.200"
#         self.server_path = "/home/cesaramj/"
        
#         # Path del archivo LiDAR
#         self.lidar_log_file = "/home/orangepi/lidar_log.jsonl"

#     def odom_callback(self, msg):
#         """Callback para el suscriptor de odometría"""
#         try:
#             self.current_x = msg.pose.pose.position.x
#             self.current_y = msg.pose.pose.position.y

#             q = msg.pose.pose.orientation
#             qx = q.x
#             qy = q.y
#             qz = q.z
#             qw = q.w
            
#             # Fórmula CORRECTA para obtener yaw (theta) de un cuaternión
#             # Esta es la fórmula estándar para convertir cuaternión a ángulos de Euler
#             siny_cosp = 2.0 * (qw * qz + qx * qy)
#             cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
#             self.current_theta = math.atan2(siny_cosp, cosy_cosp)
            
#             self.has_odom_data = True
#             self.node.get_logger().debug(f"Odometría actualizada: x={self.current_x:.2f}, y={self.current_y:.2f}, θ={self.current_theta:.2f} rad")
            
#         except Exception as e:
#             self.node.get_logger().error(f"Error procesando odometría: {e}")

#     def save_waypoints_to_file(self):
#             """Guarda waypoints en archivo JSON"""
#             try:
#                 with open(self.waypoints_file, 'w') as f:
#                     json.dump(self.waypoints_list, f, indent=2)
                
#                 if hasattr(self, 'node'):
#                     self.node.get_logger().info(f"💾 Waypoints guardados: {self.waypoints_file}")
                
#                 return True
#             except Exception as e:
#                 if hasattr(self, 'node'):
#                     self.node.get_logger().error(f"Error guardando waypoints: {e}")
#                 return False

#     def register(self, sio, node):
#         """
#         Registra todos los eventos de monitoreo del robot
        
#         Args:
#             sio: Cliente de socketio
#             node: Nodo de ROS2 
#         """
#         self.node = node
        
#         # Crear publicador para controlar el path follower
#         self.path_control_pub = self.node.create_publisher(
#             String, 
#             '/path_control', 
#             10
#         )

#         # Crear suscriptor para odometría
#         self.odom_sub = self.node.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10
#         )
#         # Crear publicador para waypoints
#         self.waypoint_pub = self.node.create_publisher(
#             Float32MultiArray,
#             '/waypoints',
#             10
#         )

#         self.rrt_control_pub = self.node.create_publisher(
#             String,
#             '/rrt_control',
#             10
#         )

#         # Iniciar proceso de navegación
#         @sio.on("start-process")
#         def handle_start(data):
#             node.get_logger().info("🚀 Iniciando proceso...")
#             wiringpi.digitalWrite(self.motors,GPIO.HIGH)
#             if self.path_control_pub:
#                 msg = String()
#                 msg.data = "START"
#                 self.path_control_pub.publish(msg)
#                 self.path_follower_running = True
#                 self.node.get_logger().info("📤 Comando START enviado a path follower")
#             # Responder al servidor con el estado
#             self.leds.set_color(r=100,g=0,b=50)
#             self.audio.play(Config.sound_go_robot, blocking=False)
#             sio.emit("go-robot", {"status": "go", "success": True})
#             # Aquí puedes publicar a un topic ROS2 para iniciar el proceso
#             # ejemplo: self.pub_start.publish(String(data="start"))
#             sio.emit("start-yolo")
        
#         # Inicia Proceso de regreso a casa
#         @sio.on("go-home")
#         def handle_home(data):
#             node.get_logger().info("🏠 Regresando a casa...")
#             # Responder con confirmación
#             sio.emit("go-home", {"status": "home", "success": True})
#             # Aquí publicar a topic de navegación para ir a home
#             self.audio.play(Config.sound_bellako, blocking=False)
        
#         @sio.on("ejec-sound-alert")
#         def handle_sound_alert(data):
#             node.get_logger().info("🔔 Alerta de sonido recibida")
#             if not self.audio.is_playing_audio():
#                 success = self.audio.play("/home/orangepi/sound_ia/resource/desalojo.mp3", blocking=False)

#         # Toogle a para permitir conmutacion de lamparas UVC
#         @sio.on("toggle-LampsUVC")
#         def handle_toggle_lamps(data):
#             # Aquí debes leer el estado real de las lámparas
#             # Por ahora simulamos el toggle (deberías guardar el estado)
#             node.get_logger().info("Conmutando lámparas UVC...")

#             # TODO: Implementar lógica real de toggle
#             self.status_lamps = not self.status_lamps  
#             msg_lamps = "on" if self.status_lamps else "off"
#             node.get_logger().info(f"Lamparas: {msg_lamps}")
#             if msg_lamps == "on":
#                 wiringpi.digitalWrite(self.lamps,GPIO.HIGH)
#                 self.audio.play(Config.sound_on_lamps, blocking=False)
#             elif msg_lamps == "off":
#                 wiringpi.digitalWrite(self.lamps,GPIO.LOW)
#                 self.audio.play(Config.sound_off_lamps, blocking=False)
#             else:
#                 wiringpi.digitalWrite(self.lamps,GPIO.LOW)
#                 self.audio.play(Config.sound_off_lamps, blocking=False)
#             sio.emit("uvc-status", {"status": f"{msg_lamps}", "success": True})
#             node.get_logger().info(f"Lámparas UVC: {msg_lamps}")
        
        
#         # Paro de emergencia general
#         @sio.on("stop-all")
#         def handle_stop(data):
#             node.get_logger().info("PARO DE EMERGENCIA ACTIVADO")
#             # Responder con confirmación
#             sio.emit("stop-all-now", {"status": "stop", "success": True})
#             # Aquí detener todos los motores/procesos del robot
#             self.audio.stop()
#             wiringpi.digitalWrite(self.lamps,GPIO.LOW)
#             wiringpi.digitalWrite(self.motors,GPIO.LOW)
#             self.leds.set_color(r=100,g=0,b=0)
#             self.audio.play(Config.sound_emergency_stop, blocking=False)
        
#         # Solicitamos datos del robot
#         @sio.on("solicitar-datos")
#         def handle_solicitar_datos(data):
#             node.get_logger().info("📊 Solicitud de datos recibida")
            
#             # TODO: Obtener datos reales desde topics de ROS2
#             # Por ahora simulamos (como en tu código original)

#             if self.sensorC02.read_measurement():
#                 co2, temp, humy = self.sensorC02.get_measurement()
#             else:
#                 co2, temp, humy = 1,1,1
#             _, bateria_motores, bateria_orangepi  = self.adc.read_channels_0_1()

#             #import random
#             #bateria = random.randint(20, 100)
#             #co2 = random.randint(300, 600)
#             sio.emit("datos-bateria", {"battery": bateria_motores,"success": True})
#             sio.emit("datos-co2", {"co2": co2,"temp":temp,"humy":humy, "success": True})
            
#             node.get_logger().info(f"📤 Batería: {bateria_motores}%, CO2: {co2}ppm")

#         # Paro de emergencia general
#         @sio.on("manual-control-start")
#         def handle_motors_manual():
#             node.get_logger().info("Motores activados")
#             wiringpi.digitalWrite(self.motors,GPIO.HIGH)
#             self.leds.set_color(r=10,g=100,b=100)
#             self.audio.play(Config.sound_ctrl_manual, blocking=False)
        
        
#         @sio.on("panel-control-start")
#         def handle_panel_control():
#             node.get_logger().info("Motores desactivados")
#             wiringpi.digitalWrite(self.motors,GPIO.LOW)
#             self.leds.set_color(r=0,g=100,b=0)
#             self.audio.play(Config.sound_mnt_gui, blocking=False)
        
#         @sio.on("odo-data")
#         def odom_emit(data):
#             node.get_logger().info("🚀 Solicitando datos de odometría...")
            
#             if self.has_odom_data:
#                 node.get_logger().info(f"x: {self.current_x:.3f} y: {self.current_y:.3f}, theta: {self.current_theta:.3f}")
#                 sio.emit("odo-measure", {
#                     "x": float(self.current_x),
#                     "y": float(self.current_y),
#                     "theta": float(self.current_theta), 
#                     "success": True
#                 })
#             else:
#                 node.get_logger().warning("No hay datos de odometría disponibles")
#                 sio.emit("odo-measure", {
#                     "x": 0.0,
#                     "y": 0.0,
#                     "theta": 0.0, 
#                     "success": False,
#                     "message": "No hay datos de odometría disponibles"
#                 })
                
#             if self.map_active:
#                 pose = (self.current_x, self.current_y, self.current_theta)
#                 self.lidar.capture_once(pose)
#                 node.get_logger().info("Cap LiDAR")
        
#         @sio.on("get-set-home")
#         def handle_createmap(data):
#             msg = bool(data.get("active", False))
#             if msg and not self.map_active:
#                 self.lidar.start()
#                 self.map_active = True
#                 node.get_logger().info("Iniciado LiDAR")
#             else:
#                 self.lidar.stop()
#                 self.map_active = False
#                 node.get_logger().info("Terminando LiDAR")
#                 self.lidar.close()
#                 node.get_logger().info("Enviando archivo LiDAR al servidor...")
                
#                 if os.path.exists(self.lidar_log_file):
#                     file_size = os.path.getsize(self.lidar_log_file) / 1024  # KB
#                     node.get_logger().info(f"   Archivo: {self.lidar_log_file} ({file_size:.2f} KB)")
                    
#                     scp_success = self.send_file_to_server(self.lidar_log_file)
                    
#                     if scp_success:
#                         node.get_logger().info("✅ Archivo LiDAR enviado exitosamente")
#                         sio.emit("lidar-transfer-status", {
#                             "success": True,
#                             "message": "Archivo LiDAR transferido",
#                             "size_kb": file_size
#                         })
#                     else:
#                         node.get_logger().warning("⚠️  Error enviando archivo (continuando...)")
#                         sio.emit("lidar-transfer-status", {
#                             "success": False,
#                             "message": "Error en transferencia SCP"
#                         })
#                 else:
#                     node.get_logger().warning("⚠️  Archivo LiDAR no existe (continuando...)")
#                     sio.emit("lidar-transfer-status", {
#                         "success": False,
#                         "message": "Archivo LiDAR no encontrado"
#                     })


#                 node.get_logger().info("Generando ruta RRT...")
                
#                 if len(self.waypoints_list) < 2:
#                     node.get_logger().warning("⚠️  Menos de 2 waypoints, no se generará ruta")
#                     sio.emit("rrt-generation-status", {
#                         "success": False,
#                         "message": "Se necesitan al menos 2 waypoints"
#                     })
#                 else:
#                     if self.rrt_control_pub:
#                         msg = String()
#                         msg.data = "GENERATE_PATH"
#                         self.rrt_control_pub.publish(msg)
#                         node.get_logger().info("✅ Comando GENERATE_PATH enviado a RRT planner")
#                         sio.emit("rrt-generation-status", {
#                             "success": True,
#                             "message": "Ruta RRT en generación (local)",
#                             "waypoints": len(self.waypoints_list)
#                         })
#                     else:
#                         node.get_logger().warning("⚠️  rrt_control_pub no disponible")
#                         sio.emit("rrt-generation-status", {
#                             "success": False,
#                             "message": "Publicador RRT no disponible"
#                         })
                
#         # Evento para marcar waypoint actual
#         @sio.on("get-mark_waypoint")
#         def mapa_pos(data=None):
#             if not self.has_odom_data:
#                 sio.emit("waypoint_error", {
#                     "message": "No hay datos de odometría disponibles",
#                     "success": False
#                 })
#                 return
            
#             self.waypoint_counter += 1
#             waypoint = {
#                 'id': self.waypoint_counter,
#                 'x': float(self.current_x)*2,
#                 'y': float(self.current_y)*2,
#                 'theta': float(self.current_theta),
#                 'timestamp': datetime.now().isoformat()
#             }
            
#             # Agregar a lista local
#             self.waypoints_list.append(waypoint)
#             self.audio.play(Config.sound_disconnect, blocking=False)
#             # PUBLICAR waypoint como mensaje ROS2
#             waypoint_msg = Float32MultiArray()
#             waypoint_msg.data = [waypoint['x'], waypoint['y'], waypoint['theta']]
#             self.waypoint_pub.publish(waypoint_msg)
            
#             # Guardar en archivo
#             self.save_waypoints_to_file()
            
#             # Enviar confirmación al cliente web
#             sio.emit("waypoint_added", {
#                 "id": waypoint['id'],
#                 "x": waypoint['x'],
#                 "y": waypoint['y'],
#                 "theta": waypoint['theta'],
#                 "total": len(self.waypoints_list),
#                 "success": True
#             })
            
#             self.node.get_logger().info(
#                 f"📍 Waypoint {waypoint['id']}: ({waypoint['x']:.3f}, {waypoint['y']:.3f})"
#             )
            
#         node.get_logger().info("✅ Eventos de monitoreo registrados correctamente")

#     def send_file_to_server(self, file_path):
#         """
#         Envía un archivo al servidor mediante SCP.
        
#         Args:
#             file_path: Ruta completa del archivo a enviar
            
#         Returns:
#             bool: True si el envío fue exitoso, False en caso contrario
#         """
#         try:
#             # Verificar que existe el archivo
#             if not os.path.exists(file_path):
#                 self.node.get_logger().error(f"❌ Archivo no existe: {file_path}")
#                 return False
            
#             # Construir comando SCP
#             remote_dest = f"{self.server_user}@{self.server_ip}:{self.server_path}/"
            
#             # Comando SCP con opciones para evitar interacción
#             scp_command = [
#                 "scp",
#                 "-o", "StrictHostKeyChecking=no",
#                 "-o", "UserKnownHostsFile=/dev/null",
#                 "-o", "ConnectTimeout=10",
#                 file_path,
#                 remote_dest
#             ]
            
#             self.node.get_logger().info(f"   Ejecutando SCP...")
#             self.node.get_logger().debug(f"   Comando: {' '.join(scp_command)}")
            
#             # Ejecutar comando
#             result = subprocess.run(
#                 scp_command,
#                 capture_output=True,
#                 text=True,
#                 timeout=30
#             )
            
#             if result.returncode == 0:
#                 self.node.get_logger().info(f"   ✅ Archivo enviado: {os.path.basename(file_path)}")
#                 return True
#             else:
#                 self.node.get_logger().error(f"   ❌ Error SCP (código {result.returncode})")
#                 if result.stderr:
#                     self.node.get_logger().error(f"   Error: {result.stderr.strip()}")
#                 return False
                
#         except subprocess.TimeoutExpired:
#             self.node.get_logger().error("   ❌ Timeout en transferencia SCP (>30s)")
#             return False
#         except Exception as e:
#             self.node.get_logger().error(f"   ❌ Excepción en SCP: {e}")
#             return False


from library_opi.audio_player import AudioPlayer
from library_opi.sdc40_sensor import SCD40_SMBus    
from library_opi.ads1115 import ADS1115
from library_opi.pca9685_leds import SimpleRGBPCA9685
from library_opi.lidarpoint import TauLidarLogger

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math
import json
import os
import subprocess
from pathlib import Path

import wiringpi
from wiringpi import GPIO
from datetime import datetime
from nav_autonoma.configRobot import Config
import time
import smbus2


class Monitoreo_RobotEvents():
    def __init__(self):
        wiringpi.wiringPiSetup()
        self.audio = AudioPlayer()
        smbus2.SMBus(8).close()
        self.adc = ADS1115()
        self.lamps = 0
        self.motors = 1
        self.status_lamps = False
        wiringpi.pinMode(self.lamps, GPIO.OUTPUT)
        wiringpi.digitalWrite(self.lamps,GPIO.LOW)
        wiringpi.pinMode(self.motors, GPIO.OUTPUT)
        wiringpi.digitalWrite(self.motors,GPIO.LOW)

        # Publicador para control del path follower
        self.path_control_pub = None
        self.path_follower_running = False
        
        # Leds
        self.leds = SimpleRGBPCA9685()
        self.leds.set_color(r=100,g=100,b=100)
        
        # Variables para almacenar la odometría
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.has_odom_data = False
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Publicadores para waypoints
        self.waypoint_pub = None
        self.waypoints_list = []
        self.waypoints_file = f"/home/orangepi/map_test/waypoints_{self.timestamp}.json"
        
        # Contador de waypoints
        self.waypoint_counter = 0

        self.lidar = TauLidarLogger()
        self.map_active = False
        self.lidar.setup()
        print("LiDAR OK . . .")

        # Configuración para transferencia de archivos
        self.server_user = "cesaramj"
        self.server_ip = "192.168.0.200"
        self.server_path = "/home/cesaramj/"
        
        # Path del archivo LiDAR
        self.lidar_log_file = "/home/orangepi/lidar_log.jsonl"
        self.rrt_file = "/home/orangepi/map_test/ruta_grabada.json"

    def odom_callback(self, msg):
        """Callback para el suscriptor de odometría"""
        try:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            qx = q.x
            qy = q.y
            qz = q.z
            qw = q.w
            
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self.current_theta = math.atan2(siny_cosp, cosy_cosp)
            
            self.has_odom_data = True
            self.node.get_logger().debug(f"Odometría: x={self.current_x:.2f}, y={self.current_y:.2f}, θ={self.current_theta:.2f}")
            
        except Exception as e:
            self.node.get_logger().error(f"Error procesando odometría: {e}")

    def save_waypoints_to_file(self):
        """Guarda waypoints en archivo JSON"""
        try:
            with open(self.waypoints_file, 'w') as f:
                json.dump(self.waypoints_list, f, indent=2)
            
            if hasattr(self, 'node'):
                self.node.get_logger().info(f"💾 Waypoints guardados: {self.waypoints_file}")
            
            return True
        except Exception as e:
            if hasattr(self, 'node'):
                self.node.get_logger().error(f"Error guardando waypoints: {e}")
            return False

    def register(self, sio, node):
        """
        Registra todos los eventos de monitoreo del robot
        
        Args:
            sio: Cliente de socketio
            node: Nodo de ROS2 
        """
        self.node = node

        self.sensorC02 = SCD40_SMBus(node=node)
        self.sensorC02.start_periodic_measurement()
        
        # Crear publicador para controlar el path follower
        self.path_control_pub = self.node.create_publisher(
            String, 
            '/path_control', 
            10
        )

        # Crear suscriptor para odometría
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Crear publicador para waypoints
        self.waypoint_pub = self.node.create_publisher(
            Float32MultiArray,
            '/waypoints',
            10
        )

        # ⭐ Publicador para comandos RRT
        self.rrt_control_pub = self.node.create_publisher(
            String,
            '/rrt_control',
            10
        )

        # Iniciar proceso de navegación
        @sio.on("start-process")
        def handle_start(data):
            node.get_logger().info("🚀 Iniciando proceso...")
            wiringpi.digitalWrite(self.motors,GPIO.HIGH)
            
            if self.path_control_pub:
                msg = String()
                msg.data = "START"
                self.path_control_pub.publish(msg)
                self.path_follower_running = True
                self.node.get_logger().info("📤 Comando START enviado a path follower")
            
            self.leds.set_color(r=100,g=0,b=50)
            self.audio.play(Config.sound_go_robot, blocking=False)
            sio.emit("go-robot", {"status": "go", "success": True})
            sio.emit("start-yolo")
        
        # Inicia Proceso de regreso a casa
        @sio.on("go-home")
        def handle_home(data):
            node.get_logger().info("🏠 Regresando a casa...")
            sio.emit("go-home", {"status": "home", "success": True})
            self.audio.play(Config.sound_bellako, blocking=False)
        
        @sio.on("ejec-sound-alert")
        def handle_sound_alert(data):
            node.get_logger().info("🔔 Alerta de sonido recibida")
            if not self.audio.is_playing_audio():
                success = self.audio.play("/home/orangepi/sound_ia/resource/desalojo.mp3", blocking=False)

        # Toggle para conmutación de lámparas UVC
        @sio.on("toggle-LampsUVC")
        def handle_toggle_lamps(data):
            node.get_logger().info("Conmutando lámparas UVC...")

            self.status_lamps = not self.status_lamps  
            msg_lamps = "on" if self.status_lamps else "off"
            node.get_logger().info(f"Lamparas: {msg_lamps}")
            
            if msg_lamps == "on":
                wiringpi.digitalWrite(self.lamps,GPIO.HIGH)
                self.audio.play(Config.sound_on_lamps, blocking=False)
            else:
                wiringpi.digitalWrite(self.lamps,GPIO.LOW)
                self.audio.play(Config.sound_off_lamps, blocking=False)
            
            sio.emit("uvc-status", {"status": f"{msg_lamps}", "success": True})
            node.get_logger().info(f"Lámparas UVC: {msg_lamps}")
        
        # Paro de emergencia general
        @sio.on("stop-all")
        def handle_stop(data):
            node.get_logger().info("⛔ PARO DE EMERGENCIA ACTIVADO")
            sio.emit("stop-all-now", {"status": "stop", "success": True})
            
            self.audio.stop()
            wiringpi.digitalWrite(self.lamps,GPIO.LOW)
            wiringpi.digitalWrite(self.motors,GPIO.LOW)
            self.leds.set_color(r=100,g=0,b=0)
            self.audio.play(Config.sound_emergency_stop, blocking=False)
            self.sensorC02.stop_periodic_measurement()
            self.sensorC02.close()
        
        # Solicitamos datos del robot
        @sio.on("solicitar-datos")
        def handle_solicitar_datos(data):
            node.get_logger().info("📊 Solicitud de datos recibida")
            
            if self.sensorC02.read_measurement():
                co2, temp, humy = self.sensorC02.get_measurement()
            else:
                co2, temp, humy = 1, 1, 1
                self.sensorC02.start_periodic_measurement()

            _, bateria_motores, bateria_orangepi = self.adc.read_channels_0_1()

            sio.emit("datos-bateria", {"battery": bateria_motores, "success": True})
            sio.emit("datos-co2", {"co2": co2, "temp": temp, "humy": humy, "success": True})
            
            node.get_logger().info(f"📤 Batería: {bateria_motores}%, CO2: {co2}ppm")

        # Control manual
        @sio.on("manual-control-start")
        def handle_motors_manual():
            node.get_logger().info("🎮 Motores activados (control manual)")
            wiringpi.digitalWrite(self.motors,GPIO.HIGH)
            self.leds.set_color(r=10,g=100,b=100)
            self.audio.play(Config.sound_ctrl_manual, blocking=False)
        
        @sio.on("panel-control-start")
        def handle_panel_control():
            node.get_logger().info("🖥️ Panel de control activado")
            wiringpi.digitalWrite(self.motors,GPIO.LOW)
            self.leds.set_color(r=0,g=100,b=0)
            self.audio.play(Config.sound_mnt_gui, blocking=False)
        
        @sio.on("odo-data")
        def odom_emit(data):
            node.get_logger().info("📍 Solicitando datos de odometría...")
            
            if self.has_odom_data:
                node.get_logger().info(f"x: {self.current_x:.3f} y: {self.current_y:.3f}, theta: {self.current_theta:.3f}")
                sio.emit("odo-measure", {
                    "x": float(self.current_x),
                    "y": float(self.current_y),
                    "theta": float(self.current_theta), 
                    "success": True
                })
            else:
                node.get_logger().warning("⚠️ No hay datos de odometría disponibles")
                sio.emit("odo-measure", {
                    "x": 0.0,
                    "y": 0.0,
                    "theta": 0.0, 
                    "success": False,
                    "message": "No hay datos de odometría disponibles"
                })
                
            if self.map_active:
                pose = (self.current_x, self.current_y, self.current_theta)
                self.lidar.capture_once(pose)
                node.get_logger().info("📷 Captura LiDAR")
        
        @sio.on("get-set-home")
        def handle_createmap(data):
            msg = bool(data.get("active", False))
            
            if msg and not self.map_active:
                # ========== INICIANDO MAPEO ==========
                self.lidar.start()
                self.map_active = True
                node.get_logger().info("🗺️ LiDAR iniciado - Mapeo activo")
                
            else:
                # ========== FINALIZANDO MAPEO Y GENERANDO RUTA ==========
                self.lidar.stop()
                self.map_active = False
                node.get_logger().info("⏹️ LiDAR detenido - Procesando datos...")
                self.lidar.close()
                
                # ===== 1. TRANSFERIR ARCHIVO LIDAR =====
                node.get_logger().info("📡 Enviando archivo LiDAR al servidor...")
                
                if os.path.exists(self.lidar_log_file):
                    file_size = os.path.getsize(self.lidar_log_file) / 1024
                    node.get_logger().info(f"   📄 Archivo: {self.lidar_log_file} ({file_size:.2f} KB)")
                    
                    scp_success = self.send_file_to_server(self.lidar_log_file)
                    
                    if scp_success:
                        node.get_logger().info("✅ Archivo LiDAR enviado exitosamente")
                        sio.emit("lidar-transfer-status", {
                            "success": True,
                            "message": "Archivo LiDAR transferido",
                            "size_kb": file_size
                        })
                    else:
                        node.get_logger().warning("⚠️ Error enviando archivo (continuando...)")
                        sio.emit("lidar-transfer-status", {
                            "success": False,
                            "message": "Error en transferencia SCP"
                        })
                else:
                    node.get_logger().warning("⚠️ Archivo LiDAR no existe")
                    sio.emit("lidar-transfer-status", {
                        "success": False,
                        "message": "Archivo LiDAR no encontrado"
                    })

                # ===== 2. GENERAR RUTA RRT =====
                node.get_logger().info("🛤️ Generando ruta RRT...")
                
                if len(self.waypoints_list) < 2:
                    node.get_logger().warning("⚠️ Menos de 2 waypoints, no se generará ruta")
                    sio.emit("rrt-generation-status", {
                        "success": False,
                        "message": "Se necesitan al menos 2 waypoints"
                    })
                else:
                    # ✅ PASO 1: Enviar GENERATE_PATH al RRT Planner
                    if self.rrt_control_pub:
                        msg_rrt = String()
                        msg_rrt.data = "GENERATE_PATH"
                        self.rrt_control_pub.publish(msg_rrt)
                        node.get_logger().info(
                            f"✅ [1/2] Comando GENERATE_PATH enviado a RRT planner "
                            f"({len(self.waypoints_list)} waypoints)"
                        )
                        
                        # ✅ PASO 2: Esperar y luego enviar RELOAD al Path Follower
                        def send_reload_to_path_follower():
                            """
                            Callback ejecutado después de 3 segundos.
                            Envía RELOAD al PATH FOLLOWER (no al RRT planner).
                            """
                            try:
                                if self.path_control_pub:
                                    node.get_logger().info("🔄 [2/2] Enviando RELOAD al path follower...")
                                    reload_msg = String()
                                    reload_msg.data = "LOAD:"+self.rrt_file
                                    # ⭐ CORRECCIÓN: Enviar a path_control_pub (Path Follower)
                                    # NO a rrt_control_pub (RRT Planner)
                                    self.path_control_pub.publish(reload_msg)
                                    
                                    node.get_logger().info("✅ Path follower recargado - Ruta lista para ejecutar")
                                    
                                    sio.emit("rrt-generation-status", {
                                        "success": True,
                                        "message": "Ruta RRT generada y cargada en path follower",
                                        "waypoints": len(self.waypoints_list),
                                        "status": "ready_to_start"
                                    })
                                else:
                                    node.get_logger().error("❌ path_control_pub no disponible")
                                    sio.emit("rrt-generation-status", {
                                        "success": False,
                                        "message": "Path control publisher no disponible"
                                    })
                                    
                            except Exception as e:
                                node.get_logger().error(f"❌ Error enviando RELOAD: {e}")
                                sio.emit("rrt-generation-status", {
                                    "success": False,
                                    "message": f"Error en RELOAD: {str(e)}"
                                })
                        
                        # Crear timer one-shot para enviar RELOAD después de 3 segundos
                        reload_timer = self.node.create_timer(3.0, send_reload_to_path_follower)
                        
                        # Cancelar el timer después de ejecutarlo (one-shot)
                        def cancel_reload_timer():
                            reload_timer.cancel()
                        
                        cancel_timer = self.node.create_timer(3.1, cancel_reload_timer)
                        
                        # Notificación inicial
                        sio.emit("rrt-generation-status", {
                            "success": True,
                            "message": "Generando ruta RRT (esperando 3s para cargar)...",
                            "waypoints": len(self.waypoints_list),
                            "status": "generating"
                        })
                    else:
                        node.get_logger().warning("⚠️ rrt_control_pub no disponible")
                        sio.emit("rrt-generation-status", {
                            "success": False,
                            "message": "Publicador RRT no disponible"
                        })
                
        # Evento para marcar waypoint actual
        @sio.on("get-mark_waypoint")
        def mapa_pos(data=None):
            if not self.has_odom_data:
                sio.emit("waypoint_error", {
                    "message": "No hay datos de odometría disponibles",
                    "success": False
                })
                return
            
            self.waypoint_counter += 1
            waypoint = {
                'id': self.waypoint_counter,
                'x': float(self.current_x)*1.75,
                'y': float(self.current_y)*1.8,
                'theta': float(self.current_theta),
                'timestamp': datetime.now().isoformat()
            }
            
            # Agregar a lista local
            self.waypoints_list.append(waypoint)
            self.audio.play(Config.sound_disconnect, blocking=False)
            
            # PUBLICAR waypoint como mensaje ROS2
            waypoint_msg = Float32MultiArray()
            waypoint_msg.data = [waypoint['x'], waypoint['y'], waypoint['theta']]
            self.waypoint_pub.publish(waypoint_msg)
            
            # Guardar en archivo
            self.save_waypoints_to_file()
            
            # Enviar confirmación al cliente web
            sio.emit("waypoint_added", {
                "id": waypoint['id'],
                "x": waypoint['x'],
                "y": waypoint['y'],
                "theta": waypoint['theta'],
                "total": len(self.waypoints_list),
                "success": True
            })
            
            self.node.get_logger().info(
                f"📍 Waypoint {waypoint['id']}: ({waypoint['x']:.3f}, {waypoint['y']:.3f})"
            )
            
        node.get_logger().info("✅ Eventos de monitoreo registrados correctamente")

    def send_file_to_server(self, file_path):
        """
        Envía un archivo al servidor mediante SCP.
        
        Args:
            file_path: Ruta completa del archivo a enviar
            
        Returns:
            bool: True si el envío fue exitoso, False en caso contrario
        """
        try:
            if not os.path.exists(file_path):
                self.node.get_logger().error(f"❌ Archivo no existe: {file_path}")
                return False
            
            remote_dest = f"{self.server_user}@{self.server_ip}:{self.server_path}/"
            
            scp_command = [
                "scp",
                "-o", "StrictHostKeyChecking=no",
                "-o", "UserKnownHostsFile=/dev/null",
                "-o", "ConnectTimeout=10",
                file_path,
                remote_dest
            ]
            
            self.node.get_logger().info(f"   🚀 Ejecutando SCP...")
            self.node.get_logger().debug(f"   📝 Comando: {' '.join(scp_command)}")
            
            result = subprocess.run(
                scp_command,
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                self.node.get_logger().info(f"   ✅ Archivo enviado: {os.path.basename(file_path)}")
                return True
            else:
                self.node.get_logger().error(f"   ❌ Error SCP (código {result.returncode})")
                if result.stderr:
                    self.node.get_logger().error(f"   📋 Error: {result.stderr.strip()}")
                return False
                
        except subprocess.TimeoutExpired:
            self.node.get_logger().error("   ⏱️ Timeout en transferencia SCP (>30s)")
            return False
        except Exception as e:
            self.node.get_logger().error(f"   ❌ Excepción en SCP: {e}")
            return False
