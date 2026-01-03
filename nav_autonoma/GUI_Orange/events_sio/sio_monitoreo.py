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

import wiringpi
from wiringpi import GPIO
from datetime import datetime
from nav_autonoma.configRobot import Config
import time

class Monitoreo_RobotEvents():
    def __init__(self):
        wiringpi.wiringPiSetup()
        self.sensorC02 = SCD40_SMBus()
        self.sensorC02.start_periodic_measurement()
        self.audio = AudioPlayer()
        self.adc = ADS1115()
        self.lamps = 0
        self.motors = 1
        self.status_lamps = False
        wiringpi.pinMode(self.lamps, GPIO.OUTPUT)
        wiringpi.digitalWrite(self.lamps,GPIO.LOW)
        wiringpi.pinMode(self.motors, GPIO.OUTPUT)
        wiringpi.digitalWrite(self.motors,GPIO.LOW)

        # Publicador para control del path follower (se inicializará después)
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

        #self.lidar = TauLidarLogger()
        #self.map_active = False

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
            
            # Fórmula CORRECTA para obtener yaw (theta) de un cuaternión
            # Esta es la fórmula estándar para convertir cuaternión a ángulos de Euler
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self.current_theta = math.atan2(siny_cosp, cosy_cosp)
            
            self.has_odom_data = True
            self.node.get_logger().debug(f"Odometría actualizada: x={self.current_x:.2f}, y={self.current_y:.2f}, θ={self.current_theta:.2f} rad")
            
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

            # # Ruta fija del archivo RRT
            # rrt_file = "/home/orangepi/map_test/ruta_grabada.json"

            # if os.path.exists(rrt_file):
            #     node.get_logger().info(f"📂 Encontrado archivo RRT: {rrt_file}")
                
            #     # Enviar comando para cargar la ruta
            #     load_msg = String()
            #     load_msg.data = f"LOAD:{rrt_file}"
                
            #     self.path_control_pub.publish(load_msg)
                
            #     time.sleep(0.5)
                
            #     # Enviar START
            #     start_msg = String()
            #     start_msg.data = "START"
            #     self.path_control_pub.publish(start_msg)
                
            #     self.node.get_logger().info("📤 Comandos enviados: LOAD y START")
                
            #     # Opcional: destruir publicador después de usar
            #     self.node.destroy_publisher(self.path_control_pub)
                
            # else:
            #     node.get_logger().warn("⚠️ No se encontró archivo RRT, usando modo L")
                
            #     # Modo L tradicional
            #     if self.path_control_pub:
            #         msg = String()
            #         msg.data = "START"
            #         self.path_control_pub.publish(msg)
            #         self.path_follower_running = True
            #         self.node.get_logger().info("📤 Comando START enviado (modo L)")
            # INICIAR TRAYECTORIA EN L
            if self.path_control_pub:
                msg = String()
                msg.data = "START"
                self.path_control_pub.publish(msg)
                self.path_follower_running = True
                self.node.get_logger().info("📤 Comando START enviado a path follower")
            # Responder al servidor con el estado
            self.leds.set_color(r=100,g=0,b=50)
            sio.emit("go-robot", {"status": "go", "success": True})
            # Aquí puedes publicar a un topic ROS2 para iniciar el proceso
            # ejemplo: self.pub_start.publish(String(data="start"))
            sio.emit("start-yolo")
        
        # Inicia Proceso de regreso a casa
        @sio.on("go-home")
        def handle_home(data):
            node.get_logger().info("🏠 Regresando a casa...")
            # Responder con confirmación
            sio.emit("go-home", {"status": "home", "success": True})
            # Aquí publicar a topic de navegación para ir a home
        
        @sio.on("ejec-sound-alert")
        def handle_sound_alert(data):
            node.get_logger().info("🔔 Alerta de sonido recibida")
            if not self.audio.is_playing_audio():
                success = self.audio.play("/home/orangepi/sound_ia/resource/desalojo.mp3", blocking=False)

        # Toogle a para permitir conmutacion de lamparas UVC
        @sio.on("toggle-LampsUVC")
        def handle_toggle_lamps(data):
            # Aquí debes leer el estado real de las lámparas
            # Por ahora simulamos el toggle (deberías guardar el estado)
            node.get_logger().info("Conmutando lámparas UVC...")

            # TODO: Implementar lógica real de toggle
            self.status_lamps = not self.status_lamps  
            msg_lamps = "on" if self.status_lamps else "off"
            node.get_logger().info(f"Lamparas: {msg_lamps}")
            if msg_lamps == "on":
                wiringpi.digitalWrite(self.lamps,GPIO.HIGH)
            elif msg_lamps == "off":
                wiringpi.digitalWrite(self.lamps,GPIO.LOW)
            else:
                wiringpi.digitalWrite(self.lamps,GPIO.LOW)
            sio.emit("uvc-status", {"status": f"{msg_lamps}", "success": True})
            node.get_logger().info(f"Lámparas UVC: {msg_lamps}")
        
        
        # Paro de emergencia general
        @sio.on("stop-all")
        def handle_stop(data):
            node.get_logger().info("PARO DE EMERGENCIA ACTIVADO")
            # Responder con confirmación
            sio.emit("stop-all-now", {"status": "stop", "success": True})
            # Aquí detener todos los motores/procesos del robot
            self.audio.stop()
            wiringpi.digitalWrite(self.lamps,GPIO.LOW)
            wiringpi.digitalWrite(self.motors,GPIO.LOW)
            self.leds.set_color(r=100,g=0,b=0)
        
        # Solicitamos datos del robot
        @sio.on("solicitar-datos")
        def handle_solicitar_datos(data):
            node.get_logger().info("📊 Solicitud de datos recibida")
            
            # TODO: Obtener datos reales desde topics de ROS2
            # Por ahora simulamos (como en tu código original)

            if self.sensorC02.read_measurement():
                co2, temp, humy = self.sensorC02.get_measurement()
            else:
                co2, temp, humy = 1,1,1
            _, bateria_motores, bateria_orangepi  = self.adc.read_channels_0_1()

            #import random
            #bateria = random.randint(20, 100)
            #co2 = random.randint(300, 600)
            sio.emit("datos-bateria", {"battery": bateria_motores,"success": True})
            sio.emit("datos-co2", {"co2": co2,"temp":temp,"humy":humy, "success": True})
            
            node.get_logger().info(f"📤 Batería: {bateria_motores}%, CO2: {co2}ppm")

        # Paro de emergencia general
        @sio.on("manual-control-start")
        def handle_motors_manual():
            node.get_logger().info("Motores activados")
            wiringpi.digitalWrite(self.motors,GPIO.HIGH)
            self.leds.set_color(r=10,g=100,b=100)
        
        @sio.on("panel-control-start")
        def handle_panel_control():
            node.get_logger().info("Motores desactivados")
            wiringpi.digitalWrite(self.motors,GPIO.LOW)
            self.leds.set_color(r=0,g=100,b=0)
        
        @sio.on("odo-data")
        def odom_emit(data):
            node.get_logger().info("🚀 Solicitando datos de odometría...")
            
            if self.has_odom_data:
                node.get_logger().info(f"x: {self.current_x:.3f} y: {self.current_y:.3f}, theta: {self.current_theta:.3f}")
                sio.emit("odo-measure", {
                    "x": float(self.current_x),
                    "y": float(self.current_y),
                    "theta": float(self.current_theta), 
                    "success": True
                })
            else:
                node.get_logger().warning("No hay datos de odometría disponibles")
                sio.emit("odo-measure", {
                    "x": 0.0,
                    "y": 0.0,
                    "theta": 0.0, 
                    "success": False,
                    "message": "No hay datos de odometría disponibles"
                })
                #if Config.ROBOT_CREATE_MAP:
                    #self.lidar.set_pose(self.current_x, self.current_y, self.current_theta)
        
        @sio.on("get-set-home")
        def handle_createmap(data):
            msg = bool(data.get("active", False))
            if msg and not self.map_active:
                #self.lidar.start()
                self.map_active = True
            else:
                #self.lidar.stop()
                self.map_active = False

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
                'x': float(self.current_x),
                'y': float(self.current_y),
                'theta': float(self.current_theta),
                'timestamp': datetime.now().isoformat()
            }
            
            # Agregar a lista local
            self.waypoints_list.append(waypoint)
            
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