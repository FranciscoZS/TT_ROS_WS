#!/usr/bin/env python3
"""
Nodo ROS2 para control de motores via UART (Arduino Nano)
Soporta modo MANUAL (desde Socket.IO) y AUTONOMOUS (desde cmd_vel)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import serial
import threading
import time

class MotorUARTNode(Node):
    def __init__(self):
        super().__init__('motor_uart_node')
        
        # ==================== PARÁMETROS ====================
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('mode', 'manual')  # 'manual' o 'autonomous'
        self.declare_parameter('heartbeat_rate', 2.0)  # Hz
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.mode = self.get_parameter('mode').value
        heartbeat_rate = self.get_parameter('heartbeat_rate').value
        
        # ==================== CONEXIÓN UART ====================
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(3)  # Esperar reset de Arduino
            self.get_logger().info(f'✅ Conectado a Arduino en {port}')
        except Exception as e:
            self.get_logger().error(f'❌ Error conectando UART: {e}')
            raise
        
        # Iniciar thread de lectura
        self._start_reader()
        time.sleep(1)
        
        # ==================== CONFIGURAR ARDUINO ====================
        self._configure_motors()
        self._set_mode(self.mode)
        
        # ==================== SUBSCRIBERS ====================
        # Control manual desde Socket.IO
        self.manual_cmd_sub = self.create_subscription(
            String,
            'motor/manual_command',
            self.manual_command_callback,
            10
        )
        
        # Control autónomo desde navegación
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Cambio de modo
        self.mode_sub = self.create_subscription(
            String,
            'motor/set_mode',
            self.mode_callback,
            10
        )
        
        # ==================== PUBLISHERS ====================
        self.status_pub = self.create_publisher(
            String,
            'motor/status',
            10
        )
        
        # ==================== TIMERS ====================
        # Heartbeat para mantener conexión con Arduino
        self.heartbeat_timer = self.create_timer(
            1.0 / heartbeat_rate,
            self.heartbeat_callback
        )
        
        # ==================== VARIABLES ====================
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.5  # segundos
        
        self.get_logger().info(
            f'✅ Nodo de motores inicializado\n'
            f'   Modo: {self.mode}\n'
            f'   Puerto: {port}\n'
            f'   Baudrate: {baudrate}'
        )

    def _start_reader(self):
        """Inicia thread para leer respuestas del Arduino"""
        def reader():
            while rclpy.ok():
                try:
                    if self.ser.in_waiting > 0:
                        response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            self.get_logger().info(f'🤖 Arduino: {response}')
                            
                            # Publicar status si es relevante
                            if response.startswith('STATUS:') or response.startswith('OK:'):
                                msg = String()
                                msg.data = response
                                self.status_pub.publish(msg)
                                
                except Exception as e:
                    if "Bad file descriptor" not in str(e):
                        self.get_logger().error(f'Error leyendo UART: {e}')
                    time.sleep(0.1)
        
        self.reader_thread = threading.Thread(target=reader, daemon=True)
        self.reader_thread.start()

    def _send_command(self, command):
        """Envía comando al Arduino"""
        try:
            full_command = f"{command}\n"
            self.ser.write(full_command.encode())
            self.ser.flush()
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Error enviando comando: {e}')
            return False

    def _configure_motors(self):
        """Configura los 4 motores en el Arduino"""
        self.get_logger().info('⚙️ Configurando motores en Arduino...')
        
        # Configuración según tu hardware
        motor_configs = [
            (0, 2, 3),   # Motor 0: PWM=2, DIR=3
            (1, 7, 6),   # Motor 1: PWM=7, DIR=6
            (2, 8, 5),   # Motor 2: PWM=8, DIR=5
            (3, 9, 4)    # Motor 3: PWM=9, DIR=4
        ]
        
        for motor_id, pwm_pin, dir_pin in motor_configs:
            cmd = f"CONFIG_MOTOR,{motor_id},{pwm_pin},{dir_pin}"
            self._send_command(cmd)
            time.sleep(0.5)
        
        self.get_logger().info('✅ Motores configurados')

    def _set_mode(self, mode):
        """Cambia el modo del Arduino"""
        if mode == 'manual':
            self._send_command('MODE:MANUAL')
        elif mode == 'autonomous':
            self._send_command('MODE:AUTONOMOUS')
        elif mode == 'stop':
            self._send_command('MODE:STOP')
        else:
            self.get_logger().error(f'Modo desconocido: {mode}')

    def manual_command_callback(self, msg):
        """
        Recibe comandos de control manual desde Socket.IO
        Formato esperado: "direction:action" 
        Ejemplos: "forward:press", "left:release"
        """
        if self.mode != 'manual':
            self.get_logger().warn('⚠️ Comando manual recibido pero no en modo manual')
            return
        
        try:
            parts = msg.data.split(':')
            if len(parts) != 2:
                self.get_logger().error(f'Formato inválido: {msg.data}')
                return
            
            direction = parts[0]
            action = parts[1]
            
            # Mapear direcciones a movimientos del Arduino
            movement_map = {
                'forward': 1,       # MOVE_FORWARD
                'backward': 2,      # MOVE_BACKWARD
                'left': 3,          # MOVE_LEFT
                'right': 4,         # MOVE_RIGHT
                'cw': 5,            # MOVE_CW
                'ccw': 6,           # MOVE_CCW
                'forward_left': 8,  # MOVE_DIAGONAL_FL
                'forward_right': 7, # MOVE_DIAGONAL_FR
                'backward_left': 10,# MOVE_DIAGONAL_BL
                'backward_right': 9 # MOVE_DIAGONAL_BR
            }
            
            if action == 'press':
                # Iniciar movimiento
                if direction in movement_map:
                    movement = movement_map[direction]
                    speed = 60  # Velocidad por defecto
                    cmd = f"MANUAL:{movement},{speed}"
                    self._send_command(cmd)
                    self.last_cmd_time = time.time()
                    self.get_logger().info(f'▶️ Movimiento: {direction}')
                else:
                    self.get_logger().warn(f'Dirección desconocida: {direction}')
                    
            elif action == 'release':
                # Detener movimiento
                cmd = "MANUAL:0,0"  # MOVE_STOP
                self._send_command(cmd)
                self.get_logger().info('⏹️ Detenido')
                
        except Exception as e:
            self.get_logger().error(f'Error procesando comando manual: {e}')

    def cmd_vel_callback(self, msg):
        """
        Recibe comandos de velocidad desde navegación autónoma
        Convierte Twist a comandos individuales por motor
        """
        if self.mode != 'autonomous':
            return
        
        try:
            # Cinemática inversa para mecanum
            vx = msg.linear.x
            vy = msg.linear.y
            wz = msg.angular.z
            
            # Calcular velocidades de ruedas (simplificado)
            # FL, FR, RL, RR
            wheel_speeds = self._mecanum_inverse_kinematics(vx, vy, wz)
            
            # Convertir a duty cycles (0-100)
            duties = [self._speed_to_duty(ws) for ws in wheel_speeds]
            
            # Determinar direcciones
            directions = [1 if ws >= 0 else 0 for ws in wheel_speeds]
            
            # Construir comando para Arduino
            # Formato: AUTO:dir0,duty0,dir1,duty1,dir2,duty2,dir3,duty3
            cmd = "AUTO:" + ",".join([f"{d},{abs(duty)}" for d, duty in zip(directions, duties)])
            
            self._send_command(cmd)
            self.last_cmd_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f'Error en cmd_vel: {e}')

    def _mecanum_inverse_kinematics(self, vx, vy, wz):
        """Cinemática inversa para robot mecanum"""
        L = 0.3 / 2.0  # Semi-distancia longitudinal
        W = 0.4 / 2.0  # Semi-distancia transversal
        R = 0.05       # Radio de rueda
        
        # Velocidades de rueda en m/s
        wheel_fl = (vx - vy - (L + W) * wz) / R
        wheel_fr = (vx + vy + (L + W) * wz) / R
        wheel_rl = (vx + vy - (L + W) * wz) / R
        wheel_rr = (vx - vy + (L + W) * wz) / R
        
        return [wheel_fl, wheel_fr, wheel_rl, wheel_rr]

    def _speed_to_duty(self, speed):
        """Convierte velocidad (m/s) a duty cycle (%)"""
        max_speed = 0.5  # m/s
        duty = int((abs(speed) / max_speed) * 100)
        return min(duty, 100)

    def mode_callback(self, msg):
        """Cambia el modo de operación"""
        new_mode = msg.data.lower()
        
        if new_mode in ['manual', 'autonomous', 'stop']:
            self.mode = new_mode
            self._set_mode(new_mode)
            self.get_logger().info(f'🔄 Modo cambiado a: {new_mode}')
        else:
            self.get_logger().error(f'Modo inválido: {new_mode}')

    def heartbeat_callback(self):
        """Envía ping al Arduino para mantener conexión"""
        self._send_command('PING')

    def destroy_node(self):
        """Cleanup al cerrar"""
        self.get_logger().info('🛑 Cerrando nodo de motores...')
        self._send_command('STOP')
        time.sleep(0.5)
        
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorUARTNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
