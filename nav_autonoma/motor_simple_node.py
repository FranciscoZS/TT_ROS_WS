# #!/usr/bin/env python3
# """
# Nodo ROS2 SIMPLIFICADO para control de motores via UART
# Solo recibe comandos de movimiento y los envía al Arduino
# """
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import serial
# import threading
# import time

# class MotorSimpleNode(Node):
#     def __init__(self):
#         super().__init__('motor_simple_node')
        
#         # ==================== PARÁMETROS ====================
#         self.declare_parameter('port', '/dev/ttyUSB0')
#         self.declare_parameter('baudrate', 115200)
#         self.declare_parameter('default_speed', 50)
        
#         port = self.get_parameter('port').value
#         baudrate = self.get_parameter('baudrate').value
#         self.default_speed = self.get_parameter('default_speed').value
        
#         # ==================== CONEXIÓN UART ====================
#         try:
#             self.ser = serial.Serial(port, baudrate, timeout=1)
#             time.sleep(3)  # Esperar reset de Arduino
#             self.get_logger().info(f'✅ Conectado a Arduino en {port}')
#         except Exception as e:
#             self.get_logger().error(f'❌ Error conectando UART: {e}')
#             raise
        
#         # Iniciar thread de lectura
#         self._start_reader()
#         time.sleep(1)
        

#!/usr/bin/env python3
"""
Nodo ROS2 SIMPLIFICADO para control de motores via UART
Solo recibe comandos de movimiento y los envía al Arduino
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time

class MotorSimpleNode(Node):
    def __init__(self):
        super().__init__('motor_simple_node')
        
        # ==================== PARÁMETROS ====================
        # Puerto UART4 de Orange Pi 5 Max
        self.declare_parameter('port', '/dev/ttyS4')  # UART4
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('default_speed', 50)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.default_speed = self.get_parameter('default_speed').value
        
        # ==================== CONEXIÓN UART ====================
        try:
            # Configuración específica para UART4 de Orange Pi
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            time.sleep(3)  # Esperar reset de Arduino
            self.get_logger().info(f'✅ Conectado a Arduino en {port} (UART4)')
        except Exception as e:
            self.get_logger().error(f'❌ Error conectando UART4: {e}')
            self.get_logger().info('Verifica que:')
            self.get_logger().info('1. El UART4 esté habilitado en la Orange Pi')
            self.get_logger().info('2. Los permisos de /dev/ttyS4 sean correctos')
            self.get_logger().info('3. El Arduino esté conectado a los pines TX/RX correctos')
            raise
       
        # Iniciar thread de lectura
        self._start_reader()
        time.sleep(1)
        # ==================== SUBSCRIBER ====================
        # Solo un subscriber para comandos de movimiento
        self.cmd_sub = self.create_subscription(
            String,
            'motor_command',
            self.command_callback,
            10
        )
        
        self.get_logger().info(
            f'✅ Nodo de motores simplificado inicializado\n'
            f'   Puerto: {port}\n'
            f'   Baudrate: {baudrate}\n'
            f'   Velocidad por defecto: {self.default_speed}%'
        )

    def _start_reader(self):
        """Thread para leer respuestas del Arduino"""
        def reader():
            while rclpy.ok():
                try:
                    if self.ser.in_waiting > 0:
                        response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            self.get_logger().info(f'🤖 Arduino: {response}')
                except Exception as e:
                    if "Bad file descriptor" not in str(e):
                        self.get_logger().error(f'Error leyendo UART: {e}')
                    time.sleep(0.1)
        
        thread = threading.Thread(target=reader, daemon=True)
        thread.start()

    def _send_to_arduino(self, command):
        """Envía comando al Arduino"""
        try:
            full_command = f"{command}\n"
            self.ser.write(full_command.encode())
            self.ser.flush()
            self.get_logger().debug(f'📤 Enviado: {command}')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Error enviando: {e}')
            return False

    def command_callback(self, msg):
        """
        Recibe comandos de movimiento y los envía al Arduino
        
        Formato esperado: "DIRECTION:ACTION[:SPEED]"
        Ejemplos:
            - "FORWARD:PRESS"
            - "LEFT:PRESS:70"
            - "FORWARD:RELEASE"
        """
        try:
            parts = msg.data.split(':')
            
            if len(parts) < 2:
                self.get_logger().error(f'Formato inválido: {msg.data}')
                return
            
            direction = parts[0].upper()
            action = parts[1].upper()
            speed = int(parts[2]) if len(parts) > 2 else self.default_speed
            
            # Mapear direcciones a comandos del Arduino
            direction_map = {
                'FORWARD': 'FORWARD',
                'BACKWARD': 'BACKWARD',
                'LEFT': 'LEFT',
                'RIGHT': 'RIGHT',
                'CW': 'CW',
                'CCW': 'CCW',
                'FORWARD_LEFT': 'FORWARD_LEFT',
                'FORWARD_RIGHT': 'FORWARD_RIGHT',
                'BACKWARD_LEFT': 'BACKWARD_LEFT',
                'BACKWARD_RIGHT': 'BACKWARD_RIGHT',
                'STOP':'STOP',
            }
            
            if direction not in direction_map:
                self.get_logger().warn(f'Dirección desconocida: {direction}')
                return
            
            # Procesar acción
            if action == 'PRESS':
                # Iniciar movimiento
                arduino_cmd = f"MOVE:{direction_map[direction]}:{speed}"
                self._send_to_arduino(arduino_cmd)
                self.get_logger().info(f'▶️ Movimiento: {direction} @ {speed}%')
                
            elif action == 'RELEASE':
                # Detener
                arduino_cmd = "MOVE:STOP:0"
                self._send_to_arduino(arduino_cmd)
                self.get_logger().info('⏹️ Detenido')
            
            else:
                self.get_logger().warn(f'Acción desconocida: {action}')
                
        except Exception as e:
            self.get_logger().error(f'Error procesando comando: {e}')

    def destroy_node(self):
        """Cleanup al cerrar"""
        self.get_logger().info('🛑 Cerrando nodo...')
        
        # Detener motores
        self._send_to_arduino('MOVE:STOP:0')
        time.sleep(0.5)
        
        # Cerrar puerto serial
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorSimpleNode()
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
