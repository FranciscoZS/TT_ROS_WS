"""
Módulo simplificado de Socket.IO para control manual del robot
Solo maneja comandos de movimiento
"""
from std_msgs.msg import String

class ControlSimpleEvents:
    """Clase simplificada para control manual"""
    
    def __init__(self):
        self.motor_pub = None
    
    def register(self, sio, node):
        """
        Registra eventos de control manual
        
        Args:
            sio: Cliente de socketio
            node: Nodo de ROS2
        """
        
        # Crear publisher para comandos de motor
        self.motor_pub = node.create_publisher(
            String,
            'motor_command',
            10
        )
        
        node.get_logger().info("📡 Publisher de control simple creado")
        
        # ========================================
        # CONTROL MANUAL
        # ========================================
        @sio.on('on-manual_control')
        def handle_control(data):
            """
            Maneja comandos de control del joystick
            
            Formato de data:
                {
                    'direction': 'go-forward',
                    'action': 'press',
                    'timestamp': 1234567890
                }
            """
            try:
                direction = data.get('direction', '')
                action = data.get('action', '')
                
                # Limpiar dirección (quitar 'go-')
                clean_direction = direction.replace('go-', '').upper()
                
                # Mapear direcciones de la app a comandos del robot
                direction_map = {
                    'FORWARD': 'FORWARD',
                    'BACKWARD': 'BACKWARD',
                    'LEFT': 'LEFT',
                    'RIGHT': 'RIGHT',
                    'FORWARD_LEFT': 'FORWARD_LEFT',
                    'FORWARD_RIGHT': 'FORWARD_RIGHT',
                    'BACKWARD_LEFT': 'BACKWARD_LEFT',
                    'BACKWARD_RIGHT': 'BACKWARD_RIGHT',
                    'CW': 'CW',
                    'CCW': 'CCW',
                    'STOP': 'STOP',
                }
                
                # Convertir dirección
                if clean_direction in direction_map:
                    robot_direction = direction_map[clean_direction]
                else:
                    node.get_logger().warn(f'Dirección desconocida: {clean_direction}')
                    return
                
                # Crear comando para el nodo de motores
                # Formato: "DIRECTION:ACTION"
                command = f"{robot_direction}:{action.upper()}"
                
                # Publicar
                msg = String()
                msg.data = command
                self.motor_pub.publish(msg)
                
                # Log
                if action == 'press':
                    node.get_logger().info(f'▶️ {robot_direction}')
                else:
                    node.get_logger().info('⏹️ STOP')
                
                # Responder al servidor
                sio.emit('on-control_response', {
                    'status': 'success',
                    'direction': clean_direction,
                    'action': action
                })
                
            except Exception as e:
                node.get_logger().error(f'❌ Error en control: {e}')
                sio.emit('on-control_response', {
                    'status': 'error',
                    'message': str(e)
                })
        
        # ========================================
        # PARO DE EMERGENCIA
        # ========================================
        @sio.on('stop-all')
        def handle_stop(data):
            """Detiene inmediatamente el robot"""
            node.get_logger().error('🚨 PARO DE EMERGENCIA')
            
            # Enviar STOP
            msg = String()
            msg.data = 'FORWARD:RELEASE'  # Cualquier release detiene
            self.motor_pub.publish(msg)
            
            sio.emit('stop-all-now', {
                'status': 'success',
                'message': 'Robot detenido'
            })
        
        node.get_logger().info("✅ Eventos de control simple registrados")


