class ControlManual_RobotEvents:
    """Clase para manejar eventos de control manual del robot"""
    
    def __init__(self):
        """Inicializa variables de estado"""
        self.direction = None
        self.action = None
        self.timestamp = None
    
    def register(self, sio, node):
        """
        Registra los eventos del control manual del robot
        
        Args:
            sio: Cliente de socketio
            node: Nodo de ROS2 para logging
        """
        
        # ========================================
        # ENTRADA DE CONTROL MANUAL
        # ========================================
        @sio.on('on-manual_control')
        def handle_manual_control(data):
            """
            Recibe comandos de control manual del joystick
            """
            try:
                # Extraer datos del evento
                self.direction = data.get('direction')
                self.action = data.get('action')
                self.timestamp = data.get('timestamp')
                
                # Usar logger de ROS2 en lugar de print
                node.get_logger().info(
                    f"📡 Control recibido: {self.action.upper()} - Dirección: {self.direction}"
                )
                
                # ========================================
                # LÓGICA DE CONTROL DEL ROBOT
                # ========================================
                
                if self.action == 'press':
                    node.get_logger().info(f"▶️ Iniciando movimiento: {self.direction}")
                    
                    # Manejar diferentes direcciones
                    if self.direction == 'go-forward':
                        node.get_logger().info("🤖 Robot moviéndose hacia adelante")
                    elif self.direction == 'go-backward':
                        node.get_logger().info("🤖 Robot moviéndose hacia atrás")
                    elif self.direction == 'go-left':
                        node.get_logger().info("🤖 Robot girando a la izquierda")
                    elif self.direction == 'go-right':
                        node.get_logger().info("🤖 Robot girando a la derecha")
                    elif self.direction == 'go-forward_left':
                        node.get_logger().info("🤖 Robot moviéndose adelante-izquierda")
                    elif self.direction == 'go-forward_right':
                        node.get_logger().info("🤖 Robot moviéndose adelante-derecha")
                    elif self.direction == 'go-backward_left':
                        node.get_logger().info("🤖 Robot moviéndose atrás-izquierda")
                    elif self.direction == 'go-backward_right':
                        node.get_logger().info("🤖 Robot moviéndose atrás-derecha")
                    else:
                        node.get_logger().warn(f"⚠️ Dirección desconocida: {self.direction}")
                    
                    # Emitir respuesta
                    sio.emit('on-control_response', {
                        'status': 'success',
                        'message': f'Movimiento iniciado: {self.direction}',
                        'direction': self.direction,  # Corregido: sin self. (era self.self.direction)
                        'action': self.action,
                        'timestamp': self.timestamp
                    })
                
                elif self.action == 'release':
                    node.get_logger().info(f"⏹️ Deteniendo movimiento: {self.direction}")
                    
                    # Aquí iría la lógica para detener el robot
                    node.get_logger().info("🤖 Robot detenido")
                    
                    # Emitir respuesta
                    sio.emit('on-control_response', {
                        'status': 'success',
                        'message': 'Robot detenido',
                        'direction': self.direction,  # Corregido: sin self.
                        'action': self.action,
                        'timestamp': self.timestamp
                    })
                
                else:
                    node.get_logger().warn(f"⚠️ Acción desconocida: {self.action}")
                    sio.emit('on-control_response', {
                        'status': 'error',
                        'message': f'Acción no reconocida: {self.action}',
                        'timestamp': self.timestamp
                    })
                    
            except Exception as e:
                node.get_logger().error(f"❌ Error en handle_manual_control: {e}")
                sio.emit('on-control_response', {
                    'status': 'error',
                    'message': f'Error interno: {str(e)}'
                })
        
        # ========================================
        # MANEJADOR PARA EL BOTÓN DE BANDERA
        # ========================================
        @sio.on('get-mark_waypoint')
        def handle_mark_waypoint():
            """
            Maneja la marcación de waypoints
            """
            node.get_logger().info(f"Marcando waypoint :happy")