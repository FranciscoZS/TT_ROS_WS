# Eventos de monitoreo para Socket.IO Client
from library_opi.audio_player import AudioPlayer
from std_msgs.msg import String, Bool

class Monitoreo_RobotEvents:


    def __init__(self):
        # ==================== PARÁMETROS DE AUDIO ====================
        self.declare_parameter('audio_file', '/home/orangepi/sound_ia/resource/desalojo.mp3')
        self.declare_parameter('audio_device', 'plughw:3,0')
        self.declare_parameter('trigger_cooldown', 5.0)  # Segundos entre triggers

        self.audio_file = self.get_parameter('audio_file').value
        audio_device = self.get_parameter('audio_device').value
        self.trigger_cooldown = self.get_parameter('trigger_cooldown').value

        # Inicializar audio
        self.get_logger().info('🔊 Inicializando audio...')
        self.audio = AudioPlayer(audio_device, node=self)
    

    def _trigger_audio(self):
        """Activa la reproducción de audio"""
        try:
            # Solo reproducir si no hay audio en curso
            if not self.audio.is_playing_audio():
                success = self.audio.play(self.audio_file, blocking=False)
                
                if success:
                    self.trigger_count += 1
                    self.last_trigger_time = time.time()
                    
                    # Publicar evento de trigger
                    trigger_msg = Bool()
                    trigger_msg.data = True
                    self.trigger_pub.publish(trigger_msg)
                    
                    self.get_logger().info(
                        f'🔔 Audio activado! (Trigger #{self.trigger_count})'
                    )
        except Exception as e:
            self.get_logger().error(f'❌ Error activando audio: {e}')


    def register(self, sio, node):
        """
        Registra todos los eventos de monitoreo del robot
        
        Args:
            sio: Cliente de socketio
            node: Nodo de ROS2 
        """

        @sio.on("ejec-sound-alert")
        def handle_sound_alert(data):
            node.get_logger().info("🔔 Alerta de sonido recibida")
            self._trigger_audio()
        
        # Iniciar proceso de navegación
        @sio.on("start-process")
        def handle_start(data):
            node.get_logger().info("🚀 Iniciando proceso...")
            # Responder al servidor con el estado
            sio.emit("go-robot", {"status": "go", "success": True})
            # Aquí puedes publicar a un topic ROS2 para iniciar el proceso
            # ejemplo: self.pub_start.publish(String(data="start"))
        
        
        # Inicia Proceso de regreso a casa
        @sio.on("go-home")
        def handle_home(data):
            node.get_logger().info("🏠 Regresando a casa...")
            # Responder con confirmación
            sio.emit("go-home", {"status": "home", "success": True})
            # Aquí publicar a topic de navegación para ir a home
        
        
        # Toogle a para permitir conmutacion de lamparas UVC
        @sio.on("toggle-LampsUVC")
        def handle_toggle_lamps(data):
            # Aquí debes leer el estado real de las lámparas
            # Por ahora simulamos el toggle (deberías guardar el estado)
            node.get_logger().info("Conmutando lámparas UVC...")
            
            # TODO: Implementar lógica real de toggle
            # status_lamps = not self.status_lamps  
            # msg_lamps = "on" if status_lamps else "off"            
            sio.emit("uvc-status", {"status": "on", "success": True})
            node.get_logger().info("Lámparas UVC activadas")
        
        
        # Paro de emergencia general
        @sio.on("stop-all")
        def handle_stop(data):
            node.get_logger().error("PARO DE EMERGENCIA ACTIVADO")
            # Responder con confirmación
            sio.emit("stop-all-now", {"status": "stop", "success": True})
            # Aquí detener todos los motores/procesos del robot
        
        
        # Solicitamos datos del robot
        @sio.on("solicitar-datos")
        def handle_solicitar_datos(data):
            node.get_logger().info("📊 Solicitud de datos recibida")
            
            # TODO: Obtener datos reales desde topics de ROS2
            # Por ahora simulamos (como en tu código original)
            import random
            bateria = random.randint(20, 100)
            co2 = random.randint(300, 600)
            
            sio.emit("datos-bateria", {"battery": bateria, "success": True})
            sio.emit("datos-co2", {"co2": co2, "success": True})
            
            node.get_logger().info(f"📤 Batería: {bateria}%, CO2: {co2}ppm")
        
        
        node.get_logger().info("✅ Eventos de monitoreo registrados correctamente")