# Eventos de monitoreo para Socket.IO Client
from library_opi.audio_player import AudioPlayer

class Monitoreo_RobotEvents:
    
    def register(self, sio, node):
        """
        Registra todos los eventos de monitoreo del robot
        
        Args:
            sio: Cliente de socketio
            node: Nodo de ROS2 
        """
        audio = AudioPlayer()
        
        # Iniciar proceso de navegación
        @sio.on("start-process")
        def handle_start(data):
            node.get_logger().info("🚀 Iniciando proceso...")
            # Responder al servidor con el estado
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
            if not audio.is_playing_audio():
                success = audio.play("/home/orangepi/sound_ia/resource/desalojo.mp3", blocking=False)

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
            node.get_logger().info("PARO DE EMERGENCIA ACTIVADO")
            # Responder con confirmación
            sio.emit("stop-all-now", {"status": "stop", "success": True})
            # Aquí detener todos los motores/procesos del robot
            audio.stop()
        
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