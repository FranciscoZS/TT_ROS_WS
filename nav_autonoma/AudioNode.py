
# nav_autonoma/nav_autonoma/audio_manager_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from library_opi.audio_player import AudioPlayer
from nav_autonoma.configRobot import Config
import time


class AudioManagerNode(Node):
    """Nodo que maneja todo lo relacionado con audio"""
    
    def __init__(self):
        super().__init__('audio_manager_node')
        
        # ==================== PARÁMETROS ====================
        self.declare_parameter('audio_file', Config.AUDIO_FILE)
        self.declare_parameter('audio_device', Config.AUDIO_DEVICE)
        self.declare_parameter('trigger_cooldown', Config.AUDIO_TRIGGER_COOLDOWN)
        
        self.audio_file = self.get_parameter('audio_file').value
        audio_device = self.get_parameter('audio_device').value
        self.trigger_cooldown = self.get_parameter('trigger_cooldown').value
        
        # ==================== INICIALIZAR AUDIO ====================
        self.get_logger().info('🔊 Inicializando audio...')
        self.audio = AudioPlayer(audio_device, node=self)
        
        # ==================== PUBLISHERS ====================
        self.status_pub = self.create_publisher(
            Bool,
            Config.TOPIC_AUDIO_STATUS,
            10
        )
        
        # ==================== SUBSCRIBERS ====================
        self.trigger_sub = self.create_subscription(
            String,
            Config.TOPIC_AUDIO_TRIGGER,
            self.trigger_callback,
            10
        )
        
        # ==================== ESTADO ====================
        self.trigger_count = 0
        self.last_trigger_time = 0
        
        self.get_logger().info('✅ Audio Manager Node iniciado')
    
    def trigger_callback(self, msg):
        """
        Callback que se ejecuta cuando alguien publica en /audio/trigger
        
        Puedes enviar:
        - "play" - reproducir audio por defecto
        - "stop" - detener audio
        - "ruta/al/archivo.mp3" - reproducir archivo específico
        """
        command = msg.data.lower()
        
        if command == "play":
            self.play_audio()
        elif command == "stop":
            self.stop_audio()
        else:
            # Asumir que es una ruta de archivo
            self.play_audio(command)
    
    def play_audio(self, audio_file=None):
        """Reproduce audio con cooldown"""
        
        # Verificar cooldown
        time_since_last = time.time() - self.last_trigger_time
        if time_since_last < self.trigger_cooldown:
            remaining = self.trigger_cooldown - time_since_last
            self.get_logger().warn(
                f'⏳ Cooldown activo. Espera {remaining:.1f}s más'
            )
            return
        
        # Verificar si ya hay audio reproduciéndose
        if self.audio.is_playing_audio():
            self.get_logger().warn('⚠️ Ya hay audio reproduciéndose')
            return
        
        try:
            file_to_play = audio_file or self.audio_file
            success = self.audio.play(file_to_play, blocking=False)
            
            if success:
                self.trigger_count += 1
                self.last_trigger_time = time.time()
                
                # Publicar estado
                status_msg = Bool()
                status_msg.data = True
                self.status_pub.publish(status_msg)
                
                self.get_logger().info(
                    f'🔔 Audio activado! (#{self.trigger_count}): {file_to_play}'
                )
            else:
                self.get_logger().error('❌ Fallo al reproducir audio')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error reproduciendo audio: {e}')
    
    def stop_audio(self):
        """Detiene la reproducción de audio"""
        try:
            self.audio.stop()
            self.get_logger().info('⏹️ Audio detenido')
            
            # Publicar estado
            status_msg = Bool()
            status_msg.data = False
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error deteniendo audio: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AudioManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()