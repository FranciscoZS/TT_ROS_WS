#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from library_opi.audio_player import AudioPlayer

class TestAudioNode(Node):
    def __init__(self):
        super().__init__('test_audio_node')
        
        # Parámetros
        self.declare_parameter('audio_file', '/home/orangepi/sound_ia/resource/desalojo.mp3')
        self.declare_parameter('audio_device', 'plughw:3,0')
        self.declare_parameter('loop', False)
        self.declare_parameter('volume', 80)
        
        audio_file = self.get_parameter('audio_file').value
        audio_device = self.get_parameter('audio_device').value
        loop = self.get_parameter('loop').value
        volume = self.get_parameter('volume').value
        
        # Inicializar reproductor
        self.audio = AudioPlayer(audio_device=audio_device, node=self)
        
        # Ajustar volumen
        if volume != 80:
            self.audio.set_volume(volume)
        
        # Publisher de estado
        self.status_pub = self.create_publisher(String, 'audio/status', 10)
        
        # Timer para verificar estado
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Reproducir audio
        self.get_logger().info(f'🎵 Iniciando reproducción de: {audio_file}')
        
        success = self.audio.play(audio_file, blocking=False, loop=loop)
        
        if success:
            mode = "loop continuo" if loop else "una vez"
            self.get_logger().info(f'✅ Reproducción iniciada ({mode})')
        else:
            self.get_logger().error('❌ Error al iniciar reproducción')
        
        self.get_logger().info('💡 Presiona Ctrl+C para detener')

    def publish_status(self):
        """Publica el estado de reproducción"""
        status_msg = String()
        status = "🔊 Reproduciendo" if self.audio.is_playing_audio() else "🔇 Detenido"
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('🛑 Deteniendo reproducción...')
        self.audio.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestAudioNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()