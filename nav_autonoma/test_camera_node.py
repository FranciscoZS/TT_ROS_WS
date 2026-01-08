#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from library_opi.camera_usb import USBCamera

class TestCameraNode(Node):
    def __init__(self):
        super().__init__('test_camera_node')
        
        # Parámetros configurables
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('use_compressed', True)  # Usar CompressedImage por defecto
        
        # Obtener parámetros
        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        publish_rate = self.get_parameter('publish_rate').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
        # Inicializar cámara
        self.get_logger().info('🎥 Inicializando cámara USB...')
        self.camera = USBCamera(
            device_id=device_id,
            width=width,
            height=height,
            fps=fps,
            node=self
        )
        
        if not self.camera.is_opened:
            self.get_logger().error('❌ Fallo al inicializar la cámara. Cerrando nodo.')
            raise RuntimeError('No se pudo inicializar la cámara')
        
        # Bridge de OpenCV a ROS
        self.bridge = CvBridge()
        
        # Publishers
        if self.use_compressed:
            self.image_pub = self.create_publisher(
                CompressedImage, 
                'camera/image/compressed', 
                10
            )
            self.get_logger().info('📡 Publisher: /camera/image/compressed (CompressedImage)')
        else:
            self.image_pub = self.create_publisher(
                Image, 
                'camera/image_raw', 
                10
            )
            self.get_logger().info('📡 Publisher: /camera/image_raw (Image)')
        
        # Publisher para información de cámara
        self.info_pub = self.create_publisher(String, 'camera/info', 10)
        
        # Timer para captura y publicación
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        
        # Contadores para estadísticas
        self.frame_count = 0
        self.error_count = 0
        
        # Timer para estadísticas cada 5 segundos
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        self.get_logger().info(
            f'✅ Nodo de prueba de cámara inicializado\n'
            f'   📹 Device: {device_id} | Resolución: {width}x{height}\n'
            f'   🔄 Tasa de publicación: {publish_rate} Hz\n'
            f'   📦 Modo: {"Comprimido" if self.use_compressed else "Raw"}'
        )

    def capture_and_publish(self):
        """Captura un frame y lo publica en ROS2"""
        try:
            # Leer frame de la cámara
            if not self.camera.read_frame():
                self.error_count += 1
                return
            
            # Obtener frame
            frame = self.camera.get_frame()
            
            if frame is None:
                self.error_count += 1
                return
            
            # Crear timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Publicar según el modo configurado
            if self.use_compressed:
                self.publish_compressed(frame, timestamp)
            else:
                self.publish_raw(frame, timestamp)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en captura/publicación: {e}')
            self.error_count += 1

    def publish_raw(self, frame, timestamp):
        """Publica imagen raw sin comprimir"""
        try:
            # Convertir OpenCV (BGR) a ROS Image (RGB)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = 'camera_link'
            
            self.image_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando imagen raw: {e}')

    def publish_compressed(self, frame, timestamp):
        """Publica imagen comprimida (JPEG)"""
        try:
            # Crear mensaje CompressedImage
            msg = CompressedImage()
            msg.header.stamp = timestamp
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            
            # Codificar a JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            success, encoded = cv2.imencode('.jpg', frame, encode_param)
            
            if success:
                msg.data = encoded.tobytes()
                self.image_pub.publish(msg)
            else:
                self.get_logger().error('❌ Error codificando imagen JPEG')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando imagen comprimida: {e}')

    def publish_statistics(self):
        """Publica estadísticas de la cámara"""
        try:
            width, height = self.camera.get_resolution()
            
            info_msg = String()
            info_msg.data = (
                f"Frames: {self.frame_count} | "
                f"Errores: {self.error_count} | "
                f"Resolución: {width}x{height} | "
                f"Tasa éxito: {(self.frame_count/(self.frame_count+self.error_count)*100):.1f}%"
            )
            
            self.info_pub.publish(info_msg)
            
            self.get_logger().info(
                f'📊 Estadísticas - Frames: {self.frame_count} | '
                f'Errores: {self.error_count} | '
                f'Res: {width}x{height}'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en estadísticas: {e}')

    def destroy_node(self):
        """Cleanup al cerrar el nodo"""
        self.get_logger().info('🛑 Cerrando nodo de cámara...')
        
        # Liberar cámara
        self.camera.cleanup()
        
        # Mostrar estadísticas finales
        if self.frame_count > 0:
            self.get_logger().info(
                f'📈 Estadísticas finales:\n'
                f'   Total frames: {self.frame_count}\n'
                f'   Total errores: {self.error_count}\n'
                f'   Tasa de éxito: {(self.frame_count/(self.frame_count+self.error_count)*100):.1f}%'
            )
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'❌ Error crítico: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
