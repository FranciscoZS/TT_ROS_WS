#!/usr/bin/env python3
"""
Nodo de cámara optimizado con display directo usando OpenCV
Para máximo rendimiento en visualización local
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import threading

from library_opi.camera_usb import USBCamera

class TestCameraDisplayNode(Node):
    def __init__(self):
        super().__init__('test_camera_display_node')
        
        # Parámetros
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('show_display', True)  # Mostrar ventana OpenCV
        self.declare_parameter('publish_compressed', True)  # Publicar en ROS
        
        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        publish_rate = self.get_parameter('publish_rate').value
        self.show_display = self.get_parameter('show_display').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        
        # Inicializar cámara
        self.get_logger().info('🎥 Inicializando cámara USB optimizada...')
        self.camera = USBCamera(
            device_id=device_id,
            width=width,
            height=height,
            fps=fps,
            node=self
        )
        
        if not self.camera.is_opened:
            raise RuntimeError('No se pudo inicializar la cámara')
        
        # Publisher (opcional)
        if self.publish_compressed:
            self.image_pub = self.create_publisher(
                CompressedImage, 
                'camera/image/compressed', 
                10
            )
            self.get_logger().info('📡 Publisher habilitado: /camera/image/compressed')
        
        self.info_pub = self.create_publisher(String, 'camera/info', 10)
        
        # Variables de control
        self.frame_count = 0
        self.display_running = False
        self.capture_thread = None
        
        # Timer para captura y publicación
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.capture_callback)
        
        # Timer para estadísticas
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        # Iniciar thread de display si está habilitado
        if self.show_display:
            self.display_running = True
            self.capture_thread = threading.Thread(target=self.display_loop, daemon=True)
            self.capture_thread.start()
            self.get_logger().info('🖥️  Display OpenCV habilitado')
        
        self.get_logger().info(
            f'✅ Nodo inicializado | {width}x{height} @ {publish_rate}Hz'
        )

    def capture_callback(self):
        """Captura frame y publica (solo si publish está habilitado)"""
        if not self.publish_compressed:
            return
        
        try:
            if self.camera.read_frame():
                frame = self.camera.get_frame()
                if frame is not None:
                    # Publicar comprimido
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_link'
                    msg.format = 'jpeg'
                    
                    # Codificar JPEG
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                    success, encoded = cv2.imencode('.jpg', frame, encode_param)
                    
                    if success:
                        msg.data = encoded.tobytes()
                        self.image_pub.publish(msg)
                        self.frame_count += 1
                    
        except Exception as e:
            self.get_logger().error(f'❌ Error en captura: {e}')

    def display_loop(self):
        """Loop de display en thread separado para máximo rendimiento"""
        window_name = 'Camera Feed - Press Q to quit'
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info('🎬 Display loop iniciado')
        
        fps_counter = 0
        import time
        start_time = time.time()
        
        while self.display_running:
            if self.camera.read_frame():
                frame = self.camera.get_frame()
                
                if frame is not None:
                    fps_counter += 1
                    
                    # Calcular FPS real
                    elapsed = time.time() - start_time
                    if elapsed >= 1.0:
                        fps_real = fps_counter / elapsed
                        fps_counter = 0
                        start_time = time.time()
                    else:
                        fps_real = fps_counter / max(elapsed, 0.001)
                    
                    # Agregar información al frame
                    h, w = frame.shape[:2]
                    cv2.putText(frame, f'FPS: {fps_real:.1f}', (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f'Res: {w}x{h}', (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f'Frames: {self.frame_count}', (10, 90),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Mostrar
                    cv2.imshow(window_name, frame)
                    
                    # Salir con 'q'
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.get_logger().info('👋 Cerrando display...')
                        self.display_running = False
                        break
        
        cv2.destroyAllWindows()
        self.get_logger().info('🛑 Display loop terminado')

    def publish_statistics(self):
        """Publicar estadísticas"""
        try:
            width, height = self.camera.get_resolution()
            info_msg = String()
            info_msg.data = f"Frames: {self.frame_count} | Res: {width}x{height}"
            self.info_pub.publish(info_msg)
            
            self.get_logger().info(f'📊 Total frames publicados: {self.frame_count}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en estadísticas: {e}')

    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('🛑 Cerrando nodo...')
        
        # Detener display
        self.display_running = False
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=2.0)
        
        # Liberar cámara
        self.camera.cleanup()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestCameraDisplayNode()
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
