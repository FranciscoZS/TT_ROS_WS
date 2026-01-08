#!/usr/bin/env python3
"""
Nodo TauLidar con display directo usando OpenCV
Máximo rendimiento para visualización local
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import threading
import time

from library_opi.taulidar_camera import TauLidarCamera
from TauLidarCommon.frame import FrameType

class TestTauLidarDisplayNode(Node):
    def __init__(self):
        super().__init__('test_taulidar_display_node')
        
        # Parámetros
        self.declare_parameter('serial_port', 'None')
        self.declare_parameter('min_range', 0)
        self.declare_parameter('max_range', 4500)
        self.declare_parameter('integration_time', 1000)
        self.declare_parameter('min_amplitude', 10)
        self.declare_parameter('upscale', 4)
        self.declare_parameter('show_display', True)
        self.declare_parameter('publish_compressed', False)  # Desactivado por defecto
        
        serial_port = self.get_parameter('serial_port').value
        if serial_port == 'None':
            serial_port = None
            
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value
        integration_time = self.get_parameter('integration_time').value
        min_amplitude = self.get_parameter('min_amplitude').value
        self.upscale = self.get_parameter('upscale').value
        self.show_display = self.get_parameter('show_display').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        
        # Inicializar TauLidar
        self.get_logger().info('🚀 Inicializando TauLidar con display...')
        self.lidar = TauLidarCamera(
            serial_port=serial_port,
            min_range=min_range,
            max_range=max_range,
            integration_time=integration_time,
            min_amplitude=min_amplitude,
            node=self
        )
        
        if not self.lidar.is_opened:
            raise RuntimeError('No se pudo inicializar TauLidar')
        
        # Publishers (opcional)
        if self.publish_compressed:
            self.depth_pub = self.create_publisher(
                CompressedImage,
                'taulidar/depth/compressed',
                10
            )
            self.get_logger().info('📡 Publisher habilitado')
        
        self.info_pub = self.create_publisher(String, 'taulidar/info', 10)
        
        # Variables de control
        self.frame_count = 0
        self.display_running = False
        self.capture_thread = None
        
        # Timer para estadísticas
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        # Iniciar thread de display
        if self.show_display:
            self.display_running = True
            self.capture_thread = threading.Thread(target=self.display_loop, daemon=True)
            self.capture_thread.start()
            self.get_logger().info('🖥️  Display OpenCV habilitado')
        
        info = self.lidar.get_info()
        self.get_logger().info(
            f'✅ Nodo inicializado\n'
            f'   Modelo: {info.get("model", "N/A")}\n'
            f'   Rango: {min_range}-{max_range} mm\n'
            f'   Upscale: {self.upscale}x'
        )

    def display_loop(self):
        """Loop de display en thread separado"""
        window_name = 'TauLidar Depth Map - Press Q to quit | ESC to exit'
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info('🎬 Display loop iniciado')
        
        fps_counter = 0
        start_time = time.time()
        fps_real = 0.0
        
        while self.display_running:
            if self.lidar.read_frame(FrameType.DISTANCE):
                depth_rgb = self.lidar.get_depth_rgb(upscale=self.upscale)
                
                if depth_rgb is not None:
                    fps_counter += 1
                    self.frame_count += 1
                    
                    # Calcular FPS real
                    elapsed = time.time() - start_time
                    if elapsed >= 1.0:
                        fps_real = fps_counter / elapsed
                        fps_counter = 0
                        start_time = time.time()
                    
                    # Agregar información al frame
                    h, w = depth_rgb.shape[:2]
                    info = self.lidar.get_info()
                    
                    cv2.putText(depth_rgb, f'FPS: {fps_real:.1f}', (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(depth_rgb, f'Res: {w}x{h}', (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(depth_rgb, f'Range: {info.get("min_range")}-{info.get("max_range")} mm', 
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(depth_rgb, f'Frames: {self.frame_count}', (10, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Mostrar
                    cv2.imshow(window_name, depth_rgb)
                    
                    # Publicar si está habilitado
                    if self.publish_compressed:
                        self.publish_compressed_image(depth_rgb)
                    
                    # Salir con 'q' o ESC
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:
                        self.get_logger().info('👋 Cerrando display...')
                        self.display_running = False
                        break
        
        cv2.destroyAllWindows()
        self.get_logger().info('🛑 Display loop terminado')

    def publish_compressed_image(self, depth_rgb):
        """Publica imagen comprimida"""
        try:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'taulidar_link'
            msg.format = 'jpeg'
            
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
            success, encoded = cv2.imencode('.jpg', depth_rgb, encode_param)
            
            if success:
                msg.data = encoded.tobytes()
                self.depth_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando: {e}')

    def publish_statistics(self):
        """Publicar estadísticas"""
        try:
            width, height = self.lidar.get_resolution()
            info = self.lidar.get_info()
            
            info_msg = String()
            info_msg.data = f"Frames: {self.frame_count} | Res: {width}x{height} (x{self.upscale})"
            self.info_pub.publish(info_msg)
            
            self.get_logger().info(f'📊 Total frames: {self.frame_count}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en estadísticas: {e}')

    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('🛑 Cerrando nodo...')
        
        # Detener display
        self.display_running = False
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=2.0)
        
        # Liberar TauLidar
        self.lidar.cleanup()
        
        self.get_logger().info(f'📈 Total frames capturados: {self.frame_count}')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestTauLidarDisplayNode()
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
