#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, PointField
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

from library_opi.taulidar_camera import TauLidarCamera
from TauLidarCommon.frame import FrameType

class TestTauLidarNode(Node):
    def __init__(self):
        super().__init__('test_taulidar_node')
        
        # Parámetros configurables (mantener los existentes)
        self.declare_parameter('serial_port', 'None')
        self.declare_parameter('min_range', 0)
        self.declare_parameter('max_range', 4500)
        self.declare_parameter('integration_time', 1000)
        self.declare_parameter('min_amplitude', 10)
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('upscale', 4)
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('publish_pointcloud', True)  # Nuevo parámetro
        
        # Obtener parámetros (mantener los existentes)
        serial_port = self.get_parameter('serial_port').value
        if serial_port == 'None':
            serial_port = None
            
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value
        integration_time = self.get_parameter('integration_time').value
        min_amplitude = self.get_parameter('min_amplitude').value
        publish_rate = self.get_parameter('publish_rate').value
        self.upscale = self.get_parameter('upscale').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.publish_pointcloud = self.get_parameter('publish_pointcloud').value
        
        # Inicializar TauLidar (mantener existente)
        self.get_logger().info('🚀 Inicializando TauLidar...')
        self.lidar = TauLidarCamera(
            serial_port=serial_port,
            min_range=min_range,
            max_range=max_range,
            integration_time=integration_time,
            min_amplitude=min_amplitude,
            node=self
        )
        
        if not self.lidar.is_opened:
            self.get_logger().error('❌ Fallo al inicializar TauLidar. Cerrando nodo.')
            raise RuntimeError('No se pudo inicializar TauLidar')
        
        # Bridge de OpenCV a ROS
        self.bridge = CvBridge()
        
        # Publishers (mantener existentes y agregar pointcloud)
        if self.use_compressed:
            self.depth_pub = self.create_publisher(
                CompressedImage,
                'taulidar/depth/compressed',
                10
            )
            self.get_logger().info('📡 Publisher: /taulidar/depth/compressed')
        else:
            self.depth_pub = self.create_publisher(
                Image,
                'taulidar/depth/image_raw',
                10
            )
            self.get_logger().info('📡 Publisher: /taulidar/depth/image_raw')
        
        # Publisher para pointcloud
        if self.publish_pointcloud:
            self.pointcloud_pub = self.create_publisher(
                PointCloud2,
                'taulidar/pointcloud',
                10
            )
            self.get_logger().info('📡 Publisher: /taulidar/pointcloud')
        
        # Publisher para información
        self.info_pub = self.create_publisher(String, 'taulidar/info', 10)
        
# En test_taulidar_node.py - modifica el __init__
        # Timer para pointcloud (más lento que las imágenes)
        pointcloud_timer_period = 1.0 / publish_rate # 10 Hz en lugar de 60 Hz
        self.pointcloud_timer = self.create_timer(pointcloud_timer_period, self.publish_pointcloud_only)
        
        # Timer para imágenes (mantener a 60 Hz si quieres)
        image_timer_period = 1.0 / publish_rate
        self.image_timer = self.create_timer(image_timer_period, self.capture_and_publish_images)
        
        # Contadores
        self.frame_count = 0
        self.error_count = 0
        
        # Timer para estadísticas
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        # Mostrar información
        info = self.lidar.get_info()
        self.get_logger().info(
            f'✅ Nodo TauLidar inicializado\n'
            f'   📹 Modelo: {info.get("model", "N/A")}\n'
            f'   🔧 Firmware: {info.get("firmware", "N/A")}\n'
            f'   📐 Resolución: {info.get("resolution", "N/A")}\n'
            f'   📏 Rango: {min_range}-{max_range} mm\n'
            f'   🔄 Tasa de publicación: {publish_rate} Hz\n'
            f'   🔍 Upscale: {self.upscale}x\n'
            f'   ☁️  PointCloud: {"Activado" if self.publish_pointcloud else "Desactivado"}'
        )

    def publish_pointcloud_only(self):
        """Publica solo el pointcloud (llamado por timer separado)"""
        try:
            # Leer frame específicamente para pointcloud
            if self.lidar.read_frame(FrameType.DISTANCE_AMPLITUDE):
                timestamp = self.get_clock().now().to_msg()
                pointcloud_msg = self.lidar.get_pointcloud_msg(timestamp)
                if pointcloud_msg:
                    self.pointcloud_pub.publish(pointcloud_msg)
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando pointcloud: {e}')

    def capture_and_publish_images(self):
        """Captura y publica solo las imágenes (sin pointcloud)"""
        try:
            # Leer frame para imágenes
            if self.lidar.read_frame(FrameType.DISTANCE):
                timestamp = self.get_clock().now().to_msg()
                depth_rgb = self.lidar.get_depth_rgb(upscale=self.upscale)
                if depth_rgb is not None:
                    if self.use_compressed:
                        self.publish_compressed(depth_rgb, timestamp)
                    else:
                        self.publish_raw(depth_rgb, timestamp)
        except Exception as e:
            self.get_logger().error(f'❌ Error en captura/publicación de imágenes: {e}')

    def capture_and_publish(self):
        """Captura un frame y lo publica en ROS2"""
        try:
            # Leer frame del TauLidar - CAMBIAR A DISTANCE_AMPLITUDE para pointcloud
            if not self.lidar.read_frame(FrameType.DISTANCE_AMPLITUDE):
                self.error_count += 1
                return
            
            # Crear timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Publicar depth image (mantener existente)
            depth_rgb = self.lidar.get_depth_rgb(upscale=self.upscale)
            if depth_rgb is not None:
                if self.use_compressed:
                    self.publish_compressed(depth_rgb, timestamp)
                else:
                    self.publish_raw(depth_rgb, timestamp)
            
            # Publicar pointcloud si está activado
            if self.publish_pointcloud:
                pointcloud_msg = self.lidar.get_pointcloud_msg(timestamp)
                if pointcloud_msg:
                    self.pointcloud_pub.publish(pointcloud_msg)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en captura/publicación: {e}')
            self.error_count += 1


    def publish_raw(self, depth_rgb, timestamp):
        """Publica imagen raw sin comprimir"""
        try:
            # Convertir a formato BGR para ROS
            msg = self.bridge.cv2_to_imgmsg(depth_rgb, encoding='bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = 'taulidar_link'
            
            self.depth_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando imagen raw: {e}')

    def publish_compressed(self, depth_rgb, timestamp):
        """Publica imagen comprimida (JPEG)"""
        try:
            msg = CompressedImage()
            msg.header.stamp = timestamp
            msg.header.frame_id = 'taulidar_link'
            msg.format = 'jpeg'
            
            # Codificar a JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
            success, encoded = cv2.imencode('.jpg', depth_rgb, encode_param)
            
            if success:
                msg.data = encoded.tobytes()
                self.depth_pub.publish(msg)
            else:
                self.get_logger().error('❌ Error codificando imagen JPEG')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando imagen comprimida: {e}')

    def publish_statistics(self):
        """Publica estadísticas del TauLidar"""
        try:
            width, height = self.lidar.get_resolution()
            info = self.lidar.get_info()
            
            info_msg = String()
            info_msg.data = (
                f"Frames: {self.frame_count} | "
                f"Errores: {self.error_count} | "
                f"Resolución: {width}x{height} (x{self.upscale}) | "
                f"Modelo: {info.get('model', 'N/A')}"
            )
            
            self.info_pub.publish(info_msg)
            
            success_rate = (self.frame_count/(self.frame_count+self.error_count)*100) if (self.frame_count+self.error_count) > 0 else 0
            
            self.get_logger().info(
                f'📊 Estadísticas - Frames: {self.frame_count} | '
                f'Errores: {self.error_count} | '
                f'Éxito: {success_rate:.1f}%'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en estadísticas: {e}')

    def destroy_node(self):
        """Cleanup al cerrar el nodo"""
        self.get_logger().info('🛑 Cerrando nodo TauLidar...')
        
        # Liberar TauLidar
        self.lidar.cleanup()
        
        # Mostrar estadísticas finales
        if self.frame_count > 0:
            success_rate = (self.frame_count/(self.frame_count+self.error_count)*100)
            self.get_logger().info(
                f'📈 Estadísticas finales:\n'
                f'   Total frames: {self.frame_count}\n'
                f'   Total errores: {self.error_count}\n'
                f'   Tasa de éxito: {success_rate:.1f}%'
            )
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestTauLidarNode()
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

