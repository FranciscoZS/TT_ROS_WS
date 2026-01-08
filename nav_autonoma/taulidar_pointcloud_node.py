#!/usr/bin/env python3
"""
Nodo para publicar PointCloud2 desde TauLidar
Convierte la matriz de distancias a nube de puntos 3D
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

from library_opi.taulidar_camera import TauLidarCamera
from TauLidarCommon.frame import FrameType

class TauLidarPointCloudNode(Node):
    def __init__(self):
        super().__init__('taulidar_pointcloud_node')
        
        # Parámetros
        self.declare_parameter('serial_port', 'None')
        self.declare_parameter('min_range', 100)    # mm - filtrar puntos muy cercanos
        self.declare_parameter('max_range', 4500)   # mm
        self.declare_parameter('integration_time', 1000)
        self.declare_parameter('min_amplitude', 10)
        self.declare_parameter('publish_rate', 30.0)
        
        # Parámetros de cámara (FOV del TauLidar)
        # OnionTau tiene ~69° horizontal y ~51° vertical
        self.declare_parameter('fov_horizontal', 69.0)  # grados
        self.declare_parameter('fov_vertical', 51.0)    # grados
        
        serial_port = self.get_parameter('serial_port').value
        if serial_port == 'None':
            serial_port = None
            
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        integration_time = self.get_parameter('integration_time').value
        min_amplitude = self.get_parameter('min_amplitude').value
        publish_rate = self.get_parameter('publish_rate').value
        self.fov_h = np.radians(self.get_parameter('fov_horizontal').value)
        self.fov_v = np.radians(self.get_parameter('fov_vertical').value)
        
        # Inicializar TauLidar
        self.get_logger().info('🚀 Inicializando TauLidar para PointCloud...')
        self.lidar = TauLidarCamera(
            serial_port=serial_port,
            min_range=self.min_range,
            max_range=self.max_range,
            integration_time=integration_time,
            min_amplitude=min_amplitude,
            node=self
        )
        
        if not self.lidar.is_opened:
            raise RuntimeError('No se pudo inicializar TauLidar')
        
        # Obtener resolución
        self.width, self.height = self.lidar.get_resolution()
        self.get_logger().info(f'📐 Resolución: {self.width}x{self.height}')
        
        # Precalcular ángulos para cada píxel (optimización)
        self._precompute_angles()
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            'taulidar/pointcloud',
            10
        )
        
        # Timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_pointcloud)
        
        # Contadores
        self.frame_count = 0
        
        info = self.lidar.get_info()
        self.get_logger().info(
            f'✅ Nodo PointCloud inicializado\n'
            f'   Modelo: {info.get("model", "N/A")}\n'
            f'   Resolución: {self.width}x{self.height}\n'
            f'   Rango: {self.min_range}-{self.max_range} mm\n'
            f'   FOV: {np.degrees(self.fov_h):.1f}° x {np.degrees(self.fov_v):.1f}°\n'
            f'   Tasa: {publish_rate} Hz'
        )

    def _precompute_angles(self):
        """Precalcula los ángulos para cada píxel"""
        # Crear grillas de coordenadas de píxeles
        u = np.arange(self.width)
        v = np.arange(self.height)
        u_grid, v_grid = np.meshgrid(u, v)
        
        # Convertir coordenadas de píxel a ángulos
        # Centro de la imagen como referencia
        u_center = self.width / 2.0
        v_center = self.height / 2.0
        
        # Calcular ángulos horizontales y verticales
        self.theta = (u_grid - u_center) / self.width * self.fov_h
        self.phi = (v_grid - v_center) / self.height * self.fov_v
        
        self.get_logger().info('✅ Ángulos precalculados')

    def publish_pointcloud(self):
        """Captura frame y publica nube de puntos"""
        try:
            # Leer frame
            if not self.lidar.read_frame(FrameType.DISTANCE):
                return
            
            # Obtener matriz de distancias (en mm)
            distance_matrix = self.lidar.get_distance_matrix()
            
            if distance_matrix is None:
                return
            
            # Convertir a nube de puntos
            points = self._distance_to_pointcloud(distance_matrix)
            
            if points is None or len(points) == 0:
                self.get_logger().warn('⚠️ No se generaron puntos válidos')
                return
            
            # Crear mensaje PointCloud2
            msg = self._create_pointcloud2_msg(points)
            
            # Publicar
            self.pointcloud_pub.publish(msg)
            self.frame_count += 1
            
            if self.frame_count % 50 == 0:
                self.get_logger().info(
                    f'📊 Frame {self.frame_count} | Puntos: {len(points)}'
                )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando pointcloud: {e}')

    def _distance_to_pointcloud(self, distance_matrix):
        """
        Convierte matriz de distancias a coordenadas XYZ
        
        Args:
            distance_matrix: Matriz HxW con distancias en mm
        
        Returns:
            numpy.ndarray: Array Nx3 con puntos [x, y, z] en metros
        """
        # Convertir mm a metros
        distances = distance_matrix.astype(np.float32) / 1000.0
        
        # Filtrar por rango válido
        valid_mask = (distances >= self.min_range / 1000.0) & \
                     (distances <= self.max_range / 1000.0)
        
        # Calcular coordenadas 3D usando geometría esférica
        # Sistema de coordenadas:
        # X: hacia adelante (profundidad)
        # Y: hacia la izquierda
        # Z: hacia arriba
        
        # Proyección en el plano XY (horizontal)
        x = distances * np.cos(self.phi) * np.cos(self.theta)
        y = distances * np.cos(self.phi) * np.sin(self.theta)
        z = distances * np.sin(self.phi)
        
        # Aplicar máscara de validez
        x_valid = x[valid_mask]
        y_valid = y[valid_mask]
        z_valid = z[valid_mask]
        
        # Combinar en array Nx3
        points = np.column_stack((x_valid, y_valid, z_valid))
        
        return points

    def _create_pointcloud2_msg(self, points):
        """
        Crea mensaje PointCloud2 desde array de puntos
        
        Args:
            points: Array Nx3 con coordenadas XYZ
        
        Returns:
            PointCloud2: Mensaje ROS2
        """
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'taulidar_link'
        
        # Definir campos (X, Y, Z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Configurar dimensiones
        msg.height = 1  # Nube no organizada
        msg.width = len(points)
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True  # No hay puntos inválidos (ya filtrados)
        
        # Empaquetar datos
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))
        
        msg.data = b''.join(buffer)
        
        return msg

    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('🛑 Cerrando nodo PointCloud...')
        self.get_logger().info(f'📈 Total frames: {self.frame_count}')
        self.lidar.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TauLidarPointCloudNode()
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
