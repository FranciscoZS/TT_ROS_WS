#!/usr/bin/env python3
"""
Nodo para convertir PointCloud2 a LaserScan
Proyecta la nube de puntos 3D a un scan 2D para Cartographer
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import numpy as np
import struct

class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_node')
        
        # Parámetros
        self.declare_parameter('min_height', -0.2)  # metros - altura mínima de puntos
        self.declare_parameter('max_height', 0.3)   # metros - altura máxima de puntos
        self.declare_parameter('angle_min', -1.57)  # radianes (-90°)
        self.declare_parameter('angle_max', 1.57)   # radianes (+90°)
        self.declare_parameter('angle_increment', 0.017)  # ~1° en radianes
        self.declare_parameter('range_min', 0.1)    # metros
        self.declare_parameter('range_max', 4.5)    # metros
        self.declare_parameter('scan_time', 0.1)    # segundos
        
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_time = self.get_parameter('scan_time').value
        
        # Calcular número de rayos
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        # Subscriber a PointCloud
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'taulidar/pointcloud',
            self.pointcloud_callback,
            10
        )
        
        # Publisher de LaserScan
        self.scan_pub = self.create_publisher(
            LaserScan,
            'scan',
            10
        )
        
        # Contador
        self.scan_count = 0
        
        self.get_logger().info(
            f'✅ PointCloud → LaserScan inicializado\n'
            f'   Altura: {self.min_height} a {self.max_height} m\n'
            f'   Ángulo: {np.degrees(self.angle_min):.0f}° a {np.degrees(self.angle_max):.0f}°\n'
            f'   Resolución: {np.degrees(self.angle_increment):.2f}°\n'
            f'   Rayos: {self.num_ranges}'
        )

    def pointcloud_callback(self, msg):
        """Procesa PointCloud2 y publica LaserScan"""
        try:
            # Extraer puntos del mensaje
            points = self._extract_points(msg)
            
            if points is None or len(points) == 0:
                return
            
            # Filtrar por altura (proyección 2D)
            mask = (points[:, 2] >= self.min_height) & (points[:, 2] <= self.max_height)
            points_2d = points[mask]
            
            if len(points_2d) == 0:
                return
            
            # Convertir a LaserScan
            scan = self._points_to_scan(points_2d)
            
            # Configurar header
            scan.header.stamp = msg.header.stamp
            scan.header.frame_id = 'taulidar_link'
            
            # Publicar
            self.scan_pub.publish(scan)
            self.scan_count += 1
            
            if self.scan_count % 50 == 0:
                valid_ranges = np.sum(np.isfinite(scan.ranges))
                self.get_logger().info(
                    f'📡 Scan {self.scan_count} | Rayos válidos: {valid_ranges}/{self.num_ranges}'
                )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error convirtiendo: {e}')

    def _extract_points(self, msg):
        """
        Extrae coordenadas XYZ del mensaje PointCloud2
        
        Returns:
            numpy.ndarray: Array Nx3 con puntos [x, y, z]
        """
        try:
            # Determinar offsets de los campos
            x_offset = None
            y_offset = None
            z_offset = None
            
            for field in msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().error('❌ Campos XYZ no encontrados')
                return None
            
            # Desempaquetar datos
            points = []
            point_step = msg.point_step
            
            for i in range(msg.width):
                offset = i * point_step
                
                x = struct.unpack_from('f', msg.data, offset + x_offset)[0]
                y = struct.unpack_from('f', msg.data, offset + y_offset)[0]
                z = struct.unpack_from('f', msg.data, offset + z_offset)[0]
                
                points.append([x, y, z])
            
            return np.array(points, dtype=np.float32)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error extrayendo puntos: {e}')
            return None

    def _points_to_scan(self, points):
        """
        Convierte puntos 2D a LaserScan
        
        Args:
            points: Array Nx3 (pero solo usa X, Y)
        
        Returns:
            LaserScan: Mensaje de scan
        """
        scan = LaserScan()
        
        # Configurar parámetros del scan
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Inicializar rangos con inf (sin detección)
        ranges = np.full(self.num_ranges, np.inf, dtype=np.float32)
        
        # Calcular ángulo y distancia para cada punto
        x = points[:, 0]
        y = points[:, 1]
        
        # Ángulo desde el eje X (hacia adelante)
        angles = np.arctan2(y, x)
        
        # Distancia desde el origen
        distances = np.sqrt(x**2 + y**2)
        
        # Filtrar por rango válido
        valid_mask = (distances >= self.range_min) & (distances <= self.range_max)
        angles = angles[valid_mask]
        distances = distances[valid_mask]
        
        # Mapear ángulos a índices de rayo
        indices = ((angles - self.angle_min) / self.angle_increment).astype(np.int32)
        
        # Filtrar índices válidos
        valid_indices = (indices >= 0) & (indices < self.num_ranges)
        indices = indices[valid_indices]
        distances = distances[valid_indices]
        
        # Asignar distancia mínima a cada rayo (en caso de múltiples puntos)
        for idx, dist in zip(indices, distances):
            if dist < ranges[idx]:
                ranges[idx] = dist
        
        # Convertir inf a 0 o max_range según preferencia de Cartographer
        ranges[np.isinf(ranges)] = 0.0
        
        scan.ranges = ranges.tolist()
        
        # Intensidades (opcional, puede ayudar a Cartographer)
        scan.intensities = []
        
        return scan

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PointCloudToLaserScanNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
