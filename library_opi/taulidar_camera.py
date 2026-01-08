#!/usr/bin/env python3
"""
Librería para manejo de cámara TauLidar OnionTau
Compatible con el sistema de navegación autónoma
"""
import numpy as np
import cv2
import threading
from cv_bridge import CvBridge
from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

class TauLidarCamera:
    def __init__(self, serial_port=None, min_range=0, max_range=4500, 
                 integration_time=1000, min_amplitude=10, node=None):
        """
        Inicializa la cámara TauLidar
        
        Args:
            serial_port (str): Puerto serial (None para auto-scan)
            min_range (int): Rango mínimo de distancia (mm)
            max_range (int): Rango máximo de distancia (mm)
            integration_time (int): Tiempo de integración 3D
            min_amplitude (int): Amplitud mínima
            node: Referencia al nodo ROS2 para logging
        """
        self.serial_port = serial_port
        self.min_range = min_range
        self.max_range = max_range
        self.integration_time = integration_time
        self.min_amplitude = min_amplitude
        self.node = node
        
        # Variables de estado
        self.camera = None
        self.is_opened = False
        self.current_frame = None
        self.lock = threading.Lock()
        
        # Información de la cámara
        self.camera_info = None
        
        # Inicializar cámara
        self._init_camera()
    
    def _init_camera(self):
        """Inicializa la cámara TauLidar"""
        try:
            # Buscar puerto si no se especificó
            port = self.serial_port
            
            if port is None:
                if self.node:
                    self.node.get_logger().info('🔍 Escaneando puertos TauLidar...')
                
                ports = Camera.scan()
                
                if len(ports) > 0:
                    port = ports[0]
                    if self.node:
                        self.node.get_logger().info(f'📡 Puerto encontrado: {port}')
                else:
                    if self.node:
                        self.node.get_logger().error('❌ No se encontraron puertos TauLidar')
                    return False
            
            # Configurar rango de distancias
            Camera.setRange(self.min_range, self.max_range)
            
            # Abrir cámara
            self.camera = Camera.open(port)
            
            if self.camera is None:
                if self.node:
                    self.node.get_logger().error(f'❌ No se pudo abrir el puerto {port}')
                return False
            
            # Configurar parámetros
            self.camera.setModulationChannel(0)  # autoChannelEnabled: 0, channel: 0
            self.camera.setIntegrationTime3d(0, self.integration_time)
            self.camera.setMinimalAmplitude(0, self.min_amplitude)
            
            # Obtener información de la cámara
            self.camera_info = self.camera.info()
            self.is_opened = True
            
            if self.node:
                self.node.get_logger().info(
                    f'✅ TauLidar inicializado:\n'
                    f'   Modelo: {self.camera_info.model}\n'
                    f'   Firmware: {self.camera_info.firmware}\n'
                    f'   UID: {self.camera_info.uid}\n'
                    f'   Resolución: {self.camera_info.resolution}\n'
                    f'   Puerto: {self.camera_info.port}\n'
                    f'   Rango: {self.min_range}-{self.max_range} mm'
                )
            
            return True
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error inicializando TauLidar: {e}')
            return False
    
    def read_frame(self, frame_type=FrameType.DISTANCE):
        """
        Lee un frame de la cámara TauLidar
        
        Args:
            frame_type: Tipo de frame (FrameType.DISTANCE, etc.)
        
        Returns:
            bool: True si se leyó correctamente
        """
        if not self.is_opened or self.camera is None:
            return False
        
        try:
            frame = self.camera.readFrame(frame_type)
            
            if frame:
                with self.lock:
                    self.current_frame = frame
                return True
            else:
                return False
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error leyendo frame: {e}')
            return False
    
    def get_depth_rgb(self, upscale=4):
        """
        Obtiene el mapa de profundidad como imagen RGB
        
        Args:
            upscale (int): Factor de escalado (1, 2, 4, etc.)
        
        Returns:
            numpy.ndarray: Imagen RGB del mapa de profundidad o None
        """
        with self.lock:
            if self.current_frame is None:
                return None
            
            try:
                # Convertir datos a matriz RGB
                mat_depth_rgb = np.frombuffer(
                    self.current_frame.data_depth_rgb,
                    dtype=np.uint16,
                    count=-1,
                    offset=0
                ).reshape(
                    self.current_frame.height,
                    self.current_frame.width,
                    3
                )
                
                # Convertir a uint8
                mat_depth_rgb = mat_depth_rgb.astype(np.uint8)
                
                # Escalar imagen si es necesario
                if upscale > 1:
                    mat_depth_rgb = cv2.resize(
                        mat_depth_rgb,
                        (self.current_frame.width * upscale,
                         self.current_frame.height * upscale)
                    )
                
                return mat_depth_rgb
                
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f'❌ Error procesando depth RGB: {e}')
                return None
    
    def get_distance_matrix(self):
        """
        Obtiene la matriz de distancias en milímetros
        
        Returns:
            numpy.ndarray: Matriz de distancias o None
        """
        with self.lock:
            if self.current_frame is None:
                return None
            
            try:
                # Extraer datos de distancia
                if hasattr(self.current_frame, 'data_depth'):
                    mat_distance = np.frombuffer(
                        self.current_frame.data_depth,
                        dtype=np.uint16,
                        count=-1,
                        offset=0
                    ).reshape(
                        self.current_frame.height,
                        self.current_frame.width
                    )
                    return mat_distance
                else:
                    return None
                    
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f'❌ Error extrayendo matriz de distancia: {e}')
                return None
    
    def get_resolution(self):
        """
        Obtiene la resolución de la cámara
        
        Returns:
            tuple: (width, height) o (0, 0) si no está disponible
        """
        if self.camera_info:
            # El formato puede ser "WxH" como "160x60"
            res_str = self.camera_info.resolution
            try:
                w, h = res_str.split('x')
                return (int(w), int(h))
            except:
                pass
        
        if self.current_frame:
            return (self.current_frame.width, self.current_frame.height)
        
        return (0, 0)
    
    def get_info(self):
        """
        Obtiene información de la cámara
        
        Returns:
            dict: Información de la cámara
        """
        if self.camera_info:
            return {
                'model': self.camera_info.model,
                'firmware': self.camera_info.firmware,
                'uid': self.camera_info.uid,
                'resolution': self.camera_info.resolution,
                'port': self.camera_info.port,
                'min_range': self.min_range,
                'max_range': self.max_range
            }
        return {}
    
    def set_integration_time(self, time_us):
        """
        Ajusta el tiempo de integración 3D
        
        Args:
            time_us (int): Tiempo en microsegundos
        """
        if self.camera:
            try:
                self.camera.setIntegrationTime3d(0, time_us)
                self.integration_time = time_us
                if self.node:
                    self.node.get_logger().info(f'⚙️ Integration time: {time_us} us')
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f'❌ Error setting integration time: {e}')
    
    def set_minimal_amplitude(self, amplitude):
        """
        Ajusta la amplitud mínima
        
        Args:
            amplitude (int): Amplitud mínima
        """
        if self.camera:
            try:
                self.camera.setMinimalAmplitude(0, amplitude)
                self.min_amplitude = amplitude
                if self.node:
                    self.node.get_logger().info(f'⚙️ Minimal amplitude: {amplitude}')
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f'❌ Error setting amplitude: {e}')

    def get_points_3d(self):
            """
            Obtiene los puntos 3D del frame actual
            
            Returns:
                list: Lista de puntos 3D (x, y, z) o None
            """
            with self.lock:
                if self.current_frame is None:
                    return None
                
                try:
                    # Verificar si el frame tiene puntos 3D
                    if hasattr(self.current_frame, 'points_3d') and self.current_frame.points_3d:
                        return self.current_frame.points_3d
                    else:
                        return None
                        
                except Exception as e:
                    if self.node:
                        self.node.get_logger().error(f'❌ Error obteniendo puntos 3D: {e}')
                    return None

    def get_pointcloud_msg(self, timestamp):
        """
        Convierte los puntos 3D a mensaje PointCloud2 de ROS2
        
        Args:
            timestamp: Timestamp para el header
            
        Returns:
            PointCloud2: Mensaje de nube de puntos o None
        """
        try:
            points_3d = self.get_points_3d()
            
            if points_3d is None or len(points_3d) == 0:
                return None
            
            # Crear mensaje PointCloud2
            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = timestamp
            msg.header.frame_id = 'taulidar_link'
            
            # Definir campos (x, y, z)
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            
            msg.is_bigendian = False
            msg.point_step = 12  # 3 float32 * 4 bytes cada uno
            msg.row_step = msg.point_step * len(points_3d)
            msg.is_dense = False
            
            # Convertir puntos a bytes
            points_data = []
            for point in points_3d:
                if point is not None and len(point) >= 3:
                    # Convertir de milímetros a metros para ROS
                    x = point[0] / 100 if point[0] is not None else 0.0
                    y = point[1] / 100 if point[1] is not None else 0.0
                    z = point[2] / 100 if point[2] is not None else 0.0
                    
                    # Empacar datos
                    points_data.append(struct.pack('fff', x, y, z))
            
            msg.data = b''.join(points_data)
            msg.height = 1
            msg.width = len(points_data)
            
            return msg
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error creando mensaje PointCloud2: {e}')
            return None

    
    def close(self):
        """Cierra la cámara y libera recursos"""
        if self.camera:
            try:
                self.camera.close()
                self.is_opened = False
                
                if self.node:
                    self.node.get_logger().info('🛑 TauLidar cerrado correctamente')
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f'❌ Error cerrando TauLidar: {e}')
    
    def cleanup(self):
        """Alias para close() - compatibilidad con otros módulos"""
        self.close()
