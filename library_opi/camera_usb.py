#!/usr/bin/env python3
"""
Librería para manejo de cámara USB con OpenCV
Compatible con el sistema de navegación autónoma
"""
import cv2
import numpy as np
import threading

class USBCamera:
    def __init__(self, device_id=0, width=640, height=480, fps=30, node=None):
        """
        Inicializa la cámara USB
        
        Args:
            device_id (int): ID del dispositivo de cámara (0, 1, 2...)
            width (int): Ancho de la imagen en píxeles
            height (int): Alto de la imagen en píxeles
            fps (int): Frames por segundo
            node: Referencia al nodo ROS2 para logging
        """
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.node = node
        
        # Variables para almacenar frame
        self.current_frame = None
        self.lock = threading.Lock()
        self.is_opened = False
        
        # Inicializar cámara
        self.cap = None
        self._init_camera()
    
    def _init_camera(self):
        """Inicializa el objeto de captura de OpenCV"""
        try:
            self.cap = cv2.VideoCapture(self.device_id)
            
            if not self.cap.isOpened():
                if self.node:
                    self.node.get_logger().error(f'❌ No se pudo abrir la cámara {self.device_id}')
                return False
            
            # Configurar propiedades de la cámara
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Verificar configuración aplicada
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            
            self.is_opened = True
            
            if self.node:
                self.node.get_logger().info(
                    f'✅ Cámara USB inicializada - Device: {self.device_id} | '
                    f'Resolución: {actual_width}x{actual_height} | FPS: {actual_fps}'
                )
            
            return True
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error inicializando cámara: {e}')
            return False
    
    def read_frame(self):
        """
        Lee un frame de la cámara
        
        Returns:
            bool: True si se leyó correctamente, False en caso contrario
        """
        if not self.is_opened or self.cap is None:
            return False
        
        try:
            ret, frame = self.cap.read()
            
            if ret:
                with self.lock:
                    # Voltear imagen si es necesario (ajustar según orientación)
                    # frame = cv2.flip(frame, 0)  # Flip vertical
                    # frame = cv2.flip(frame, 1)  # Flip horizontal
                    self.current_frame = frame
                return True
            else:
                if self.node:
                    self.node.get_logger().warn('⚠️ No se pudo leer frame de la cámara')
                return False
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error leyendo frame: {e}')
            return False
    
    def get_frame(self):
        """
        Obtiene el frame actual de forma thread-safe
        
        Returns:
            numpy.ndarray: Frame actual o None si no hay disponible
        """
        with self.lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
            return None
    
    def get_frame_encoded(self, format='.jpg', quality=90):
        """
        Obtiene el frame actual codificado como imagen comprimida
        
        Args:
            format (str): Formato de codificación ('.jpg' o '.png')
            quality (int): Calidad de compresión para JPEG (0-100)
        
        Returns:
            bytes: Imagen codificada o None si no hay disponible
        """
        frame = self.get_frame()
        if frame is None:
            return None
        
        try:
            if format == '.jpg':
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            else:
                encode_param = []
            
            success, encoded = cv2.imencode(format, frame, encode_param)
            
            if success:
                return encoded.tobytes()
            else:
                if self.node:
                    self.node.get_logger().error('❌ Error codificando imagen')
                return None
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error en codificación: {e}')
            return None
    
    def get_resolution(self):
        """
        Obtiene la resolución actual de la cámara
        
        Returns:
            tuple: (width, height)
        """
        if self.cap is not None and self.is_opened:
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            return (width, height)
        return (0, 0)
    
    def set_resolution(self, width, height):
        """
        Cambia la resolución de la cámara
        
        Args:
            width (int): Nuevo ancho
            height (int): Nuevo alto
        
        Returns:
            bool: True si se aplicó correctamente
        """
        if self.cap is not None and self.is_opened:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.width = width
            self.height = height
            
            if self.node:
                actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.node.get_logger().info(
                    f'📐 Resolución actualizada a {actual_w}x{actual_h}'
                )
            return True
        return False
    
    def release(self):
        """Libera los recursos de la cámara"""
        if self.cap is not None:
            self.cap.release()
            self.is_opened = False
            
            if self.node:
                self.node.get_logger().info('🛑 Cámara liberada correctamente')
    
    def cleanup(self):
        """Alias para release() - compatibilidad con otros módulos"""
        self.release()