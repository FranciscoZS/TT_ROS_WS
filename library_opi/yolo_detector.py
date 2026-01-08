#!/usr/bin/env python3
"""
Librería para detección de objetos con YOLO
Compatible con el sistema de navegación autónoma
"""
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import time

class YOLODetector:
    def __init__(self, model_path, task="detect", conf_threshold=0.5, iou_threshold=0.45, node=None):
        """
        Inicializa el detector YOLO
        
        Args:
            model_path (str): Ruta al modelo YOLO (formato NCNN, PT, ONNX, etc.)
            task (str): Tipo de tarea ('detect', 'segment', 'pose')
            conf_threshold (float): Umbral de confianza (0.0-1.0)
            iou_threshold (float): Umbral de IoU para NMS
            node: Referencia al nodo ROS2 para logging
        """
        self.model_path = model_path
        self.task = task
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.node = node
        
        # Variables de estado
        self.model = None
        self.is_loaded = False
        self.last_detections = []
        self.lock = threading.Lock()
        
        # Estadísticas
        self.inference_count = 0
        self.total_inference_time = 0.0
        self.avg_fps = 0.0
        
        # Cargar modelo
        self._load_model()
    
    def _load_model(self):
        """Carga el modelo YOLO"""
        try:
            if self.node:
                self.node.get_logger().info(f'🤖 Cargando modelo YOLO desde: {self.model_path}')
            
            self.model = YOLO(self.model_path, task=self.task)
            self.is_loaded = True
            
            if self.node:
                self.node.get_logger().info(
                    f'✅ Modelo YOLO cargado exitosamente\n'
                    f'   Tipo: {self.task}\n'
                    f'   Confianza mínima: {self.conf_threshold}\n'
                    f'   IoU threshold: {self.iou_threshold}'
                )
            
            return True
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error cargando modelo YOLO: {e}')
            return False
    
    def detect(self, frame, visualize=False):
        """
        Realiza detección en un frame
        
        Args:
            frame (numpy.ndarray): Imagen BGR de OpenCV
            visualize (bool): Si dibujar las detecciones en el frame
        
        Returns:
            tuple: (frame_procesado, detecciones)
                - frame_procesado: Frame con detecciones dibujadas (si visualize=True)
                - detecciones: Lista de diccionarios con info de cada detección
        """
        if not self.is_loaded or self.model is None:
            return frame, []
        
        try:
            start_time = time.time()
            
            # Realizar inferencia
            results = self.model(
                frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False
            )
            
            # Calcular tiempo de inferencia
            inference_time = time.time() - start_time
            self.inference_count += 1
            self.total_inference_time += inference_time
            self.avg_fps = 1.0 / (self.total_inference_time / self.inference_count)
            
            # Procesar detecciones
            detections = []
            frame_with_detections = frame.copy() if visualize else frame
            
            for result in results:
                boxes = result.boxes
                
                if boxes is None or len(boxes) == 0:
                    continue
                
                for box in boxes:
                    # Extraer información
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    cls = int(box.cls[0].cpu().numpy())
                    confidence = float(box.conf[0].cpu().numpy())
                    label = self.model.names[cls]
                    
                    # Crear diccionario de detección
                    detection = {
                        'bbox': [x1, y1, x2, y2],
                        'class_id': cls,
                        'class_name': label,
                        'confidence': confidence,
                        'center': [(x1 + x2) // 2, (y1 + y2) // 2],
                        'area': (x2 - x1) * (y2 - y1)
                    }
                    detections.append(detection)
                    
                    # Visualizar si se solicita
                    if visualize:
                        # Color según la clase (puedes personalizar)
                        color = self._get_color_for_class(cls)
                        
                        # Dibujar bounding box
                        cv2.rectangle(
                            frame_with_detections,
                            (x1, y1), (x2, y2),
                            color, 2
                        )
                        
                        # Dibujar etiqueta
                        label_text = f"{label} {confidence:.2f}"
                        label_size, _ = cv2.getTextSize(
                            label_text,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, 2
                        )
                        
                        # Fondo para el texto
                        cv2.rectangle(
                            frame_with_detections,
                            (x1, y1 - label_size[1] - 10),
                            (x1 + label_size[0], y1),
                            color, -1
                        )
                        
                        # Texto
                        cv2.putText(
                            frame_with_detections,
                            label_text,
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2
                        )
                        
                        # Punto central
                        center = detection['center']
                        cv2.circle(
                            frame_with_detections,
                            tuple(center),
                            5, color, -1
                        )
            
            # Agregar información de rendimiento si visualize=True
            if visualize:
                fps_text = f"FPS: {self.avg_fps:.1f} | Detecciones: {len(detections)}"
                cv2.putText(
                    frame_with_detections,
                    fps_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2
                )
            
            # Guardar detecciones
            with self.lock:
                self.last_detections = detections
            
            return frame_with_detections, detections
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error en detección: {e}')
            return frame, []
    
    def _get_color_for_class(self, class_id):
        """
        Genera un color único para cada clase
        
        Args:
            class_id (int): ID de la clase
        
        Returns:
            tuple: Color BGR
        """
        # Colores predefinidos para las primeras clases
        colors = [
            (0, 255, 0),    # Verde
            (0, 0, 255),    # Rojo
            (255, 0, 0),    # Azul
            (0, 255, 255),  # Amarillo
            (255, 0, 255),  # Magenta
            (255, 255, 0),  # Cian
            (128, 0, 128),  # Púrpura
            (255, 165, 0),  # Naranja
        ]
        
        if class_id < len(colors):
            return colors[class_id]
        else:
            # Generar color aleatorio pero consistente para la clase
            np.random.seed(class_id)
            return tuple(np.random.randint(0, 255, 3).tolist())
    
    def get_last_detections(self):
        """
        Obtiene las últimas detecciones de forma thread-safe
        
        Returns:
            list: Lista de detecciones
        """
        with self.lock:
            return self.last_detections.copy()
    
    def get_statistics(self):
        """
        Obtiene estadísticas de rendimiento
        
        Returns:
            dict: Estadísticas
        """
        return {
            'inference_count': self.inference_count,
            'avg_inference_time': self.total_inference_time / max(self.inference_count, 1),
            'avg_fps': self.avg_fps,
            'last_detection_count': len(self.last_detections)
        }
    
    def set_confidence_threshold(self, threshold):
        """Ajusta el umbral de confianza"""
        self.conf_threshold = max(0.0, min(1.0, threshold))
        if self.node:
            self.node.get_logger().info(f'⚙️ Confianza ajustada a: {self.conf_threshold:.2f}')
    
    def set_iou_threshold(self, threshold):
        """Ajusta el umbral de IoU"""
        self.iou_threshold = max(0.0, min(1.0, threshold))
        if self.node:
            self.node.get_logger().info(f'⚙️ IoU threshold ajustado a: {self.iou_threshold:.2f}')
    
    def get_class_names(self):
        """Obtiene los nombres de las clases del modelo"""
        if self.model and self.is_loaded:
            return self.model.names
        return {}
    
    def filter_by_class(self, detections, class_names):
        """
        Filtra detecciones por nombre de clase
        
        Args:
            detections (list): Lista de detecciones
            class_names (list): Lista de nombres de clases a mantener
        
        Returns:
            list: Detecciones filtradas
        """
        return [d for d in detections if d['class_name'] in class_names]
    
    def get_closest_detection(self, detections, target_class=None):
        """
        Obtiene la detección más cercana al centro de la imagen
        
        Args:
            detections (list): Lista de detecciones
            target_class (str): Filtrar por clase específica (opcional)
        
        Returns:
            dict: Detección más cercana o None
        """
        if not detections:
            return None
        
        # Filtrar por clase si se especifica
        if target_class:
            detections = [d for d in detections if d['class_name'] == target_class]
        
        if not detections:
            return None
        
        # Asumir centro de imagen (esto se puede parametrizar)
        image_center = [320, 240]  # Ajustar según resolución
        
        # Encontrar la detección con centro más cercano
        min_distance = float('inf')
        closest = None
        
        for detection in detections:
            dx = detection['center'][0] - image_center[0]
            dy = detection['center'][1] - image_center[1]
            distance = (dx**2 + dy**2) ** 0.5
            
            if distance < min_distance:
                min_distance = distance
                closest = detection
        
        return closest
    
    def cleanup(self):
        """Libera recursos del modelo"""
        if self.node:
            self.node.get_logger().info('🛑 Liberando recursos de YOLO')
        
        self.model = None
        self.is_loaded = False
