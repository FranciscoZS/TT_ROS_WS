#!/usr/bin/env python3
"""
Nodo YOLO con display directo usando OpenCV
Para máximo rendimiento en visualización local
Similar a tu script TestYolo12Camera.py pero integrado en ROS2
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import cv2
import threading
import time

from library_opi.camera_usb import USBCamera
from library_opi.yolo_detector import YOLODetector

class YOLODisplayNode(Node):
    def __init__(self):
        super().__init__('yolo_display_node')
        
        # Parámetros
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('model_path', '/home/orangepi/Trabajo_Terminal_OrangepiCode/TestCameraAndYolo/best_ncnn_model')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('show_display', True)
        self.declare_parameter('publish_detections', False)  # Desactivado por defecto
        self.declare_parameter('flip_vertical', True)  # Como tu código
        self.declare_parameter('flip_horizontal', True)
        
        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('conf_threshold').value
        iou_threshold = self.get_parameter('iou_threshold').value
        self.show_display = self.get_parameter('show_display').value
        self.publish_detections = self.get_parameter('publish_detections').value
        self.flip_vertical = self.get_parameter('flip_vertical').value
        self.flip_horizontal = self.get_parameter('flip_horizontal').value
        
        # Inicializar cámara
        self.get_logger().info('🎥 Inicializando cámara...')
        self.camera = USBCamera(
            device_id=device_id,
            width=width,
            height=height,
            fps=fps,
            node=self
        )
        
        if not self.camera.is_opened:
            raise RuntimeError('No se pudo inicializar la cámara')
        
        # Inicializar YOLO
        self.get_logger().info('🤖 Inicializando YOLO...')
        self.detector = YOLODetector(
            model_path=model_path,
            task="detect",
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            node=self
        )
        
        if not self.detector.is_loaded:
            raise RuntimeError('No se pudo cargar modelo YOLO')
        
        # Publishers (opcional)
        if self.publish_detections:
            self.detections_pub = self.create_publisher(
                Detection2DArray,
                'yolo/detections',
                10
            )
            self.get_logger().info('📡 Publisher de detecciones habilitado')
        
        self.info_pub = self.create_publisher(String, 'yolo/info', 10)
        
        # Variables de control
        self.frame_count = 0
        self.detection_count = 0
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
        
        self.get_logger().info(
            f'✅ Nodo inicializado\n'
            f'   Modelo: {model_path}\n'
            f'   Confianza: {conf_threshold}'
        )

    def display_loop(self):
        """Loop de display con detección en tiempo real"""
        window_name = 'Detección en tiempo real - YOLO | Press Q to quit'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        self.get_logger().info('🎬 Display loop iniciado')
        
        fps_counter = 0
        start_time = time.time()
        fps_real = 0.0
        
        while self.display_running:
            if self.camera.read_frame():
                frame = self.camera.get_frame()
                
                if frame is not None:
                    # Aplicar flips como en tu código original
                    if self.flip_horizontal:
                        frame = cv2.flip(frame, 1)
                    if self.flip_vertical:
                        frame = cv2.flip(frame, 0)
                    
                    # Realizar detección con visualización
                    frame_with_detections, detections = self.detector.detect(
                        frame,
                        visualize=True
                    )
                    
                    fps_counter += 1
                    self.frame_count += 1
                    self.detection_count += len(detections)
                    
                    # Calcular FPS real
                    elapsed = time.time() - start_time
                    if elapsed >= 1.0:
                        fps_real = fps_counter / elapsed
                        fps_counter = 0
                        start_time = time.time()
                    
                    # Agregar información adicional
                    info_text = f"Total detecciones: {self.detection_count}"
                    cv2.putText(
                        frame_with_detections,
                        info_text,
                        (10, frame_with_detections.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2
                    )
                    
                    # Mostrar
                    cv2.imshow(window_name, frame_with_detections)
                    
                    # Publicar detecciones si está habilitado
                    if self.publish_detections:
                        self.publish_detections_msg(detections)
                    
                    # Salir con 'q'
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info('👋 Cerrando display...')
                        self.display_running = False
                        break
        
        cv2.destroyAllWindows()
        self.get_logger().info('🛑 Display loop terminado')

    def publish_detections_msg(self, detections):
        """Publica detecciones estructuradas"""
        try:
            msg = Detection2DArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            
            for det in detections:
                detection = Detection2D()
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(det['class_id'])
                hypothesis.hypothesis.score = det['confidence']
                detection.results.append(hypothesis)
                
                detection.bbox.center.position.x = float(det['center'][0])
                detection.bbox.center.position.y = float(det['center'][1])
                detection.bbox.size_x = float(det['bbox'][2] - det['bbox'][0])
                detection.bbox.size_y = float(det['bbox'][3] - det['bbox'][1])
                
                detection.id = f"{det['class_name']}_{self.frame_count}"
                
                msg.detections.append(detection)
            
            self.detections_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando detecciones: {e}')

    def publish_statistics(self):
        """Publicar estadísticas"""
        try:
            stats = self.detector.get_statistics()
            
            info_msg = String()
            info_msg.data = (
                f"Frames: {self.frame_count} | "
                f"Detecciones: {self.detection_count} | "
                f"FPS: {stats['avg_fps']:.1f}"
            )
            self.info_pub.publish(info_msg)
            
            self.get_logger().info(
                f'📊 Frames: {self.frame_count} | '
                f'Detecciones: {self.detection_count}'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en estadísticas: {e}')

    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('🛑 Cerrando nodo...')
        
        # Detener display
        self.display_running = False
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=2.0)
        
        # Liberar recursos
        self.detector.cleanup()
        self.camera.cleanup()
        
        self.get_logger().info(
            f'📈 Total frames: {self.frame_count} | '
            f'Total detecciones: {self.detection_count}'
        )
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLODisplayNode()
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
