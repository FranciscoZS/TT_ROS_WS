#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2

from library_opi.camera_usb import USBCamera
from library_opi.yolo_detector import YOLODetector

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # Parámetros configurables
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('model_path', '/home/orangepi/Trabajo_Terminal_OrangepiCode/TestCameraAndYolo/best_ncnn_model')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('visualize', True)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('use_compressed', True)
        
        # Obtener parámetros
        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('conf_threshold').value
        iou_threshold = self.get_parameter('iou_threshold').value
        publish_rate = self.get_parameter('publish_rate').value
        self.visualize = self.get_parameter('visualize').value
        self.publish_image = self.get_parameter('publish_image').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
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
            raise RuntimeError('No se pudo cargar el modelo YOLO')
        
        # Bridge de OpenCV a ROS
        self.bridge = CvBridge()
        
        # Publishers
        # 1. Detecciones estructuradas (vision_msgs)
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            'yolo/detections',
            10
        )
        
        # 2. Imagen con visualización (opcional)
        if self.publish_image:
            if self.use_compressed:
                self.image_pub = self.create_publisher(
                    CompressedImage,
                    'yolo/image/compressed',
                    10
                )
            else:
                self.image_pub = self.create_publisher(
                    Image,
                    'yolo/image_raw',
                    10
                )
        
        # 3. Información textual
        self.info_pub = self.create_publisher(String, 'yolo/info', 10)
        
        # Timer para captura y detección
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.process_frame)
        
        # Contadores
        self.frame_count = 0
        self.detection_count = 0
        
        # Timer para estadísticas
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        self.get_logger().info(
            f'✅ Nodo YOLO inicializado\n'
            f'   📹 Cámara: {device_id} | {width}x{height}\n'
            f'   🤖 Modelo: {model_path}\n'
            f'   🎯 Confianza: {conf_threshold} | IoU: {iou_threshold}\n'
            f'   🔄 Tasa: {publish_rate} Hz\n'
            f'   👁️  Visualización: {"ON" if self.visualize else "OFF"}'
        )

    def process_frame(self):
        """Captura frame, realiza detección y publica resultados"""
        try:
            # Leer frame
            if not self.camera.read_frame():
                return
            
            frame = self.camera.get_frame()
            if frame is None:
                return
            
            # Realizar detección
            frame_with_detections, detections = self.detector.detect(
                frame,
                visualize=self.visualize
            )
            
            # Crear timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Publicar detecciones estructuradas
            self.publish_detections(detections, timestamp)
            
            # Publicar imagen con detecciones (opcional)
            if self.publish_image:
                if self.use_compressed:
                    self.publish_compressed_image(frame_with_detections, timestamp)
                else:
                    self.publish_raw_image(frame_with_detections, timestamp)
            
            self.frame_count += 1
            self.detection_count += len(detections)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando frame: {e}')

    def publish_detections(self, detections, timestamp):
        """Publica detecciones en formato vision_msgs"""
        try:
            msg = Detection2DArray()
            msg.header.stamp = timestamp
            msg.header.frame_id = 'camera_link'
            
            for det in detections:
                detection = Detection2D()
                
                # Hypothesis (clase y confianza)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(det['class_id'])
                hypothesis.hypothesis.score = det['confidence']
                detection.results.append(hypothesis)
                
                # Bounding box
                detection.bbox.center.position.x = float(det['center'][0])
                detection.bbox.center.position.y = float(det['center'][1])
                detection.bbox.size_x = float(det['bbox'][2] - det['bbox'][0])
                detection.bbox.size_y = float(det['bbox'][3] - det['bbox'][1])
                
                # ID único (opcional)
                detection.id = f"{det['class_name']}_{self.frame_count}"
                
                msg.detections.append(detection)
            
            self.detections_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando detecciones: {e}')

    def publish_raw_image(self, frame, timestamp):
        """Publica imagen raw"""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = 'camera_link'
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando imagen: {e}')

    def publish_compressed_image(self, frame, timestamp):
        """Publica imagen comprimida"""
        try:
            msg = CompressedImage()
            msg.header.stamp = timestamp
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            success, encoded = cv2.imencode('.jpg', frame, encode_param)
            
            if success:
                msg.data = encoded.tobytes()
                self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando imagen comprimida: {e}')

    def publish_statistics(self):
        """Publica estadísticas"""
        try:
            stats = self.detector.get_statistics()
            
            info_msg = String()
            info_msg.data = (
                f"Frames: {self.frame_count} | "
                f"Detecciones totales: {self.detection_count} | "
                f"FPS: {stats['avg_fps']:.1f} | "
                f"Última detección: {stats['last_detection_count']}"
            )
            
            self.info_pub.publish(info_msg)
            
            self.get_logger().info(
                f'📊 Estadísticas - Frames: {self.frame_count} | '
                f'Detecciones: {self.detection_count} | '
                f'FPS: {stats["avg_fps"]:.1f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en estadísticas: {e}')

    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('🛑 Cerrando nodo YOLO...')
        
        # Mostrar estadísticas finales
        stats = self.detector.get_statistics()
        self.get_logger().info(
            f'📈 Estadísticas finales:\n'
            f'   Total frames: {self.frame_count}\n'
            f'   Total detecciones: {self.detection_count}\n'
            f'   FPS promedio: {stats["avg_fps"]:.1f}'
        )
        
        # Liberar recursos
        self.detector.cleanup()
        self.camera.cleanup()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLODetectionNode()
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
