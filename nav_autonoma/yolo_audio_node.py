#!/usr/bin/env python3
"""
Nodo que integra YOLO con reproducción de audio
Reproduce audio cuando detecta clases específicas
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import String, Bool
import cv2
import time

from library_opi.camera_usb import USBCamera
from library_opi.yolo_detector import YOLODetector
from library_opi.audio_player import AudioPlayer

class YOLOAudioTriggerNode(Node):
    def __init__(self):
        super().__init__('yolo_audio_trigger_node')
        
        # ==================== PARÁMETROS DE CÁMARA ====================
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        # ==================== PARÁMETROS DE YOLO ====================
        self.declare_parameter('model_path', '/home/orangepi/Trabajo_Terminal_OrangepiCode/TestCameraAndYolo/best_ncnn_model')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('publish_rate', 30.0)
        
        # ==================== PARÁMETROS DE AUDIO ====================
        self.declare_parameter('audio_file', '/home/orangepi/sound_ia/resource/desalojo.mp3')
        self.declare_parameter('audio_device', 'plughw:3,0')
        self.declare_parameter('trigger_cooldown', 5.0)  # Segundos entre triggers
        
        # ==================== PARÁMETROS DE TRIGGER ====================
        self.declare_parameter('target_classes', ['student'])  # Clases que activan el audio
        self.declare_parameter('min_confidence', 0.5)  # Confianza mínima para trigger
        self.declare_parameter('min_detections', 1)  # Detecciones mínimas para trigger
        self.declare_parameter('trigger_mode', 'on_detect')  # 'on_detect' o 'continuous'
        
        # ==================== PARÁMETROS DE VISUALIZACIÓN ====================
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
        self.audio_file = self.get_parameter('audio_file').value
        audio_device = self.get_parameter('audio_device').value
        self.trigger_cooldown = self.get_parameter('trigger_cooldown').value
        self.target_classes = self.get_parameter('target_classes').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.min_detections = self.get_parameter('min_detections').value
        self.trigger_mode = self.get_parameter('trigger_mode').value
        self.publish_image = self.get_parameter('publish_image').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
        # ==================== INICIALIZAR COMPONENTES ====================
        
        # Inicializar cámara
        self.get_logger().info('🎥 Inicializando cámara...')
        self.camera = USBCamera(device_id, width, height, fps, node=self)
        if not self.camera.is_opened:
            raise RuntimeError('No se pudo inicializar la cámara')
        
        # Inicializar YOLO
        self.get_logger().info('🤖 Inicializando YOLO...')
        self.detector = YOLODetector(
            model_path, "detect", conf_threshold, iou_threshold, node=self
        )
        if not self.detector.is_loaded:
            raise RuntimeError('No se pudo cargar el modelo YOLO')
        
        # Inicializar audio
        self.get_logger().info('🔊 Inicializando audio...')
        self.audio = AudioPlayer(audio_device, node=self)
        
        # Bridge de OpenCV a ROS
        self.bridge = None
        if self.publish_image:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
        
        # ==================== PUBLISHERS ====================
        self.detections_pub = self.create_publisher(
            Detection2DArray, 'yolo/detections', 10
        )
        
        if self.publish_image:
            if self.use_compressed:
                self.image_pub = self.create_publisher(
                    CompressedImage, 'yolo/image/compressed', 10
                )
            else:
                from sensor_msgs.msg import Image
                self.image_pub = self.create_publisher(
                    Image, 'yolo/image_raw', 10
                )
        
        self.trigger_pub = self.create_publisher(Bool, 'yolo/audio_triggered', 10)
        self.info_pub = self.create_publisher(String, 'yolo/info', 10)
        
        # ==================== VARIABLES DE ESTADO ====================
        self.frame_count = 0
        self.detection_count = 0
        self.trigger_count = 0
        self.last_trigger_time = 0
        
        # ==================== TIMERS ====================
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.process_frame)
        self.stats_timer = self.create_timer(5.0, self.publish_statistics)
        
        # ==================== INFORMACIÓN INICIAL ====================
        class_names = self.detector.get_class_names()
        self.get_logger().info(
            f'✅ Nodo YOLO + Audio inicializado\n'
            f'   📹 Cámara: {width}x{height} @ {fps} FPS\n'
            f'   🤖 Modelo: {model_path}\n'
            f'   🎯 Clases objetivo: {self.target_classes}\n'
            f'   🔊 Audio: {self.audio_file}\n'
            f'   ⏱️  Cooldown: {self.trigger_cooldown}s\n'
            f'   📊 Clases disponibles: {list(class_names.values())}'
        )

    def process_frame(self):
        """Captura frame, detecta objetos y activa audio si es necesario"""
        try:
            # Leer frame
            if not self.camera.read_frame():
                return
            
            frame = self.camera.get_frame()
            if frame is None:
                return
            
            # Realizar detección
            frame_with_detections, detections = self.detector.detect(frame, visualize=True)
            
            # Filtrar detecciones por clases objetivo
            target_detections = self._filter_target_detections(detections)
            
            # Verificar si se debe activar el audio
            if self._should_trigger_audio(target_detections):
                self._trigger_audio()
            
            # Publicar resultados
            timestamp = self.get_clock().now().to_msg()
            self._publish_detections(detections, timestamp)
            
            if self.publish_image:
                self._publish_image(frame_with_detections, timestamp)
            
            self.frame_count += 1
            self.detection_count += len(detections)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando frame: {e}')

    def _filter_target_detections(self, detections):
        """Filtra detecciones según las clases objetivo"""
        target_detections = []
        
        for det in detections:
            if (det['class_name'] in self.target_classes and 
                det['confidence'] >= self.min_confidence):
                target_detections.append(det)
        
        return target_detections

    def _should_trigger_audio(self, target_detections):
        """Determina si se debe activar el audio"""
        # Verificar número mínimo de detecciones
        if len(target_detections) < self.min_detections:
            return False
        
        # Verificar cooldown
        current_time = time.time()
        time_since_last_trigger = current_time - self.last_trigger_time
        
        if time_since_last_trigger < self.trigger_cooldown:
            return False
        
        # Verificar modo de trigger
        if self.trigger_mode == 'on_detect':
            # Solo activar si hay detecciones nuevas
            return True
        elif self.trigger_mode == 'continuous':
            # Activar mientras haya detecciones (respetando cooldown)
            return True
        
        return False

    def _trigger_audio(self):
        """Activa la reproducción de audio"""
        try:
            # Solo reproducir si no hay audio en curso
            if not self.audio.is_playing_audio():
                success = self.audio.play(self.audio_file, blocking=False)
                
                if success:
                    self.trigger_count += 1
                    self.last_trigger_time = time.time()
                    
                    # Publicar evento de trigger
                    trigger_msg = Bool()
                    trigger_msg.data = True
                    self.trigger_pub.publish(trigger_msg)
                    
                    self.get_logger().info(
                        f'🔔 Audio activado! (Trigger #{self.trigger_count})'
                    )
        except Exception as e:
            self.get_logger().error(f'❌ Error activando audio: {e}')

    def _publish_detections(self, detections, timestamp):
        """Publica detecciones en formato vision_msgs"""
        try:
            msg = Detection2DArray()
            msg.header.stamp = timestamp
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

    def _publish_image(self, frame, timestamp):
        """Publica imagen con detecciones"""
        try:
            if self.use_compressed:
                msg = CompressedImage()
                msg.header.stamp = timestamp
                msg.header.frame_id = 'camera_link'
                msg.format = 'jpeg'
                
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
                success, encoded = cv2.imencode('.jpg', frame, encode_param)
                
                if success:
                    msg.data = encoded.tobytes()
                    self.image_pub.publish(msg)
            else:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = timestamp
                msg.header.frame_id = 'camera_link'
                self.image_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando imagen: {e}')

    def publish_statistics(self):
        """Publica estadísticas del sistema"""
        try:
            stats = self.detector.get_statistics()
            
            info_msg = String()
            info_msg.data = (
                f"Frames: {self.frame_count} | "
                f"Detecciones: {self.detection_count} | "
                f"Triggers: {self.trigger_count} | "
                f"FPS: {stats['avg_fps']:.1f} | "
                f"Audio: {'🔊' if self.audio.is_playing_audio() else '🔇'}"
            )
            
            self.info_pub.publish(info_msg)
            
            self.get_logger().info(
                f'📊 Stats - Frames: {self.frame_count} | '
                f'Detecciones: {self.detection_count} | '
                f'Audio triggers: {self.trigger_count}'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en estadísticas: {e}')

    def destroy_node(self):
        """Cleanup al cerrar el nodo"""
        self.get_logger().info('🛑 Cerrando nodo YOLO + Audio...')
        
        # Detener audio
        self.audio.cleanup()
        
        # Mostrar estadísticas finales
        stats = self.detector.get_statistics()
        self.get_logger().info(
            f'📈 Estadísticas finales:\n'
            f'   Total frames: {self.frame_count}\n'
            f'   Total detecciones: {self.detection_count}\n'
            f'   Total triggers de audio: {self.trigger_count}\n'
            f'   FPS promedio: {stats["avg_fps"]:.1f}'
        )
        
        # Liberar recursos
        self.detector.cleanup()
        self.camera.cleanup()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLOAudioTriggerNode()
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
