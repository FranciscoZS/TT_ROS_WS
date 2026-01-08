# static_tf_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Publicar transform de taulidar_link a map (o world)
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'  # Frame padre
        static_transform.child_frame_id = 'taulidar_link'  # Frame hijo
        
        # Posición (ajusta según tu setup)
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        
        # Orientación (sin rotación)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info('📡 Publicando transform estático: map -> taulidar_link')

def main():
    rclpy.init()
    node = StaticTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()