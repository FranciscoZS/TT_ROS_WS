#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
from nav_autonoma.configRobot import Config

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_process')
        
        cmd = [
            "ffmpeg",
            "-loglevel", "error",    # Solo errores
            "-hide_banner",          # Sin banner inicial
            "-f", "v4l2",
            "-input_format", "mjpeg",
            "-framerate", "30",    
            "-video_size", "640x480",
            "-i", "/dev/video0",
            "-vcodec", "mpeg1video",
            "-qscale:v", "3",
            "-b:v", "8M",
            "-pix_fmt", "yuv420p",
            "-f", "mpegts",
            Config.Emisor_UDP_ADDR
        ]
        
        self.get_logger().info("Iniciando proceso FFmpeg...")
        
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.get_logger().error(f"Error al iniciar FFmpeg: {e}")
            self.process = None
    
    def destroy_node(self):
        self.get_logger().info("Cerrando proceso FFmpeg")
        if self.process:
            self.process.terminate()
        super().destroy_node()

def main(args = None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()