#!/usr/bin/env python3
"""
Ejecutor de Ruta RRT
- Lee archivo JSON de instrucciones generado por RRT Planner
- Ejecuta movimientos secuencialmente
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
import json
import math
import os

class RRTPathExecutor(Node):
    def __init__(self):
        super().__init__('rrt_path_executor')
        
        # Parámetros
        self.declare_parameter('instructions_file', '')
        self.declare_parameter('loop_execution', False)
        
        self.instructions_file = self.get_parameter('instructions_file').value
        self.loop_execution = self.get_parameter('loop_execution').value
        
        # Estado
        self.state = "IDLE"
        self.current_instruction_idx = 0
        self.instructions = []
        self.path_points = []
        self.current_pose = None
        
        # Suscriptores
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publicadores
        self.motor_pub = self.create_publisher(String, 'motor_command', 10)
        self.status_pub = self.create_publisher(String, '/rrt_executor_status', 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Cargar instrucciones si se proporciona archivo
        if self.instructions_file and os.path.exists(self.instructions_file):
            self.load_instructions()
        else:
            self.get_logger().warn("⚠️ No se especificó archivo de instrucciones")
        
        self.get_logger().info("✅ RRT Path Executor listo")
    
    def load_instructions(self):
        """Carga instrucciones desde archivo JSON"""
        try:
            with open(self.instructions_file, 'r') as f:
                data = json.load(f)
            
            self.instructions = data.get('instructions', [])
            self.path_points = data.get('path_points', [])
            
            self.get_logger().info(
                f"📂 Instrucciones cargadas: {len(self.instructions)} movimientos"
            )
            self.get_logger().info(
                f"📊 Distancia total: {data.get('metadata', {}).get('total_distance', 0):.2f}m"
            )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error cargando instrucciones: {e}")
            return False
    
    def odom_callback(self, msg):
        """Actualiza odometría"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                         1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        self.current_pose = {'x': x, 'y': y, 'theta': theta}
    
    def execute_instruction(self, instruction):
        """Ejecuta una instrucción individual"""
        inst_type = instruction.get('type', '')
        
        if inst_type == 'FORWARD':
            distance = instruction.get('distance', 0)
            speed = instruction.get('speed_percent', 50)
            
            # Enviar comando a motores
            cmd = f"FORWARD:PRESS:{speed}"
            self.send_motor_command(cmd)
            
            # Monitorear distancia recorrida
            # (Implementar lógica similar a path_follower)
            
        elif inst_type == 'ROTATE':
            angle = math.radians(instruction.get('angle', 0))
            direction = instruction.get('direction', 'CW')
            speed = instruction.get('speed_percent', 40)
            
            if direction == 'CW':
                cmd = f"CW:PRESS:{speed}"
            else:
                cmd = f"CCW:PRESS:{speed}"
                
            self.send_motor_command(cmd)
            
            # Monitorear ángulo girado
    
    def send_motor_command(self, command):
        """Envía comando a los motores"""
        msg = String()
        msg.data = command
        self.motor_pub.publish(msg)
        self.get_logger().debug(f"📤 Motor: {command}")
    
    def control_loop(self):
        """Bucle principal de control"""
        if self.state == "IDLE" or not self.current_pose:
            return
        
        if self.state == "EXECUTING":
            if self.current_instruction_idx < len(self.instructions):
                instruction = self.instructions[self.current_instruction_idx]
                self.execute_instruction(instruction)
                # Verificar si se completó la instrucción
                if self.check_instruction_complete(instruction):
                    self.current_instruction_idx += 1
                    self.get_logger().info(
                        f"✅ Instrucción {self.current_instruction_idx}/{len(self.instructions)} completada"
                    )
            else:
                self.state = "COMPLETED"
                self.send_motor_command("STOP:RELEASE")
                self.get_logger().info("🎉 Ruta completada!")
    
    def check_instruction_complete(self, instruction):
        """Verifica si la instrucción actual se completó"""
        # Implementar lógica de verificación basada en odometría
        # Similar a la del path_follower_simple.py
        return False  # Placeholder
    
    def start_execution(self):
        """Inicia la ejecución de la ruta"""
        if not self.instructions:
            self.get_logger().warn("No hay instrucciones para ejecutar")
            return
        
        self.state = "EXECUTING"
        self.current_instruction_idx = 0
        self.get_logger().info("🚀 Iniciando ejecución de ruta RRT")
        self.publish_status("STARTED")
    
    def stop_execution(self):
        """Detiene la ejecución"""
        self.send_motor_command("STOP:RELEASE")
        self.state = "IDLE"
        self.get_logger().info("⏹️ Ejecución detenida")
        self.publish_status("STOPPED")
    
    def publish_status(self, status):
        """Publica estado"""
        msg = String()
        msg.data = f"{status}|{self.state}|{self.current_instruction_idx}/{len(self.instructions)}"
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    executor = RRTPathExecutor()
    
    # Ejemplo: iniciar automáticamente si hay archivo
    if executor.instructions_file:
        executor.start_execution()
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        executor.stop_execution()
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()