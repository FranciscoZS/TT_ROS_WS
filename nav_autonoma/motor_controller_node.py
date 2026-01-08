#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

# Importar tus librerías PWM
from library_opi.pwm_controller import PWMOrangepi

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Parámetros del robot (ajustar con tus valores)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base_x', 0.3)
        self.declare_parameter('wheel_base_y', 0.4)
        self.declare_parameter('max_rpm', 300.0)
        self.declare_parameter('min_duty_cycle', 20.0)  # Mínimo duty cycle funcional
        self.declare_parameter('max_duty_cycle', 80.0)  # Máximo duty cycle
        
        # Inicializar controladores PWM (usando tus pines)
        self.motors = [
            PWMOrangepi(chip_number=2, pwm_number=0, pin_dir=21, dir=0, node=self),  # FL
            PWMOrangepi(chip_number=4, pwm_number=0, pin_dir=24, dir=0, node=self),  # FR
            PWMOrangepi(chip_number=5, pwm_number=0, pin_dir=26, dir=0, node=self),  # RL
            PWMOrangepi(chip_number=6, pwm_number=0, pin_dir=27, dir=0, node=self),  # RR
        ]
        
        # Configurar PWM de los motores
        for motor in self.motors:
            motor.setup()
            motor.configurePWM(2000, 0, True)  # 50Hz, 0% duty cycle inicial
        
        # Subscriber para comandos de velocidad
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # Variables de control
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # Timeout de comandos (segundos)
        
        # Timer de seguridad
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Motor Controller Node inicializado')

    def cmd_vel_callback(self, msg):
        """Convertir Twist a comandos PWM para motores mecanum"""
        self.last_cmd_time = self.get_clock().now()
        
        # Aplicar límites de seguridad
        vx = max(min(msg.linear.x, 1.0), -1.0)    # m/s
        vy = max(min(msg.linear.y, 1.0), -1.0)    # m/s
        wz = max(min(msg.angular.z, 2.0), -2.0)   # rad/s
        
        # Cinemática inversa para ruedas mecanum
        wheel_speeds = self.mecanum_inverse_kinematics(vx, vy, wz)
        
        # Convertir a duty cycles y enviar a motores
        for i, speed in enumerate(wheel_speeds):
            duty_cycle = self.speed_to_duty_cycle(speed)
            self.motors[i].set_duty_cycle_optimized(duty_cycle)
            
            # Control de dirección
            if speed >= 0:
                self.motors[i].change_dir(0)
            else:
                self.motors[i].change_dir(1)

    def mecanum_inverse_kinematics(self, vx, vy, wz):
        """
        Cinemática inversa para robot mecanum
        Retorna velocidades de rueda en m/s
        """
        L = self.get_parameter('wheel_base_x').value / 2.0
        W = self.get_parameter('wheel_base_y').value / 2.0
        R = self.get_parameter('wheel_radius').value
        
        # Matriz de transformación para mecanum
        wheel_fl = (vx - vy - (L + W) * wz) / R  # Delantera izquierda
        wheel_fr = (vx + vy + (L + W) * wz) / R  # Delantera derecha  
        wheel_rl = (vx + vy - (L + W) * wz) / R  # Trasera izquierda
        wheel_rr = (vx - vy + (L + W) * wz) / R  # Trasera derecha
        
        return [wheel_fl, wheel_fr, wheel_rl, wheel_rr]

    def speed_to_duty_cycle(self, speed):
        """Convertir velocidad lineal (m/s) a duty cycle (%)"""
        max_rpm = self.get_parameter('max_rpm').value
        wheel_radius = self.get_parameter('wheel_radius').value
        
        # Convertir m/s a RPM
        max_linear_speed = (max_rpm * 2 * math.pi * wheel_radius) / 60.0
        speed_ratio = speed / max_linear_speed
        
        # Limitar ratio entre -1 y 1
        speed_ratio = max(min(speed_ratio, 1.0), -1.0)
        
        # Mapear a duty cycle
        min_duty = self.get_parameter('min_duty_cycle').value
        max_duty = self.get_parameter('max_duty_cycle').value
        
        if speed_ratio >= 0:
            duty_cycle = min_duty + (max_duty - min_duty) * abs(speed_ratio)
        else:
            duty_cycle = min_duty + (max_duty - min_duty) * abs(speed_ratio)
            
        return duty_cycle

    def safety_check(self):
        """Detener motores si no hay comandos recientes"""
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.cmd_timeout:
            # Timeout - detener motores
            for motor in self.motors:
                motor.set_duty_cycle_optimized(0)
            #self.get_logger().warn('Timeout de comando - Motores detenidos')

    def destroy_node(self):
        """Cleanup al cerrar"""
        for motor in self.motors:
            motor.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()