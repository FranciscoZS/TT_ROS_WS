#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from library_opi.pwm_controller import PWMOrangepi

class TestMotorNode(Node):
    def __init__(self):
        super().__init__('test_motor_node')
        
        # Inicializar motores
        self.motors = [
            PWMOrangepi(chip_number=2, pwm_number=0, pin_dir=21, dir=0, node=self),
            PWMOrangepi(chip_number=4, pwm_number=0, pin_dir=24, dir=0, node=self),
            PWMOrangepi(chip_number=5, pwm_number=0, pin_dir=26, dir=0, node=self),
            PWMOrangepi(chip_number=6, pwm_number=0, pin_dir=27, dir=0, node=self),
        ]
        
        # Configurar PWM
        for motor in self.motors:
            motor.setup()
            motor.configurePWM(2000, 0, True)  # 50Hz, 0% duty cycle inicial
        
        # Timer para prueba de motores
        self.test_timer = self.create_timer(5.0, self.run_test_sequence)
        self.test_step = 0
        
        self.get_logger().info('Nodo de prueba de motores inicializado')

    def run_test_sequence(self):
        steps = [
            (20, 0, 0, 0),   # Solo motor 0 al 20%
            (0, 20, 0, 0),   # Solo motor 1 al 20%
            (0, 0, 20, 0),   # Solo motor 2 al 20%
            (0, 0, 0, 20),   # Solo motor 3 al 20%
            (30, 30, 30, 30), # Todos al 30%
            (0, 0, 0, 0),    # Todos apagados
        ]
        
        if self.test_step < len(steps):
            duties = steps[self.test_step]
            for i, duty in enumerate(duties):
                self.motors[i].set_duty_cycle_optimized(duty)
                self.motors[i].change_dir(0)  # Dirección forward
            
            self.get_logger().info(f'Prueba paso {self.test_step}: Motores a {duties}%')
            self.test_step += 1
        else:
            # Terminar prueba
            self.test_timer.cancel()
            self.get_logger().info('Prueba de motores completada')
            
            # Apagar todos los motores
            for motor in self.motors:
                motor.set_duty_cycle_optimized(0)

    def destroy_node(self):
        for motor in self.motors:
            motor.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TestMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()