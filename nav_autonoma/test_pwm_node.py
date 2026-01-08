#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import subprocess

from library_opi.pwm_controller import PWMOrangepi

class TestPWMSimpleNode(Node):
    def __init__(self):
        super().__init__('test_pwm_node')
        
        self.get_logger().info('🚀 Inicializando nodo de prueba PWM simple...')
        
        # Inicializar motores (usando tus pines)
        self.motors = [
            PWMOrangepi(chip_number=2, pwm_number=0, pin_dir=21, dir=0, node=self),  # FL
            PWMOrangepi(chip_number=4, pwm_number=0, pin_dir=24, dir=0, node=self),  # FR  
            PWMOrangepi(chip_number=5, pwm_number=0, pin_dir=26, dir=0, node=self),  # RL
            PWMOrangepi(chip_number=6, pwm_number=0, pin_dir=27, dir=0, node=self),  # RR
        ]
        
        # Configurar PWM de los motores
        for i, motor in enumerate(self.motors):
            try:
                motor.setup()
                motor.configurePWM(2000, 0, True)  # 50Hz, 0% duty cycle inicial
                self.get_logger().info(f'✅ Motor {i} configurado')
            except Exception as e:
                self.get_logger().error(f'❌ Error configurando motor {i}: {e}')
        
        # 🔧 SECUENCIA SIMPLE: Activar todos al 50% como tu ejemplo
        self.get_logger().info('🎯 Activando todos los motores al 50%...')
        
        try:
            # Aplicar 50% a todos los motores (como tu código original)
            for i, motor in enumerate(self.motors):
                motor.configurePWM(2000, 50, True)  # 50Hz, 50% duty cycle
                motor.change_dir(0)  # Dirección forward
                self.get_logger().info(f'✅ Motor {i} activado al 50%')
            
            # Mostrar estado continuamente
            self.get_logger().info('📊 Monitoreo activo - Usa Ctrl+C para detener')
            self.status_timer = self.create_timer(2.0, self.log_motor_status)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error activando motores: {e}')
            self.cleanup_and_shutdown()

    def log_motor_status(self):
        """Mostrar estado actual de todos los motores"""
        status = "📊 Motores activos: "
        for i, motor in enumerate(self.motors):
            status += f"M{i}:{motor.duty_cycle}% "
        self.get_logger().info(status)

    def cleanup_and_shutdown(self):
        """Apagar todos los motores y cerrar nodo"""
        self.get_logger().info('🛑 Apagando todos los motores...')
        
        # Apagar todos los motores
        for i, motor in enumerate(self.motors):
            try:
                motor.set_duty_cycle_optimized(0)
                self.get_logger().info(f'✅ Motor {i} apagado')
            except Exception as e:
                self.get_logger().error(f'❌ Error apagando motor {i}: {e}')
        
        # Cleanup
        for i, motor in enumerate(self.motors):
            try:
                motor.cleanup()
                self.get_logger().info(f'✅ Motor {i} limpiado')
            except Exception as e:
                self.get_logger().error(f'❌ Error limpiando motor {i}: {e}')
        
        self.get_logger().info('🎯 Prueba PWM terminada')
        
        # Cerrar el nodo después de 1 segundo
        self.create_timer(1.0, self.shutdown_node)

    def shutdown_node(self):
        """Cerrar el nodo"""
        rclpy.shutdown()

    def destroy_node(self):
        """Cleanup al destruir el nodo (por si acaso)"""
        self.get_logger().info('🔧 Ejecutando cleanup final...')
        for motor in self.motors:
            try:
                motor.set_duty_cycle_optimized(0)
                motor.cleanup()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TestPWMSimpleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⚠️ Interrupción por teclado - Apagando motores...')
        node.cleanup_and_shutdown()
    except Exception as e:
        node.get_logger().error(f'💥 Error en main: {e}')
        node.cleanup_and_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()