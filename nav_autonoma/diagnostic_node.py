#!/usr/bin/env python3
"""
Nodo de diagnóstico para calibrar encoders y cinemática mecanum
Ejecutar ANTES de usar el sistema de odometría
"""

import rclpy
from rclpy.node import Node
from library_opi.encoders import OpticalEncoder
from library_opi.mpu6050_improved import MPU6050
import time
import math

class DiagnosticCalibrationNode(Node):
    def __init__(self):
        super().__init__('diagnostic_calibration_node')
        
        # Parámetros del robot
        self.wheel_radius = 0.05  # metros
        self.lx = 0.15
        self.ly = 0.14
        
        # Inicializar encoders (SIN invert por ahora, lo calibramos)
        self.get_logger().info("🔧 Inicializando encoders...")
        self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert=False)
        self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert=False)
        self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert=False)
        self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert=False)
        
        # Inicializar IMU
        self.get_logger().info("🔧 Inicializando IMU...")
        self.imu = MPU6050(node=self)
        
        self.get_logger().info("✅ Hardware inicializado\n")

    def test_encoder_direction(self):
        """
        Prueba 1: Verificar dirección de encoders
        INSTRUCCIONES: Mover MANUALMENTE cada rueda HACIA ADELANTE
        """
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("📋 PRUEBA 1: DIRECCIÓN DE ENCODERS")
        self.get_logger().info("="*70)
        self.get_logger().info("INSTRUCCIONES:")
        self.get_logger().info("1. Levanta el robot (ruedas en el aire)")
        self.get_logger().info("2. Gira MANUALMENTE cada rueda HACIA ADELANTE (dirección de avance)")
        self.get_logger().info("3. Observa si el contador AUMENTA (+) o DISMINUYE (-)")
        self.get_logger().info("")
        self.get_logger().info("Para mecanum, 'ADELANTE' significa:")
        self.get_logger().info("  - Ruedas IZQUIERDAS: rodillos van hacia ATRÁS-DERECHA")
        self.get_logger().info("  - Ruedas DERECHAS: rodillos van hacia ADELANTE-DERECHA")
        self.get_logger().info("")
        input("Presiona ENTER cuando estés listo...")
        
        # Resetear contadores
        self.enc_fl.counter = 0
        self.enc_fr.counter = 0
        self.enc_rl.counter = 0
        self.enc_rr.counter = 0
        
        input("\n🔹 Gira FRONTAL IZQUIERDA hacia adelante, luego ENTER...")
        fl_count = self.enc_fl.counter
        self.get_logger().info(f"   FL: {fl_count:+6d} {'✅ CORRECTO' if fl_count > 0 else '❌ INVERTIR'}")
        
        self.enc_fr.counter = 0
        input("\n🔹 Gira FRONTAL DERECHA hacia adelante, luego ENTER...")
        fr_count = self.enc_fr.counter
        self.get_logger().info(f"   FR: {fr_count:+6d} {'✅ CORRECTO' if fr_count > 0 else '❌ INVERTIR'}")
        
        self.enc_rl.counter = 0
        input("\n🔹 Gira TRASERA IZQUIERDA hacia adelante, luego ENTER...")
        rl_count = self.enc_rl.counter
        self.get_logger().info(f"   RL: {rl_count:+6d} {'✅ CORRECTO' if rl_count > 0 else '❌ INVERTIR'}")
        
        self.enc_rr.counter = 0
        input("\n🔹 Gira TRASERA DERECHA hacia adelante, luego ENTER...")
        rr_count = self.enc_rr.counter
        self.get_logger().info(f"   RR: {rr_count:+6d} {'✅ CORRECTO' if rr_count > 0 else '❌ INVERTIR'}")
        
        # Generar código de configuración
        self.get_logger().info("\n📝 CONFIGURACIÓN SUGERIDA:")
        self.get_logger().info("En dual_odometry_ekf_node.py, usa:")
        self.get_logger().info(f"  self.enc_fl = OpticalEncoder(4, 6, reduction_ratio=6, invert={fl_count < 0})")
        self.get_logger().info(f"  self.enc_fr = OpticalEncoder(9, 10, reduction_ratio=6, invert={fr_count < 0})")
        self.get_logger().info(f"  self.enc_rl = OpticalEncoder(13, 15, reduction_ratio=6, invert={rl_count < 0})")
        self.get_logger().info(f"  self.enc_rr = OpticalEncoder(16, 18, reduction_ratio=6, invert={rr_count < 0})")

    def test_encoder_counts(self):
        """
        Prueba 2: Verificar PPR efectivo
        """
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("📋 PRUEBA 2: PULSOS POR REVOLUCIÓN (PPR)")
        self.get_logger().info("="*70)
        self.get_logger().info("INSTRUCCIONES:")
        self.get_logger().info("1. Marca una posición en la rueda frontal izquierda")
        self.get_logger().info("2. Gira la rueda EXACTAMENTE 1 VUELTA COMPLETA (360°)")
        self.get_logger().info("")
        
        self.enc_fl.counter = 0
        input("Presiona ENTER para empezar a contar...")
        initial = self.enc_fl.counter
        
        input("Gira 1 vuelta completa, luego presiona ENTER...")
        final = self.enc_fl.counter
        pulses = abs(final - initial)
        
        self.get_logger().info(f"\n   Pulsos contados: {pulses}")
        self.get_logger().info(f"   PPR configurado: {self.enc_fl.ppr_wheel:.0f}")
        
        if abs(pulses - self.enc_fl.ppr_wheel) > 100:
            suggested_ratio = pulses / 1000.0
            self.get_logger().warn(f"   ⚠️  DIFERENCIA DETECTADA!")
            self.get_logger().info(f"   📝 Usa: reduction_ratio={suggested_ratio:.1f}")
        else:
            self.get_logger().info(f"   ✅ PPR correcto")

    def test_wheel_radius(self):
        """
        Prueba 3: Calibrar radio de rueda
        """
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("📋 PRUEBA 3: RADIO DE RUEDA")
        self.get_logger().info("="*70)
        self.get_logger().info("INSTRUCCIONES:")
        self.get_logger().info("1. Coloca el robot en el suelo")
        self.get_logger().info("2. Marca la posición inicial")
        self.get_logger().info("3. EMPUJA el robot RECTO hacia adelante EXACTAMENTE 1 METRO")
        self.get_logger().info("")
        
        input("Presiona ENTER para empezar...")
        
        # Resetear contadores
        self.enc_fl.counter = 0
        self.enc_fr.counter = 0
        self.enc_rl.counter = 0
        self.enc_rr.counter = 0
        
        input("Empuja 1 metro, luego presiona ENTER...")
        
        # Calcular pulsos promedio
        avg_pulses = (abs(self.enc_fl.counter) + abs(self.enc_fr.counter) + 
                      abs(self.enc_rl.counter) + abs(self.enc_rr.counter)) / 4.0
        
        # Calcular distancia con radio actual
        revolutions = avg_pulses / self.enc_fl.ppr_wheel
        distance_calculated = revolutions * 2 * math.pi * self.wheel_radius
        
        # Calcular radio real
        real_distance = 1.0  # metros
        real_radius = real_distance / (revolutions * 2 * math.pi)
        
        self.get_logger().info(f"\n   Pulsos promedio: {avg_pulses:.0f}")
        self.get_logger().info(f"   Revoluciones: {revolutions:.2f}")
        self.get_logger().info(f"   Distancia calculada: {distance_calculated:.3f} m")
        self.get_logger().info(f"   Distancia real: {real_distance} m")
        self.get_logger().info(f"   Radio configurado: {self.wheel_radius:.4f} m")
        self.get_logger().info(f"   📝 Radio real medido: {real_radius:.4f} m")
        
        if abs(real_radius - self.wheel_radius) > 0.005:
            self.get_logger().warn(f"   ⚠️  AJUSTAR RADIO!")
            self.get_logger().info(f"   Usa: wheel_radius={real_radius:.4f}")

    def test_imu_calibration(self):
        """
        Prueba 4: Verificar calibración IMU
        """
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("📋 PRUEBA 4: CALIBRACIÓN IMU")
        self.get_logger().info("="*70)
        self.get_logger().info("INSTRUCCIONES:")
        self.get_logger().info("1. Coloca el robot COMPLETAMENTE QUIETO")
        self.get_logger().info("2. NO LO TOQUES durante 5 segundos")
        self.get_logger().info("")
        
        input("Presiona ENTER para calibrar...")
        self.imu.calibrate(duration=5.0)
        
        # Verificar estabilidad
        self.get_logger().info("\n📊 Verificando estabilidad (5 segundos)...")
        samples_ax, samples_ay, samples_wz = [], [], []
        
        for _ in range(100):
            data = self.imu.update()
            samples_ax.append(data['acc_x'])
            samples_ay.append(data['acc_y'])
            samples_wz.append(data['w_z'])
            time.sleep(0.05)
        
        import numpy as np
        std_ax = np.std(samples_ax)
        std_ay = np.std(samples_ay)
        std_wz = np.std(samples_wz)
        
        self.get_logger().info(f"   Ruido aceleración X: {std_ax:.4f} m/s²")
        self.get_logger().info(f"   Ruido aceleración Y: {std_ay:.4f} m/s²")
        self.get_logger().info(f"   Ruido gyro Z: {std_wz:.6f} rad/s")
        
        if std_wz > 0.01:
            self.get_logger().warn("   ⚠️  Ruido alto en gyro!")
            self.get_logger().info("   Considera aumentar GYRO_NOISE_THRESHOLD")

    def test_kinematic_model(self):
        """
        Prueba 5: Verificar cinemática mecanum
        """
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("📋 PRUEBA 5: CINEMÁTICA MECANUM")
        self.get_logger().info("="*70)
        self.get_logger().info("Vamos a probar cada dirección de movimiento:")
        self.get_logger().info("")
        
        tests = [
            ("ADELANTE", "Empuja el robot RECTO hacia adelante", "vx+"),
            ("ATRÁS", "Empuja el robot RECTO hacia atrás", "vx-"),
            ("DERECHA", "Empuja el robot LATERAL a la DERECHA", "vy-"),
            ("IZQUIERDA", "Empuja el robot LATERAL a la IZQUIERDA", "vy+"),
        ]
        
        for name, instruction, expected in tests:
            self.get_logger().info(f"\n🔹 Prueba: {name}")
            self.get_logger().info(f"   {instruction}")
            
            # Resetear
            self.enc_fl.counter = 0
            self.enc_fr.counter = 0
            self.enc_rl.counter = 0
            self.enc_rr.counter = 0
            
            input("   Presiona ENTER, mueve el robot, luego ENTER de nuevo...")
            input("   (Moviendo...)")
            
            # Leer encoders
            counts = [
                self.enc_fl.counter,
                self.enc_fr.counter,
                self.enc_rl.counter,
                self.enc_rr.counter
            ]
            
            self.get_logger().info(f"   Contadores: FL={counts[0]:+6d}, FR={counts[1]:+6d}, RL={counts[2]:+6d}, RR={counts[3]:+6d}")
            
            # Cinemática esperada para cada dirección
            if "vx+" in expected:
                expected_signs = ["+", "+", "+", "+"]
            elif "vx-" in expected:
                expected_signs = ["-", "-", "-", "-"]
            elif "vy-" in expected:  # Derecha
                expected_signs = ["+", "-", "-", "+"]
            elif "vy+" in expected:  # Izquierda
                expected_signs = ["-", "+", "+", "-"]
            
            actual_signs = ['+' if c >= 0 else '-' for c in counts]
            
            if actual_signs == expected_signs:
                self.get_logger().info(f"   ✅ Signos correctos: {expected_signs}")
            else:
                self.get_logger().warn(f"   ❌ Signos incorrectos!")
                self.get_logger().info(f"      Esperado: {expected_signs}")
                self.get_logger().info(f"      Obtenido: {actual_signs}")

    def run_all_diagnostics(self):
        """Ejecuta todas las pruebas"""
        self.get_logger().info("\n")
        self.get_logger().info("="*70)
        self.get_logger().info("🔬 DIAGNÓSTICO COMPLETO DEL SISTEMA")
        self.get_logger().info("="*70)
        self.get_logger().info("")
        self.get_logger().info("Este diagnóstico te ayudará a calibrar correctamente:")
        self.get_logger().info("  1. Dirección de encoders (invert)")
        self.get_logger().info("  2. PPR efectivo (reduction_ratio)")
        self.get_logger().info("  3. Radio de ruedas")
        self.get_logger().info("  4. Calibración del IMU")
        self.get_logger().info("  5. Cinemática mecanum")
        self.get_logger().info("")
        
        try:
            self.test_encoder_direction()
            input("\n▶️  Presiona ENTER para continuar con la siguiente prueba...")
            
            self.test_encoder_counts()
            input("\n▶️  Presiona ENTER para continuar...")
            
            self.test_wheel_radius()
            input("\n▶️  Presiona ENTER para continuar...")
            
            self.test_imu_calibration()
            input("\n▶️  Presiona ENTER para continuar...")
            
            self.test_kinematic_model()
            
            self.get_logger().info("\n" + "="*70)
            self.get_logger().info("✅ DIAGNÓSTICO COMPLETADO")
            self.get_logger().info("="*70)
            self.get_logger().info("Revisa los resultados y ajusta los parámetros según las sugerencias.")
            
        except KeyboardInterrupt:
            self.get_logger().info("\n⚠️  Diagnóstico interrumpido")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticCalibrationNode()
    
    try:
        node.run_all_diagnostics()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
