import wiringpi
import time
import math
import numpy as np

class MPU6050:
    def __init__(self, address=0x68, i2cbus=2, node=None):
        self.fd = wiringpi.wiringPiI2CSetupInterface(f"/dev/i2c-{i2cbus}", address)
        self.node = node
        
        # Estado Angular (confiable con gyro)
        self.theta = 0.0
        self.wz = 0.0
        
        # Calibración
        self.bias_ax = 0.0
        self.bias_ay = 0.0
        self.bias_wz = 0.0
        
        # Calibración adaptativa (para compensar deriva térmica)
        self.adaptive_bias_wz = 0.0
        self.static_samples = []
        self.max_static_samples = 50
        
        # Filtro Complementario para orientación
        self.alpha_complementary = 0.98  # Confianza en gyro vs accel
        
        # Filtro pasa-bajas para aceleraciones (antes de integrar)
        self.ax_filtered = 0.0
        self.ay_filtered = 0.0
        self.alpha_accel = 0.3  # Filtro más agresivo para aceleraciones
        
        # Buffer para detección de estático (validación estadística)
        self.gyro_buffer = []
        self.accel_buffer_x = []
        self.accel_buffer_y = []
        self.buffer_size = 10
        
        # Umbrales adaptativos
        self.GYRO_NOISE_THRESHOLD = 0.003  # rad/s (más conservador)
        self.ACCEL_NOISE_THRESHOLD = 0.1   # m/s^2
        
        self.last_time = time.perf_counter()
        
        # Configuración del sensor
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x6B, 0x00)  # Wake up
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x1B, 0x00)  # Gyro ±250°/s
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x1C, 0x00)  # Accel ±2g
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x1A, 0x05)  # DLPF 10Hz (crítico!)
        
        time.sleep(0.1)  # Estabilización
        
    def calibrate(self, duration=5.0):
        """Calibración con validación estadística"""
        if self.node: 
            self.node.get_logger().info("🔧 Calibrando IMU (NO MOVER)...")
        
        start = time.time()
        samples_ax, samples_ay, samples_wz = [], [], []
        
        while time.time() - start < duration:
            data = self._read_raw()
            samples_ax.append(data['ax'])
            samples_ay.append(data['ay'])
            samples_wz.append(data['wz'])
            time.sleep(0.01)
        
        # Calcular bias promedio
        self.bias_ax = np.mean(samples_ax)
        self.bias_ay = np.mean(samples_ay)
        self.bias_wz = np.mean(samples_wz)
        
        # Calcular varianza (para detección de ruido)
        std_ax = np.std(samples_ax)
        std_ay = np.std(samples_ay)
        std_wz = np.std(samples_wz)
        
        if self.node:
            self.node.get_logger().info(
                f"✅ Calibración completa:\n"
                f"   Bias Accel: ({self.bias_ax:.3f}, {self.bias_ay:.3f}) m/s²\n"
                f"   Bias Gyro: {self.bias_wz:.5f} rad/s\n"
                f"   Ruido (std): ax={std_ax:.3f}, ay={std_ay:.3f}, wz={std_wz:.5f}"
            )
    
    def _read_raw(self):
        """Lectura de registros con manejo de errores"""
        def read_word(reg):
            try:
                h = wiringpi.wiringPiI2CReadReg8(self.fd, reg)
                l = wiringpi.wiringPiI2CReadReg8(self.fd, reg+1)
                val = (h << 8) | l
                return val - 65536 if val > 32767 else val
            except:
                return 0
        
        return {
            'ax': read_word(0x3B) / 16384.0 * 9.81,  # m/s²
            'ay': read_word(0x3D) / 16384.0 * 9.81,
            'wz': read_word(0x47) / 131.0 * (math.pi/180.0)  # rad/s
        }
    
    def _is_static(self):
        """Detecta si el robot está estático mediante análisis estadístico"""
        if len(self.gyro_buffer) < self.buffer_size:
            return False
        
        # Varianza del gyro
        gyro_std = np.std(self.gyro_buffer)
        accel_std_x = np.std(self.accel_buffer_x)
        accel_std_y = np.std(self.accel_buffer_y)
        
        is_static = (
            gyro_std < self.GYRO_NOISE_THRESHOLD and
            accel_std_x < self.ACCEL_NOISE_THRESHOLD and
            accel_std_y < self.ACCEL_NOISE_THRESHOLD
        )
        
        return is_static
    
    def update(self):
        """Actualización con fusión de sensores y filtrado adaptativo"""
        now = time.perf_counter()
        dt = now - self.last_time
        self.last_time = now
        
        if dt > 0.5:  # Prevenir saltos temporales
            dt = 0.05
        
        raw = self._read_raw()
        
        # ========== 1. PREPROCESAMIENTO ==========
        # Remover bias
        acc_x_raw = raw['ax'] - self.bias_ax
        acc_y_raw = raw['ay'] - self.bias_ay
        gyro_z_raw = raw['wz'] - self.bias_wz
        
        # Actualizar buffers para detección de estático
        self.gyro_buffer.append(abs(gyro_z_raw))
        self.accel_buffer_x.append(abs(acc_x_raw))
        self.accel_buffer_y.append(abs(acc_y_raw))
        
        if len(self.gyro_buffer) > self.buffer_size:
            self.gyro_buffer.pop(0)
            self.accel_buffer_x.pop(0)
            self.accel_buffer_y.pop(0)
        
        # ========== 2. CALIBRACIÓN ADAPTATIVA ==========
        # Si está estático, recalibrar bias de gyro (deriva térmica)
        if self._is_static():
            self.static_samples.append(gyro_z_raw)
            if len(self.static_samples) > self.max_static_samples:
                self.static_samples.pop(0)
            
            # Actualizar bias adaptativo
            if len(self.static_samples) >= 20:
                self.adaptive_bias_wz = np.mean(self.static_samples)
        else:
            self.static_samples = []
        
        # Aplicar bias adaptativo
        gyro_z = gyro_z_raw - self.adaptive_bias_wz
        
        # ========== 3. FILTRADO DE SEÑALES ==========
        # Filtro pasa-bajas para aceleraciones
        self.ax_filtered = self.alpha_accel * acc_x_raw + (1 - self.alpha_accel) * self.ax_filtered
        self.ay_filtered = self.alpha_accel * acc_y_raw + (1 - self.alpha_accel) * self.ay_filtered
        
        # Deadzone adaptativo para gyro (solo si está casi estático)
        if abs(gyro_z) < self.GYRO_NOISE_THRESHOLD and self._is_static():
            gyro_z = 0.0
        
        # Deadzone para aceleraciones (solo si está muy estático)
        acc_x = self.ax_filtered if abs(self.ax_filtered) > 0.08 else 0.0
        acc_y = self.ay_filtered if abs(self.ay_filtered) > 0.08 else 0.0
        
        # ========== 4. INTEGRACIÓN ANGULAR (CONFIABLE) ==========
        # El gyro es muy confiable para orientación
        self.wz = gyro_z
        self.theta += self.wz * dt
        
        # Normalizar ángulo
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # ========== 5. RETORNO DE DATOS ==========
        # NO integramos aceleraciones aquí (se hace en el EKF)
        return {
            'w_z': self.wz,              # Velocidad angular [rad/s]
            'theta': self.theta,          # Orientación [rad]
            'acc_x': acc_x,               # Aceleración X filtrada [m/s²]
            'acc_y': acc_y,               # Aceleración Y filtrada [m/s²]
            'is_static': self._is_static()
        }
    
    def get_orientation(self):
        """Retorna solo la orientación (la parte confiable del IMU)"""
        return self.theta
    
    def get_angular_velocity(self):
        """Retorna velocidad angular"""
        return self.wz
