import numpy as np
import math

class ExtendedKalmanFilter:
    """
    Filtro de Kalman Extendido para fusión de sensores en robot mecanum
    
    Estado: [x, y, theta, vx, vy, omega]
    - x, y: Posición en el plano [m]
    - theta: Orientación [rad]
    - vx, vy: Velocidades lineales en frame del robot [m/s]
    - omega: Velocidad angular [rad/s]
    
    Sensores:
    - Encoders: Proveen velocidades (vx, vy, omega)
    - Gyro: Provee omega (más confiable que encoders)
    - Acelerómetros: Corrección suave de velocidades
    """
    
    def __init__(self, wheel_radius, lx, ly):
        """
        Inicializa el EKF
        
        Args:
            wheel_radius: Radio de las ruedas [m]
            lx: Distancia del centro al eje X de las ruedas [m]
            ly: Distancia del centro al eje Y de las ruedas [m]
        """
        self.wheel_radius = wheel_radius
        self.lx = lx
        self.ly = ly
        
        # ========== ESTADO ==========
        # Vector de estado: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        
        # ========== MATRIZ DE COVARIANZA ==========
        # Incertidumbre del estado (P)
        # Valores iniciales pequeños = confianza alta
        self.P = np.eye(6) * 0.1
        
        # ========== RUIDO DEL PROCESO (Q) ==========
        # Representa cuánto confiamos en el modelo de predicción
        # Valores más altos = menos confianza en el modelo
        self.Q = np.diag([
            0.001,  # x - Posición X
            0.001,  # y - Posición Y
            0.0005, # theta - Orientación (muy confiable con gyro)
            0.05,   # vx - Velocidad X (puede variar rápido)
            0.05,   # vy - Velocidad Y (puede variar rápido)
            0.01    # omega - Velocidad angular
        ])
        
        # ========== RUIDO DE MEDICIÓN ENCODERS (R_encoders) ==========
        # Representa cuánto confiamos en las mediciones de encoders
        # Valores más altos = menos confianza en encoders
        self.R_encoders = np.diag([
            0.02,  # vx - Razonablemente confiable
            0.03,  # vy - Menos confiable (patinaje lateral en mecanum)
            0.02   # omega - Mejorado con gyro pero no perfecto
        ])
        
        # ========== RUIDO DE MEDICIÓN GYRO (R_gyro) ==========
        # El gyro es muy preciso para velocidad angular
        self.R_gyro = np.array([[0.001]])
        
        # ========== GANANCIA DE ACELERACIONES ==========
        # Para corrección suave (no integración directa)
        self.accel_gain = 0.1
        
    def predict(self, dt):
        """
        Paso de predicción del EKF
        Predice el siguiente estado usando el modelo cinemático
        
        Args:
            dt: Intervalo de tiempo [s]
        """
        # Extraer estado actual
        x, y, theta, vx, vy, omega = self.state
        
        # ========== MODELO CINEMÁTICO ==========
        # Transformar velocidades del frame del robot al frame global
        vx_global = vx * math.cos(theta) - vy * math.sin(theta)
        vy_global = vx * math.sin(theta) + vy * math.cos(theta)
        
        # Integrar posición
        x_new = x + vx_global * dt
        y_new = y + vy_global * dt
        theta_new = theta + omega * dt
        
        # Normalizar theta a rango [-π, π]
        theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
        
        # Modelo de fricción/decaimiento de velocidades
        # (Aproxima la fricción del robot con el suelo)
        decay_linear = 0.95   # Fricción lineal
        decay_angular = 0.98  # Fricción angular
        
        vx_new = vx * decay_linear
        vy_new = vy * decay_linear
        omega_new = omega * decay_angular
        
        # Actualizar estado predicho
        self.state = np.array([x_new, y_new, theta_new, vx_new, vy_new, omega_new])
        
        # ========== JACOBIANO DEL MODELO (F) ==========
        # Linealización del modelo no lineal
        # F = ∂f/∂x donde f es el modelo de movimiento
        
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)
        
        F = np.array([
            [1, 0, -vx*sin_theta*dt - vy*cos_theta*dt, cos_theta*dt, -sin_theta*dt, 0],
            [0, 1,  vx*cos_theta*dt - vy*sin_theta*dt, sin_theta*dt,  cos_theta*dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, decay_linear, 0, 0],
            [0, 0, 0, 0, decay_linear, 0],
            [0, 0, 0, 0, 0, decay_angular]
        ])
        
        # ========== ACTUALIZAR COVARIANZA ==========
        # P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q
        
    def update_encoders(self, vx_enc, vy_enc, omega_enc):
        """
        Paso de actualización con mediciones de encoders
        
        Args:
            vx_enc: Velocidad X medida por encoders [m/s]
            vy_enc: Velocidad Y medida por encoders [m/s]
            omega_enc: Velocidad angular medida por encoders [rad/s]
        """
        # ========== MEDICIÓN ESPERADA ==========
        # Predicción de lo que deberían medir los sensores
        z_pred = np.array([self.state[3], self.state[4], self.state[5]])
        
        # ========== MEDICIÓN REAL ==========
        z_meas = np.array([vx_enc, vy_enc, omega_enc])
        
        # ========== MATRIZ DE OBSERVACIÓN (H) ==========
        # H = ∂h/∂x donde h es el modelo de medición
        # Los encoders miden directamente las velocidades
        H = np.array([
            [0, 0, 0, 1, 0, 0],  # vx está en posición 3 del estado
            [0, 0, 0, 0, 1, 0],  # vy está en posición 4 del estado
            [0, 0, 0, 0, 0, 1]   # omega está en posición 5 del estado
        ])
        
        # ========== INNOVACIÓN ==========
        # Diferencia entre medición real y predicha
        y = z_meas - z_pred
        
        # ========== MATRIZ DE INNOVACIÓN ==========
        # S = H * P * H^T + R
        S = H @ self.P @ H.T + self.R_encoders
        
        # ========== GANANCIA DE KALMAN ==========
        # K = P * H^T * S^-1
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # ========== ACTUALIZAR ESTADO ==========
        # x = x + K * y
        self.state += K @ y
        
        # ========== ACTUALIZAR COVARIANZA ==========
        # P = (I - K * H) * P
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
        
    def update_gyro(self, omega_gyro):
        """
        Paso de actualización con medición del gyro
        El gyro es muy confiable para velocidad angular
        
        Args:
            omega_gyro: Velocidad angular medida por gyro [rad/s]
        """
        # ========== MEDICIÓN ESPERADA ==========
        z_pred = np.array([self.state[5]])
        
        # ========== MEDICIÓN REAL ==========
        z_meas = np.array([omega_gyro])
        
        # ========== MATRIZ DE OBSERVACIÓN (H) ==========
        # El gyro mide directamente omega (posición 5 del estado)
        H = np.array([[0, 0, 0, 0, 0, 1]])
        
        # ========== INNOVACIÓN ==========
        y = z_meas - z_pred
        
        # ========== MATRIZ DE INNOVACIÓN ==========
        S = H @ self.P @ H.T + self.R_gyro
        
        # ========== GANANCIA DE KALMAN ==========
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # ========== ACTUALIZAR ESTADO ==========
        self.state += K @ y
        
        # ========== ACTUALIZAR COVARIANZA ==========
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
    
    def correct_with_accelerations(self, acc_x, acc_y, dt):
        """
        Corrección suave con aceleraciones
        NO es una actualización completa de Kalman, sino una corrección heurística
        
        IMPORTANTE: No integramos directamente porque la doble integración
        del IMU causa drift exponencial. Solo usamos las aceleraciones para
        ajustar suavemente las velocidades.
        
        Args:
            acc_x: Aceleración X en frame del robot [m/s²]
            acc_y: Aceleración Y en frame del robot [m/s²]
            dt: Intervalo de tiempo [s]
        """
        # ========== TRANSFORMAR A FRAME GLOBAL ==========
        theta = self.state[2]
        
        acc_x_global = acc_x * math.cos(theta) - acc_y * math.sin(theta)
        acc_y_global = acc_x * math.sin(theta) + acc_y * math.cos(theta)
        
        # ========== CORRECCIÓN SUAVE ==========
        # Usamos una ganancia pequeña (accel_gain) para evitar sobrecorrección
        # Esto ayuda a reducir el drift sin causar inestabilidad
        
        self.state[3] += acc_x_global * dt * self.accel_gain
        self.state[4] += acc_y_global * dt * self.accel_gain
        
        # Limitar velocidades para evitar valores explosivos
        max_vel = 2.0  # m/s (ajustar según tu robot)
        self.state[3] = np.clip(self.state[3], -max_vel, max_vel)
        self.state[4] = np.clip(self.state[4], -max_vel, max_vel)
        
    def get_state(self):
        """
        Retorna el estado completo como diccionario
        
        Returns:
            dict: Estado actual con claves x, y, theta, vx, vy, omega
        """
        return {
            'x': self.state[0],
            'y': self.state[1],
            'theta': self.state[2],
            'vx': self.state[3],
            'vy': self.state[4],
            'omega': self.state[5]
        }
    
    def get_covariance(self):
        """
        Retorna la diagonal de la matriz de covarianza
        (incertidumbre de cada variable de estado)
        
        Returns:
            np.array: [σ²_x, σ²_y, σ²_theta, σ²_vx, σ²_vy, σ²_omega]
        """
        return np.diag(self.P)
    
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """
        Reinicia el estado del filtro
        
        Args:
            x: Posición X inicial [m]
            y: Posición Y inicial [m]
            theta: Orientación inicial [rad]
        """
        self.state = np.array([x, y, theta, 0.0, 0.0, 0.0])
        self.P = np.eye(6) * 0.1
    
    def set_process_noise(self, q_pos, q_theta, q_vel):
        """
        Ajusta el ruido del proceso (Q)
        
        Args:
            q_pos: Ruido de posición [m²]
            q_theta: Ruido de orientación [rad²]
            q_vel: Ruido de velocidades [m²/s² o rad²/s²]
        """
        self.Q = np.diag([q_pos, q_pos, q_theta, q_vel, q_vel, q_vel])
    
    def set_measurement_noise_encoders(self, r_vx, r_vy, r_omega):
        """
        Ajusta el ruido de medición de encoders (R_encoders)
        
        Args:
            r_vx: Ruido en vx [m²/s²]
            r_vy: Ruido en vy [m²/s²]
            r_omega: Ruido en omega [rad²/s²]
        """
        self.R_encoders = np.diag([r_vx, r_vy, r_omega])
    
    def set_measurement_noise_gyro(self, r_gyro):
        """
        Ajusta el ruido de medición del gyro (R_gyro)
        
        Args:
            r_gyro: Ruido del gyro [rad²/s²]
        """
        self.R_gyro = np.array([[r_gyro]])
