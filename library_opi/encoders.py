import wiringpi
from wiringpi import GPIO
import time
import threading
import math

wiringpi.wiringPiSetup()

class OpticalEncoder:
    def __init__(self, pin_a, pin_b, ppr=1000, reduction_ratio=6.0, invert=False, sample_time=0.01):
        self.pin_a = pin_a
        self.pin_b = pin_b
        # PPR efectivo en la salida de la rueda = PPR_encoder * Relación
        self.ppr_wheel = int(ppr * reduction_ratio)  # Convertir a entero
        self.invert = -1 if invert else 1
        self.sample_time = sample_time  # Tiempo de muestreo para cálculo RPM
        
        # Contadores
        self.counter = 0
        self.last_counter = 0
        self.last_time = time.perf_counter()
        self.current_rpm_wheel = 0.0
        self.lock = threading.Lock()
        
        # Para detección de dirección (decodificación cuadratura)
        self.last_state_a = 0
        self.last_state_b = 0
        self.last_encoded = 0
        
        # Historial para filtrado
        self.rpm_history = []
        self.max_history = 3
        
        # Configuración WiringPi
        wiringpi.pinMode(self.pin_a, GPIO.INPUT)
        wiringpi.pinMode(self.pin_b, GPIO.INPUT)
        wiringpi.pullUpDnControl(self.pin_a, GPIO.PUD_UP)
        wiringpi.pullUpDnControl(self.pin_b, GPIO.PUD_UP)

        # Leer estado inicial
        self.last_state_a = wiringpi.digitalRead(self.pin_a)
        self.last_state_b = wiringpi.digitalRead(self.pin_b)
        
        # Configurar interrupciones para ambos flancos
        wiringpi.wiringPiISR(self.pin_a, GPIO.INT_EDGE_BOTH, self._isr_callback_a)
        wiringpi.wiringPiISR(self.pin_b, GPIO.INT_EDGE_BOTH, self._isr_callback_b)
        
        print(f"Encoder inicializado: PPR={ppr}, Reducción={reduction_ratio}, PPR_rueda={self.ppr_wheel}")

    def _update_encoder(self):
        """Decodificación cuadratura mejorada"""
        MSB = wiringpi.digitalRead(self.pin_a)
        LSB = wiringpi.digitalRead(self.pin_b)
        
        encoded = (MSB << 1) | LSB
        sum_val = (self.last_encoded << 2) | encoded
        
        if sum_val == 0b1101 or sum_val == 0b0100 or sum_val == 0b0010 or sum_val == 0b1011:
            self.counter += 1 * self.invert
        elif sum_val == 0b1110 or sum_val == 0b0111 or sum_val == 0b0001 or sum_val == 0b1000:
            self.counter -= 1 * self.invert
        
        self.last_encoded = encoded

    def _isr_callback_a(self):
        """Interrupción para canal A"""
        self._update_encoder()

    def _isr_callback_b(self):
        """Interrupción para canal B"""
        self._update_encoder()

    def calculate_rpm(self):
        """Calcula RPM con filtro adaptativo"""
        with self.lock:
            now = time.perf_counter()
            dt = now - self.last_time
            
            # Solo calcular si ha pasado suficiente tiempo
            if dt < self.sample_time:
                return self.current_rpm_wheel
            
            # Obtener pulsos desde la última lectura
            current_counter = self.counter
            d_pulses = current_counter - self.last_counter
            
            # Calcular RPM: (pulsos/ppr) * (60/dt)
            if dt > 0:
                raw_rpm = (d_pulses / self.ppr_wheel) * (60.0 / dt)
            else:
                raw_rpm = 0.0
            
            # **CRÍTICO: Guardar el valor calculado**
            # Sin esta línea, siempre retorna 0
            self.current_rpm_wheel = raw_rpm
            
            # Filtro pasa-bajas ADAPTATIVO
            # Si la velocidad es alta, menos filtrado; si es baja, más filtrado
            if abs(raw_rpm) > 100:  # RPM alto
                alpha = 0.3  # Filtro ligero
            elif abs(raw_rpm) > 10:  # RPM medio
                alpha = 0.5  # Filtro medio
            else:  # RPM bajo
                alpha = 0.7  # Filtro fuerte
            
            # Aplicar filtro
            self.current_rpm_wheel = (alpha * raw_rpm) + ((1 - alpha) * self.current_rpm_wheel)
            
            # Guardar en historial para promedio móvil
            self.rpm_history.append(self.current_rpm_wheel)
            if len(self.rpm_history) > self.max_history:
                self.rpm_history.pop(0)
            
            # Usar promedio móvil si tenemos suficiente historial
            if len(self.rpm_history) >= self.max_history:
                self.current_rpm_wheel = sum(self.rpm_history) / len(self.rpm_history)
            
            # Actualizar para próxima lectura
            self.last_counter = current_counter
            self.last_time = now
            
            return self.current_rpm_wheel
    
    def calculate_rpm_high_speed(self):
        """Versión alternativa para alta velocidad (sin filtro)"""
        with self.lock:
            now = time.perf_counter()
            dt = now - self.last_time
            
            if dt < 0.001:  # 1ms mínimo
                return self.current_rpm_wheel
            
            current_counter = self.counter
            d_pulses = current_counter - self.last_counter
            
            # RPM sin filtro para máxima respuesta
            if dt > 0:
                self.current_rpm_wheel = (d_pulses / self.ppr_wheel) * (60.0 / dt)
            else:
                self.current_rpm_wheel = 0.0
            
            self.last_counter = current_counter
            self.last_time = now
            
            return self.current_rpm_wheel
    
    def get_pulses(self):
        """Retorna el contador total de pulsos"""
        with self.lock:
            return self.counter
    
    def get_rads(self):
        """Conversión RPM -> Rad/s"""
        return self.current_rpm_wheel * 0.10472
    
    def get_velocity(self, wheel_radius):
        """Calcula velocidad lineal de la rueda (m/s)"""
        rads = self.get_rads()
        return rads * wheel_radius
    
    def reset(self):
        """Resetea contadores"""
        with self.lock:
            self.counter = 0
            self.last_counter = 0
            self.current_rpm_wheel = 0.0
            self.rpm_history = []

