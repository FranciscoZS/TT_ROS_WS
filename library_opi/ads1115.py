import smbus2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ADS1115:
    def __init__(self, address=0x48, bus=8, gain=0x00, node=None):
        """
        Versión ROS simplificada del ADS1115 - Solo canales 0 y 1
        """
        self.address = address
        self.bus_num = bus
        self.gain = gain
        self.node = node
        self.initialized = False
        
        # Multiplicadores de voltaje
        self.voltage_multipliers = {
            0x00: 6.144,  # ±6.144V (para 5V)
            0x02: 4.096,  # ±4.096V
            0x04: 2.048,  # ±2.048V
        }
        self.full_scale = self.voltage_multipliers.get(gain, 6.144)
        
        try:
            self.bus = smbus2.SMBus(bus)
            self.initialized = True
            
            if self.node:
                self.node.get_logger().info(f'ADS1115 inicializado - Bus: {bus}, Addr: 0x{address:02x}')
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'Error inicializando ADS1115: {e}')
            self.initialized = False
    
    def is_initialized(self):
        return self.initialized
    
    def read_channel(self, channel):
        """
        Lee un canal específico
        channel: 0x4000 (CANAL_0) o 0x5000 (CANAL_1)
        """
        if not self.initialized:
            if self.node:
                self.node.get_logger().error("ADS1115 no está inicializado")
            return 0, 0.0
            
        try:
            # Configuración single-shot
            # OS=1 (iniciar), MUX=channel, PGA=gain, MODE=1 (single-shot), DR=128SPS
            config = (0x8000 | channel | (self.gain << 9) | 0x0100 | 0x0080 | 0x0003)
            
            # Escribir configuración
            msb = (config >> 8) & 0xFF
            lsb = config & 0xFF
            self.bus.write_i2c_block_data(self.address, 0x01, [msb, lsb])
            
            # Esperar conversión (para 128SPS ~8ms)
            time.sleep(0.01)
            
            # Leer resultado
            data = self.bus.read_i2c_block_data(self.address, 0x00, 2)
            raw_value = (data[0] << 8) | data[1]
            
            # Convertir a signed (complemento a 2)
            if raw_value >= 0x8000:
                raw_value -= 0x10000
            
            # Calcular voltaje
            voltage = (raw_value / 32767.0) * self.full_scale
            
            return raw_value, voltage
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'Error leyendo canal: {e}')
            return 0, 0.0
    
    def read_channels_0_1(self):
        """
        Lee ambos canales 0 y 1
        Returns: dict con 'channel_0' y 'channel_1'
        """
        if not self.initialized:
            return {}
            
        results = {}
        
        # Leer canal 0
        raw_0, voltage_0 = self.read_channel(0x4000)  # CHANNEL_0
        results['channel_0'] = {
            'raw': raw_0,
            'voltage': voltage_0
        }
        
        time.sleep(0.005)  # Pequeña pausa
        
        # Leer canal 1
        raw_1, voltage_1 = self.read_channel(0x5000)  # CHANNEL_1
        results['channel_1'] = {
            'raw': raw_1,
            'voltage': voltage_1
        }
        
        return results, voltage_0, voltage_1
    
    def close(self):
        if hasattr(self, 'bus'):
            self.bus.close()