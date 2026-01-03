import smbus2
import time

class SCD40_SMBus:
    def __init__(self, bus_number=8, address=0x62, node=None):
        self.address = address
        self.bus = smbus2.SMBus(bus_number)
        self.node = node
        self.co2 = None
        self.temperature = None
        self.humidity = None
        self.measurement_started = False
        
        if node:
            node.get_logger().info(f"SCD40 inicializado en bus I2C-{bus_number}, dirección 0x{address:02x}")

    def calculate_crc8(self, data):
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc
    
    def send_command(self, command):
        """Envía comando sin lock"""
        try:
            high_byte = (command >> 8) & 0xFF
            low_byte = command & 0xFF
            crc = self.calculate_crc8([high_byte, low_byte])
            
            self.bus.write_i2c_block_data(self.address, high_byte, [low_byte, crc])
            time.sleep(0.01)
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error enviando comando: {e}")
            return False
    
    def read_measurement(self):
        """Lee medición SIN lock"""
        try:
            # Verificar si las mediciones han empezado
            if not self.measurement_started:
                if self.node:
                    self.node.get_logger().error("Mediciones no iniciadas. Llama a start_periodic_measurement() primero")
                return False
            
            # Enviar comando de lectura
            if not self.send_command(0xEC05):
                return False
                
            time.sleep(0.01)
            
            # Leer datos
            write_msg = smbus2.i2c_msg.write(self.address, [0xEC, 0x05, 0xB6])
            read_msg = smbus2.i2c_msg.read(self.address, 9)
            
            self.bus.i2c_rdwr(write_msg, read_msg)
            
            data = list(read_msg)
            
            if len(data) != 9:
                if self.node:
                    self.node.get_logger().error(f"Datos incompletos: {len(data)}/9 bytes")
                return False
            
            # Extraer valores
            self.co2 = (data[0] << 8) | data[1]
            temp_raw = (data[3] << 8) | data[4]
            hum_raw = (data[6] << 8) | data[7]
            
            self.temperature = -45 + 175 * (temp_raw / 65536.0)
            self.humidity = 100 * (hum_raw / 65536.0)

            if self.node:
                self.node.get_logger().info(
                    f"SCD40 - CO₂: {self.co2} ppm, "
                    f"Temp: {self.temperature:.2f}°C, "
                    f"Hum: {self.humidity:.2f}%"
                )
            
            return True
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error en read_measurement: {e}")
            return False

    def get_measurement(self):
        return self.co2, self.temperature, self.humidity

    def start_periodic_measurement(self):
        """Inicia mediciones con espera incorporada"""
        if self.node:
            self.node.get_logger().info("Iniciando mediciones periódicas SCD40...")
        
        success = self.send_command(0x21B1)
        
        if success:
            self.measurement_started = True
            if self.node:
                self.node.get_logger().info("✅ Mediciones iniciadas - Esperando 30 segundos para primera lectura...")
            
            # Espera inicial INCORPORADA
            for i in range(30, 0, -1):
                if self.node and i % 10 == 0:
                    self.node.get_logger().info(f"Tiempo restante: {i} segundos")
                time.sleep(1)
            
            if self.node:
                self.node.get_logger().info("✅ Sensor listo para lecturas")
        
        return success
    
    def stop_periodic_measurement(self):
        if self.node:
            self.node.get_logger().info("Deteniendo mediciones periódicas SCD40...")
        self.measurement_started = False
        return self.send_command(0x3F86)
    
    def close(self):
        self.bus.close()
