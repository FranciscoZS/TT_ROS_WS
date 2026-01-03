import wiringpi
import time
import threading

class SimpleRGBPCA9685:
    def __init__(self, bus=8, node=None):
        self.fd = wiringpi.wiringPiI2CSetupInterface(f"/dev/i2c-{bus}", 0x40)
        if self.fd < 0: 
            if node:
                node.get_logger().error("Error I2C - No se pudo inicializar PCA9685")
            raise IOError("Error I2C")

        self.node = node
        self.lock = threading.Lock()

        # Configurar PCA9685 a 1000Hz para LEDs
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x00, 0x10)
        time.sleep(0.005)
        prescale = int(25000000 / (4096.0 * 1000) -1 + 0.5)
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0xFE, prescale)
        time.sleep(0.005)
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x00, 0x00)
        time.sleep(0.005)

        if node:
            node.get_logger().info('PCA9685 inicializado correctamente')

    def set_color(self, r_chan=0, g_chan=1, b_chan=2, r=0, g=0, b=0):
        """Establece color RGB (valores 0-100) de manera thread-safe"""
        with self.lock:
            def set_chan(chan, val):
                pwm = int(val * 40.95)  # Convertir 0-100 a 0-4095
                base = 0x06 + 4 * chan
                wiringpi.wiringPiI2CWriteReg8(self.fd, base + 2, pwm & 0xFF)
                wiringpi.wiringPiI2CWriteReg8(self.fd, base + 3, pwm >> 8)

            set_chan(r_chan, r)
            set_chan(g_chan, g)
            set_chan(b_chan, b)

    def clean_color(self, r_chan, g_chan, b_chan):
        """Apaga los LEDs (establece color a 0)"""
        self.set_color(r_chan, g_chan, b_chan, 0, 0, 0)

    def set_single_channel(self, channel, value):
        """Establece un canal individual (0-100)"""
        with self.lock:
            pwm = int(value * 40.95)
            base = 0x06 + 4 * channel
            wiringpi.wiringPiI2CWriteReg8(self.fd, base + 2, pwm & 0xFF)
            wiringpi.wiringPiI2CWriteReg8(self.fd, base + 3, pwm >> 8)