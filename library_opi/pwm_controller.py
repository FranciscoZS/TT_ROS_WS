"""
Scrip de python generado para manejar los pwm desde python
ocupando comandos bash. Para mas información checar el manual de usuario

Script echo especificamente para la orange pi 5 max
PWM disponibles (ejecutar comando ls /sys/class/pwm/ -l para ver los pwm disponibles):
PWM0_M0 (fd8b0000) Pin 5 pwmchip0
PWM1_M0 (fd8b0010) Pin 3 pwmchip1
PWM3_IR_M3 (fd8b0030) Pin 7 pwmchip2
PWM12_M0 (febf0000) Pin 31 pwmchip4
PWM13_M0 (febf0010) Pin 33 pwmchip5
PWM14_M0 (febf0020) Pin 35 pwmchip6
"""

import subprocess

class PWMOrangepi:
    def __init__(self, chip_number,pwm_number=0, pin_dir=22, dir = 0, node=None):
        
        """
        PWM0_M0 (fd8b0000) Pin 5 pwmchip0
        PWM1_M0 (fd8b0010) Pin 3 pwmchip1
        PWM3_IR_M3 (fd8b0030) Pin 7 pwmchip2
        PWM12_M0 (febf0000) Pin 31 pwmchip4
        PWM13_M0 (febf0010) Pin 33 pwmchip5
        PWM14_M0 (febf0020) Pin 35 pwmchip6
        """
        #Funcion para generar el path 
        self.chip_number = chip_number #pwchip#
        self.pwm_number = pwm_number #pwm#
        self.base_path = f"/sys/class/pwm/pwmchip{chip_number}"
        self.pwm_path = f"{self.base_path}/pwm{pwm_number}"
        self.pin_dir = pin_dir #pin wiringpi
        self.dir = dir # 0 ó 1
        self.is_exported = False
        self.period = None
        self.duty_cycle = None
        self.node = node
        self._duty_cache = None  # Cache para evitar escrituras redundantes

    def setup(self):
        """
        Fución para cambiar los permisos de  los archivos
        y exportaar los demas archivos 
        """
        # Cambiar propieddevad del archivo pwmchip
        subprocess.run(f"sudo chown -R orangepi {self.base_path}/",shell=True)
        # Exportar pwm
        subprocess.run(f"echo 0 > {self.base_path}/export",shell=True)
        # Cambiar propiedad del archivo pwm
        subprocess.run(f"sudo chown -R orangepi {self.pwm_path}/", shell=True)
        #Confiogurar pin de direccion
        subprocess.run(f"gpio mode {self.pin_dir} out",shell=True)
        subprocess.run(f"gpio write {self.pin_dir} {self.dir}",shell=True)
        self.is_exported = True
        print(f"PWM {self.pwm_number} en chip {self.chip_number} exportado correctamente")

    def configure(self, period, duty_cycle, enable=True):
        #Configurar periodo, cilco de trabajo y estado incial

        if not self.is_exported:
            print("PWM no exportado. Ejecuta setup() primero.")
            return False

        self.period = period
        # Configurar período
        subprocess.run(f"echo {period} > {self.pwm_path}/period", shell=True)
            
        # Configurar ciclo de trabajo
        subprocess.run(f"echo {duty_cycle} > {self.pwm_path}/duty_cycle", shell=True)
            
        # Configurar estado (enable/disable)
        enable_value = 1 if enable else 0
        subprocess.run(f"echo {enable_value} > {self.pwm_path}/enable", shell=True)
        
        print(f"PWM configurado - Periodo: {period}, Duty Cycle: {duty_cycle}, Enable: {enable}")

    def change_dir(self,dir):
        subprocess.run(f"gpio write {self.pin_dir} {dir}",shell=True)

    def set_duty_cycle(self,duty_cycle):
        #modificar ciclo de trabajo

        if not self.is_exported:
            print("PWM no exportado")
            return False

        subprocess.run(f"echo {duty_cycle} > {self.pwm_path}/duty_cycle",shell=True)
        print(f"Duty cycle actualizado: {duty_cycle}")

    def set_enable(self):
        #activar el pwm
        if not self.is_exported:
            print("PWM no exportado")
            return False
        subprocess.run(f"echo 1 > {self.pwm_path}/enable", shell=True)
            
        print(f"PWM activado")
    
    def set_unenable(self):
        #apagar el pwm
        if not self.is_exported:
            print("PWM no exportado")
            return False
        
        subprocess.run(f"echo {self.period} > {self.pwm_path}/duty_cycle",shell=True)
        
        print(f"PWM desactivado")

    def configure_frecuency(self, frecuecy):
        
        """
        Por defecto en la data sheet nos dice que para una frecuencia de 50 hz
        tenermos que poner el periodo a 20000000, ya que se maneja en nano segundos tenemos que
        f=1/T -> 1/(20000000E-9)=50hz ó 1/0.02 = 50Hz
        Para el periodo entonces se tendra que multiplicar el resultado por 1E9 para pasarlo a nano segundos
        eje:
        T=1/50Hz=0.02
        0.02*1E9=20000000 como se ve en el ejemplo
        """

        if not self.is_exported:
            print("PWM no exportado. Ejecuta setup() primero.")
            return False
        
        if frecuecy > 1000000:
            print("La frecuencia no puede ser mayor a 100MHz")
            return False
        
        self.period =  int((1/frecuecy)*(1E9))

        # Configurar período con base en la frecuencia
        subprocess.run(f"echo {self.period} > {self.pwm_path}/period", shell=True)

        print(f"PWM configurado - Frecuencia: {frecuecy} Periodo: {self.period}")
        
    def configure_duty_cycle(self, duty_cycle):
        """
        Para el ciclo de trabajo la orange trabaja de manera inversa
        cuando el ciclo de trabajo el igual al periodo en magnitud, 
        el pwm manda una señal todo en bajo, cuando el ciclo de trabajo decrece
        el ancho de pulso aumenta
        """


        if not self.is_exported:
            print("PWM no exportado. Ejecuta setup() primero.")
            return False
        
        if duty_cycle > 100:
            print("La el ciclo de no puede ser mayor a 100%")
            return False
        
        print(f"El ciclo de trabajo selecionado es: {duty_cycle}%")
        if duty_cycle == 100:
            self.duty_cycle = int(self.period*0.01) #equivale al 1% por si las dudas
        elif duty_cycle == 0:
            self.duty_cycle = self.period #para apagar el pwm o que regrese 0 period == duty cycle
        else:
            self.duty_cycle = self.period - int(self.period*(duty_cycle/100))
            print(self.duty_cycle)
            print(self.period)

        subprocess.run(f"echo {self.duty_cycle} > {self.pwm_path}/duty_cycle", shell=True)

        print(f"el valor del registro duty_cycle es: {self.duty_cycle}")
    

    def configurePWM(self, frecuencia, duty_cycle, enable=True):
        #Configurar frecuencia, cilco de trabajo y estado incial

        if not self.is_exported:
            print("PWM no exportado. Ejecuta setup() primero.")
            return False

        self.configure_frecuency(frecuencia) #configuramos la frecuencia
        self.configure_duty_cycle(duty_cycle) #configurar el ciclo de trabajo %
        # Configurar estado (enable/disable)
        enable_value = 1 if enable else 0
        subprocess.run(f"echo {enable_value} > {self.pwm_path}/enable", shell=True)
        
        print(f"PWM configurado")

    def set_duty_cycle_optimized(self, duty_cycle):
        """Versión optimizada que evita escrituras innecesarias"""
        if duty_cycle == self._duty_cache:
            return  # No escribir si no cambió
            
        if not self.is_exported:
            if self.node:
                self.node.get_logger().warn('PWM no exportado')
            return False
            
        # Calcular duty cycle como antes
        if duty_cycle == 100:
            actual_duty = int(self.period * 0.01)
        elif duty_cycle == 0:
            actual_duty = self.period
        else:
            actual_duty = self.period - int(self.period * (duty_cycle / 100))
            
        try:
            subprocess.run(
                f"echo {actual_duty} > {self.pwm_path}/duty_cycle", 
                shell=True, 
                check=True,
                timeout=0.1  # Timeout para evitar bloqueos
            )
            self._duty_cache = duty_cycle
            return True
        except subprocess.TimeoutExpired:
            if self.node:
                self.node.get_logger().error('Timeout en comando PWM')
            return False

    def cleanup(self):
        #liberar el modo pwm
        self.set_unenable()
        subprocess.run(f"echo 0 > {self.pwm_path}/enable", shell=True)
        subprocess.run(f"echo 0 > {self.base_path}/unexport", shell=True)
    
