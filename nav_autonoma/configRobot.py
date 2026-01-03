
"""
Este archivo es para centralizar las connfiguraciones de TODO el robot
pARA PODER TENER UN CONTROL COMPLETO CON ROS2
"""

from library_opi.audio_player import AudioPlayer

class Config:

#Pagina Web
    #Ojo cambiar despues "udp://192.168.0.200:1235"
    Emisor_UDP_ADDR = "udp://192.168.0.200:1235"
    

    ROBOT_START_PROCESS = False
    ROBOT_STop_PROCESS = False
    ROBOT_UVCLAMPS_ACTIVATE = False
    ROBOT_IS_ONLINE =  False
    ROBOT_CREATE_MAP = False


    # ============================================
    # CONFIGURACIÓN DE AUDIO
    # ============================================
    AUDIO_FILE = '/home/orangepi/sound_ia/resource/desalojo.mp3'
    AUDIO_DEVICE = 'plughw:3,0'
    AUDIO_TRIGGER_COOLDOWN = 5.0  # Segundos entre triggers
    
    # ============================================
    # TOPICS
    # ============================================
    TOPIC_AUDIO_TRIGGER = '/audio/trigger'
    TOPIC_AUDIO_STATUS = '/audio/status'