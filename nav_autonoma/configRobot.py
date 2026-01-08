
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
    
    #=============================================
    #Audios
    #=============================================
    sound_on_lamps = "/home/orangepi/sound_ia/resource/sounds/uvc_on.mp3"
    sound_off_lamps = "/home/orangepi/sound_ia/resource/sounds/uvc_off.mp3"
    sound_online_robot = "/home/orangepi/sound_ia/resource/sounds/online_robot.mp3"
    sound_emergency_stop = "/home/orangepi/sound_ia/resource/sounds/emergency_stop.mp3"
    sound_ctrl_manual = "/home/orangepi/sound_ia/resource/sounds/manual_ctrl.mp3"
    sound_mnt_gui = "/home/orangepi/sound_ia/resource/sounds/monitoreo_ctrl.mp3"
    sound_disconnect= "/home/orangepi/sound_ia/resource/sounds/disconnect.mp3"
    sound_bellako = "/home/orangepi/sound_ia/resource/sounds/Chichabeba.mp3"
    sound_go_robot = "/home/orangepi/sound_ia/resource/sounds/go_robot.mp3"

    # ============================================
    # TOPICS
    # ============================================
    TOPIC_AUDIO_TRIGGER = '/audio/trigger'
    TOPIC_AUDIO_STATUS = '/audio/status'