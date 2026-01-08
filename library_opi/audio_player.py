#!/usr/bin/env python3
"""
Librería para reproducción de audio con subprocess
Compatible con el sistema de navegación autónoma
"""
import subprocess
import os
import threading
import time

class AudioPlayer:
    def __init__(self, audio_device='plughw:3,0', node=None):
        """
        Inicializa el reproductor de audio
        
        Args:
            audio_device (str): Dispositivo ALSA (ej: 'plughw:3,0')
            node: Referencia al nodo ROS2 para logging
        """
        self.audio_device = audio_device
        self.node = node
        
        # Variables de control
        self.current_process = None
        self.is_playing = False
        self.lock = threading.Lock()
        
        # Configuración de reproducción
        self.blocking = False  # Por defecto no bloquea
        
        if self.node:
            self.node.get_logger().info(f'🔊 AudioPlayer inicializado - Device: {audio_device}')
    
    def play(self, audio_file, blocking=False, loop=False):
        """
        Reproduce un archivo de audio
        
        Args:
            audio_file (str): Ruta al archivo de audio
            blocking (bool): Si True, espera a que termine la reproducción
            loop (bool): Si True, reproduce en loop continuo
        
        Returns:
            bool: True si comenzó la reproducción, False si hubo error
        """
        # Verificar que el archivo existe
        if not os.path.exists(audio_file):
            if self.node:
                self.node.get_logger().error(f'❌ Archivo no encontrado: {audio_file}')
            return False
        
        # Verificar extensión del archivo
        ext = os.path.splitext(audio_file)[1].lower()
        
        # Detener reproducción actual si hay una
        if self.is_playing:
            self.stop()
        
        try:
            with self.lock:
                # Construir comando según el tipo de archivo
                if ext in ['.mp3', '.mp2']:
                    cmd = f"mpg321 -o alsa -a {self.audio_device} '{audio_file}'"
                elif ext in ['.wav']:
                    cmd = f"aplay -D {self.audio_device} '{audio_file}'"
                elif ext in ['.ogg', '.oga']:
                    cmd = f"ogg123 -d alsa -o dev:{self.audio_device} '{audio_file}'"
                else:
                    # Intentar con mpg321 por defecto
                    cmd = f"mpg321 -o alsa -a {self.audio_device} '{audio_file}'"
                
                if self.node:
                    self.node.get_logger().info(f'🎵 Reproduciendo: {os.path.basename(audio_file)}')
                
                if loop:
                    # Reproducción en loop en thread separado
                    self.is_playing = True
                    loop_thread = threading.Thread(
                        target=self._play_loop,
                        args=(cmd, audio_file),
                        daemon=True
                    )
                    loop_thread.start()
                    return True
                elif blocking:
                    # Reproducción bloqueante
                    self.is_playing = True
                    result = subprocess.run(
                        cmd,
                        shell=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    self.is_playing = False
                    return result.returncode == 0
                else:
                    # Reproducción no bloqueante
                    self.current_process = subprocess.Popen(
                        cmd,
                        shell=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    self.is_playing = True
                    
                    # Thread para monitorear cuando termine
                    monitor_thread = threading.Thread(
                        target=self._monitor_process,
                        daemon=True
                    )
                    monitor_thread.start()
                    
                    return True
                    
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error reproduciendo audio: {e}')
            self.is_playing = False
            return False
    
    def _play_loop(self, cmd, audio_file):
        """Reproduce audio en loop continuo"""
        while self.is_playing:
            try:
                result = subprocess.run(
                    cmd,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                
                if result.returncode != 0:
                    if self.node:
                        self.node.get_logger().error(f'❌ Error en reproducción loop')
                    break
                    
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f'❌ Error en loop: {e}')
                break
    
    def _monitor_process(self):
        """Monitorea el proceso de audio hasta que termine"""
        if self.current_process:
            self.current_process.wait()
            with self.lock:
                self.is_playing = False
                if self.node:
                    self.node.get_logger().debug('🔇 Reproducción finalizada')
    
    def stop(self):
        """Detiene la reproducción actual"""
        with self.lock:
            if self.current_process:
                try:
                    self.current_process.terminate()
                    self.current_process.wait(timeout=2)
                    if self.node:
                        self.node.get_logger().info('⏹️  Reproducción detenida')
                except:
                    self.current_process.kill()
                finally:
                    self.current_process = None
            
            self.is_playing = False
    
    def is_playing_audio(self):
        """
        Verifica si hay audio reproduciéndose
        
        Returns:
            bool: True si hay audio reproduciéndose
        """
        return self.is_playing
    
    def play_once_if_not_playing(self, audio_file):
        """
        Reproduce audio solo si no hay nada reproduciéndose
        Útil para evitar superposición de sonidos
        
        Args:
            audio_file (str): Ruta al archivo
        
        Returns:
            bool: True si comenzó la reproducción
        """
        if not self.is_playing:
            return self.play(audio_file, blocking=False, loop=False)
        return False
    
    def set_volume(self, volume):
        """
        Ajusta el volumen del sistema (0-100)
        
        Args:
            volume (int): Nivel de volumen (0-100)
        """
        try:
            volume = max(0, min(100, volume))
            cmd = f"amixer -D {self.audio_device} set Master {volume}%"
            subprocess.run(cmd, shell=True, check=True)
            
            if self.node:
                self.node.get_logger().info(f'🔊 Volumen ajustado a {volume}%')
            
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Error ajustando volumen: {e}')
            return False
    
    def cleanup(self):
        """Limpia recursos"""
        self.stop()
        if self.node:
            self.node.get_logger().info('🛑 AudioPlayer cerrado')
