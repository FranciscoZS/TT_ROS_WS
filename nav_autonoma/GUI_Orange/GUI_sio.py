import socketio
import threading
import time
from nav_autonoma.configRobot import Config

class SocketIOClient:
    def __init__(self, node, server_url: str):
        self.node = node
        self.server_url = server_url
        self.modules = []
        self._connected = False
        self._running = True
        
        # Iniciamos el cliente 
        self.sio = socketio.Client(
            reconnection=True, 
            reconnection_attempts=5, 
            reconnection_delay=2
        )
        
        # Registrar eventos de conexión
        self._setup_connection_events()
        
    def _setup_connection_events(self):
        #Configura los eventos básicos de conexión

        @self.sio.event
        def connect():
            self._connected = True
            self.node.get_logger().info("✅ Conectado al servidor Socket.IO")
            self.sio.emit("is-online", {"status": "online", "success": True})
        
        @self.sio.event
        def disconnect():
            self._connected = False
            self.node.get_logger().warn("⚠️ Desconectado del servidor")
        
        @self.sio.event
        def connect_error(data):
            self.node.get_logger().error(f"❌ Error de conexión: {data}")
        
        # Evento genérico para debug 
        @self.sio.event
        def message(data):
            self.node.get_logger().debug(f"📩 Mensaje recibido: {data}")
    
    def add_module(self, module):
        """
        Registra un módulo de eventos ANTES de conectar
        
        Args:
            module: Instancia de clase con método register(sio, node)
        """
        if self._connected:
            self.node.get_logger().warn(
                "⚠️ Añadiendo módulo después de conectar. "
                "Algunos eventos pueden perderse."
            )
        
        module.register(self.sio, self.node)
        self.modules.append(module)
        self.node.get_logger().info(f"📦 Módulo {module.__class__.__name__} registrado")
    
    def start(self):
        """Inicia la conexión en un thread separado"""
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.node.get_logger().warn("Thread ya está corriendo")
            return
        
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.node.get_logger().info("🔌 Intentando conectar a Socket.IO...")
    
    def _run(self):
        """Loop principal del thread de Socket.IO"""
        retry_count = 0
        max_retries = 5
        
        while self._running and retry_count < max_retries:
            try:
                if not self.sio.connected:
                    self.node.get_logger().info(
                        f"Intento de conexión {retry_count + 1}/{max_retries}..."
                    )
                    self.sio.connect(self.server_url)
                    self.sio.wait()  # Bloquea hasta desconexión
                    Config.ROBOT_IS_ONLINE = True
                    
            except socketio.exceptions.ConnectionError as e:
                retry_count += 1
                self.node.get_logger().error(
                    f"Error de conexión ({retry_count}/{max_retries}): {e}"
                )
                if retry_count < max_retries:
                    time.sleep(2 ** retry_count)  
                    
            except Exception as e:
                self.node.get_logger().error(f"Error inesperado en Socket.IO: {e}")
                retry_count += 1
                time.sleep(2)
        
        if retry_count >= max_retries:
            self.node.get_logger().error(
                "Máximo de reintentos alcanzado. Conexión Socket.IO fallida."
            )
    
    def emit(self, event, data):
        """
        Emite un evento al servidor
        
        Args:
            event: Nombre del evento
            data: Datos a enviar (debe ser serializable a JSON)
        """
        if self.sio.connected:
            try:
                self.sio.emit(event, data)
                self.node.get_logger().debug(f"📤 Emitido: {event}")
            except Exception as e:
                self.node.get_logger().error(f"Error al emitir {event}: {e}")
        else:
            self.node.get_logger().warn(
                f"⚠️ Socket no conectado. No se puede emitir '{event}'"
            )
    
    def disconnect(self):
        """Desconecta limpiamente el cliente"""
        self._running = False
        
        if self.sio.connected:
            try:
                self.sio.disconnect()
                self.node.get_logger().info("🔌 Socket.IO desconectado")
            except Exception as e:
                self.node.get_logger().error(f"Error al desconectar: {e}")
        
        # Esperar a que el thread termine (con timeout)
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        Config.ROBOT_IS_ONLINE = False
        
    
    @property
    def connected(self):
        """Propiedad para verificar el estado de conexión"""
        return self.sio.connected