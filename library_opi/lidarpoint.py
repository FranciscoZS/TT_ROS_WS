import json
import time
import threading

from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class TauLidarLogger:
    """
    Librería para captura de nube de puntos Tau LiDAR
    y guardado en formato JSONL controlado externamente.
    """

    # INIT

    def __init__(
        self,
        log_file="lidar_log.jsonl",
        capture_hz=2.0,
        z_min=0.1,
        z_max=2.0,
        amp_min=10,
        max_points=1500,
        serial_port=None
    ):
        super().__init__('TauLidarLogger')
        self.log_file_path = log_file
        self.dt = 1.0 / capture_hz

        self.z_min = z_min
        self.z_max = z_max
        self.amp_min = amp_min
        self.max_points = max_points
        self.serial_port = serial_port

        self.camera = None
        self.log_file = None

        self.running = False
        self.thread = None

        # Pose del robot (actualizable desde ROS)
        self.pose = (0.0, 0.0, 0.0)
        
                # ========== SUSCRIPTORES ==========
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    # SETUP CÁMARA
    def _setup_camera(self):
        ports = Camera.scan() if self.serial_port is None else [self.serial_port]
        if not ports:
            raise RuntimeError("No se encontró Tau LiDAR")

        Camera.setRange(0, 4500)

        cam = Camera.open(ports[0])
        cam.setModulationChannel(0)
        cam.setIntegrationTime3d(0, 1000)
        cam.setMinimalAmplitude(0, self.amp_min)

        print("[TauLiDAR] Cámara inicializada")
        return cam

    def start(self):
        """Inicia la captura de la nube de puntos"""
        if self.running:
            print("[TauLiDAR] Captura ya en ejecución")
            return

        self.camera = self._setup_camera()
        self.log_file = open(self.log_file_path, "a")

        self.running = True
        self.thread = threading.Thread(
            target=self._capture_loop,
            daemon=True
        )
        self.thread.start()

        print("[TauLiDAR] Captura iniciada")

    def stop(self):
        """Detiene la captura de forma segura"""
        if not self.running:
            return

        self.running = False

        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None

        if self.camera:
            self.camera.close()
            self.camera = None

        if self.log_file:
            self.log_file.close()
            self.log_file = None

        print("[TauLiDAR] Captura detenida")

    def set_pose(self, x, y, yaw):
        """Actualiza la pose del robot (desde ROS u otra fuente)"""
        with self.lock:
            self.pose = (x, y, yaw)

    def _capture_loop(self):
        while self.running:
            t0 = time.time()

            try:
                frame = self.camera.readFrame(FrameType.DISTANCE)
                if frame is None:
                    continue
            except Exception as e:
                print(f"[TauLiDAR] Error leyendo frame: {e}")
                continue

            points = []

            for p in frame.points_3d:
                if len(p) >= 3:
                    x, y, z = p[:3]

                    if self.z_min < z < self.z_max:
                        points.append([x, y, z])

                if len(points) >= self.max_points:
                    break

            with self.lock:
                xr, yr, yaw = self.pose

            log = {
                "t": time.time(),
                "pose": {
                    "x": xr,
                    "y": yr,
                    "yaw": yaw
                },
                "points": points
            }

            self.log_file.write(json.dumps(log) + "\n")
            self.log_file.flush()

            # Control de frecuencia
            dt_sleep = self.dt - (time.time() - t0)
            if dt_sleep > 0:
                time.sleep(dt_sleep)
