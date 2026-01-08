import json
import time
import math

from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera


class TauLidarLogger:

    def __init__(
        self,
        log_file="lidar_log.jsonl",
        z_min=0.1,
        z_max=2.0,
        amp_min=10,
        max_points=1500,
        debug=False
    ):
        self.log_file = log_file
        self.z_min = z_min
        self.z_max = z_max
        self.amp_min = amp_min
        self.max_points = max_points
        self.debug = debug

        self.camera = None
        self.enabled = False
        self.counter = 0

    # ==============================
    # SETUP
    # ==============================
    def setup(self, serial_port=None):
        ports = Camera.scan() if serial_port is None else [serial_port]
        if not ports:
            raise RuntimeError("No Tau LiDAR found")

        Camera.setRange(0, 4500)

        self.camera = Camera.open(ports[0])
        self.camera.setModulationChannel(0)
        self.camera.setIntegrationTime3d(0, 1000)
        self.camera.setMinimalAmplitude(0, self.amp_min)

        print("[TauLiDAR] Camera ready")

    # ==============================
    # CONTROL
    # ==============================
    def start(self):
        self.enabled = True
        print("[TauLiDAR] Logging ENABLED")

    def stop(self):
        self.enabled = False
        print("[TauLiDAR] Logging DISABLED")

    # ==============================
    # CAPTURA ATÓMICA
    # ==============================
    def capture_once(self, pose):
        if not self.enabled or self.camera is None:
            if self.debug:
                print("[TauLiDAR] capture_once skipped (disabled)")
            return

        frame = self.camera.readFrame(FrameType.DISTANCE)
        if frame is None:
            if self.debug:
                print("[TauLiDAR] Empty frame")
            return

        points = []

        for p in frame.points_3d:
            if len(p) >= 3:
                x, y, z = p[:3]
                if self.z_min < z < self.z_max:
                    points.append([x, y, z])

            if len(points) >= self.max_points:
                break

        log = {
            "t": time.time(),
            "pose": {
                "x": pose[1],
                "y": pose[0],
                "yaw": pose[2]
            },
            "points": points
        }

        with open(self.log_file, "a") as f:
            f.write(json.dumps(log) + "\n")

        self.counter += 1

        if self.debug:
            print(
                f"[TauLiDAR] Frame {self.counter} | "
                f"points: {len(points)} | "
                f"pose: {pose}"
            )

    # ==============================
    # CLEANUP
    # ==============================
    def close(self):
        if self.camera:
            self.camera.close()
            self.camera = None
            print("[TauLiDAR] Camera closed")
