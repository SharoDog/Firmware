import socket
import time

from picamera2 import Picamera2
import pickle
import struct


class Vision():
    def __init__(self):
        self.port = 9999
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn = None
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        self.socket.bind(('', self.port))

    def listen(self):
        # initialize camera in the new process
        # prevents access by multiple processes to the resource
        self.camera = Picamera2()

        camera_config = self.camera.create_still_configuration(
            main={"size": (320, 240)})
        self.camera.configure(camera_config)
        self.camera.start()
        try:
            self.socket.listen(1)
            while True:
                self.conn, _ = self.socket.accept()
                if self.conn:
                    while True:
                        image = self.camera.capture_array()
                        a = pickle.dumps(image)
                        message = struct.pack("Q", len(a)) + a
                        self.conn.sendall(message)
                time.sleep(0)
        except KeyboardInterrupt:
            print('Killing vision server...')
            if self.conn:
                self.conn.close()
            self.socket.close()
            self.camera.stop()
            return
