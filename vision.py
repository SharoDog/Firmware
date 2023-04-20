import socket
import time

try:
    from picamera2 import Picamera2
except ImportError:
    pass
try:
    import cv2
except ImportError:
    pass
import pickle
import struct


class Vision():
    def __init__(self, mock):
        self.mock = mock
        self.port = 9999
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn = None
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        self.socket.bind(('', self.port))

    def listen(self):
        # initialize camera in the new process
        # prevents access by multiple processes to the resource
        if self.mock:
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        else:
            self.camera = Picamera2()

            camera_config = self.camera.create_still_configuration(
                main={"size": (320, 240)})
            self.camera.configure(camera_config)
            self.camera.start()
        try:
            self.socket.listen(1)
            while True:
                try:
                    self.conn, _ = self.socket.accept()
                except Exception:
                    pass
                else:
                    while True:
                        if self.mock:
                            _, image = self.camera.read()
                        else:
                            image = self.camera.capture_array()
                        a = pickle.dumps(image)
                        message = struct.pack("Q", len(a)) + a
                        try:
                            self.conn.sendall(message)
                        except Exception:
                            self.conn.close()
                            break
                        time.sleep(0)
                time.sleep(0)
        except KeyboardInterrupt:
            print('Killing vision server...')
            if self.conn:
                self.conn.close()
            self.socket.close()
            if not self.mock:
                self.camera.stop()
            else:
                self.camera.release()
            return
