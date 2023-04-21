import socket
import time
from multiprocessing.connection import Connection

try:
    from picamera2 import Picamera2
except ImportError:
    pass
try:
    import cv2
except ImportError:
    pass
import pickle


class Vision():
    def __init__(self, mock):
        self.mock = mock
        self.addr = None
        self.port = 9999
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65527)

    def run(self, server_pipe: Connection):
        # initialize camera in the new process
        # prevents access by multiple processes to the resource
        if self.mock:
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            self.camera = Picamera2()

            camera_config = self.camera.create_still_configuration(
                main={"size": (640, 480)})
            self.camera.configure(camera_config)
            self.camera.start()
        try:
            while True:
                if server_pipe.readable and server_pipe.poll():
                    msg = server_pipe.recv()
                    if msg == 'dc':
                        self.addr = None
                    else:
                        self.addr = msg
                if self.addr:
                    if self.mock:
                        _, frame = self.camera.read()
                        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    else:
                        image = self.camera.capture_array()
                    ret, buffer = cv2.imencode(
                        '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
                    message = pickle.dumps(buffer)
                    self.socket.sendto(message, (self.addr, self.port))
                time.sleep(0)
        except KeyboardInterrupt:
            print('Killing vision server...')
            self.socket.close()
            if not self.mock:
                self.camera.stop()
            else:
                self.camera.release()
            return
