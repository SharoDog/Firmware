import time
import socket
from multiprocessing.connection import Connection


class Server():
    def __init__(self):
        self.port = 5000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.port))
        self.socket.setblocking(False)
        self.conn = None

    def listen(self, controller_pipe: Connection, sensors_pipe: Connection, speech_pipe: Connection,
               vision_pipe: Connection):
        try:
            self.socket.listen(1)
            while True:
                try:
                    self.conn, (addr, _) = self.socket.accept()
                except Exception:
                    pass
                else:
                    vision_pipe.send(addr)
                    self.conn.setblocking(False)
                    while True:
                        try:
                            data = self.conn.recv(1024).decode()
                        except Exception:
                            pass
                        else:
                            if not data:
                                self.conn.close()
                                vision_pipe.send('dc')
                                break
                            if data.startswith('sensors'):
                                sensors_pipe.send(
                                    data.split(':')[1].strip() == 'True')
                            elif data.startswith('speech'):
                                speech_pipe.send(
                                    data.split(':')[1].strip() == 'True')
                            else:
                                controller_pipe.send(data)
                        try:
                            # check if controller has updated command
                            if (controller_pipe.readable
                                    and controller_pipe.poll()):
                                msg = controller_pipe.recv()
                                print(msg)
                                self.conn.sendall((msg + '\r\n').encode())
                            # check if there is new sensors data
                            while (sensors_pipe.readable
                                   and sensors_pipe.poll()):
                                msg = sensors_pipe.recv()
                                self.conn.sendall((msg + '\r\n').encode())
                        except Exception:
                            pass
                        time.sleep(0.02)
                # prevent pipe hang
                while (controller_pipe.readable
                        and controller_pipe.poll()):
                    msg = controller_pipe.recv()
                    print(msg)
                while (sensors_pipe.readable
                       and sensors_pipe.poll()):
                    msg = sensors_pipe.recv()
                time.sleep(0.02)
        except KeyboardInterrupt:
            print('Killing server...')
            if self.conn:
                self.conn.close()
            self.socket.close()
            return
