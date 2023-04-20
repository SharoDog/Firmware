import time
import socket


class Server():
    def __init__(self):
        self.port = 5000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.port))
        self.socket.setblocking(False)
        self.conn = None

    def listen(self, controller_pipe, sensors_pipe):
        try:
            self.socket.listen(1)
            while True:
                try:
                    self.conn, _ = self.socket.accept()
                except Exception:
                    pass
                else:
                    self.conn.setblocking(False)
                    while True:
                        try:
                            data = self.conn.recv(1024).decode()
                        except Exception:
                            pass
                        else:
                            if not data:
                                self.conn.close()
                                break
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
                        time.sleep(0)
                # prevent pipe hang
                while (controller_pipe.readable
                        and controller_pipe.poll()):
                    msg = controller_pipe.recv()
                    print(msg)
                while (sensors_pipe.readable
                       and sensors_pipe.poll()):
                    msg = sensors_pipe.recv()
                time.sleep(0)
        except KeyboardInterrupt:
            print('Killing server...')
            if self.conn:
                self.conn.close()
            self.socket.close()
            return
