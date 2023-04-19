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

    def listen(self, controller_pipe):
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
                            print(data)
                            if not data:
                                self.conn.close()
                                break
                            controller_pipe.send(data)
                        # check if controller has updated command
                        try:
                            if (controller_pipe.readable
                                    and controller_pipe.poll()):
                                msg = controller_pipe.recv()
                                self.conn.send(msg.encode())
                        except Exception:
                            pass
                        time.sleep(0)
                    time.sleep(0)
        except KeyboardInterrupt:
            print('Killing server...')
            if self.conn:
                self.conn.close()
            self.socket.close()
            return
