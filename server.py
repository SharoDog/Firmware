import socket


class Server():
    def __init__(self):
        self.port = 5000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(('', self.port))
        self.socket.setblocking(False)

    def listen(self, msg_queue, quit_event):
        self.socket.listen(1)
        while True:
            if quit_event.is_set():
                print('Killing server...')
                self.socket.close()
                return
            try:
                conn, _ = self.socket.accept()
            except Exception:
                pass
            else:
                conn.setblocking(False)
                while True:
                    try:
                        data = conn.recv(1024).decode()
                    except Exception:
                        pass
                    else:
                        print(data)
                        if not data:
                            conn.close()
                            break
                        msg_queue.put(data)
                    if quit_event.is_set():
                        print('Killing server...')
                        conn.close()
                        self.socket.close()
                        return
