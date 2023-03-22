from server import Server
from controller import Controller
from queue import Queue
from threading import Thread, Event

if __name__ == '__main__':
    server = Server()
    msg_queue = Queue()
    quit_event = Event()
    comms = Thread(target=server.listen, args=(msg_queue, quit_event))
    comms.start()
    controller = Controller()
    control = Thread(target=controller.run, args=(
        msg_queue, quit_event))
    control.start()
    print('Started...')
    try:
        comms.join()
        control.join()
    except KeyboardInterrupt:
        quit_event.set()
        print('Quitting...')
