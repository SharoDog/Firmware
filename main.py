from server import Server
from controller import Controller
from vision import Vision
import multiprocessing
multiprocessing.freeze_support()

if __name__ == '__main__':
    try:
        server = Server()
        server_conn, controller_conn = multiprocessing.Pipe(duplex=True)
        comms = multiprocessing.Process(
            target=server.listen, args=(server_conn,))
        comms.start()
        controller = Controller()
        control = multiprocessing.Process(target=controller.run, args=(
            controller_conn,))
        control.start()
        vision_server = Vision()
        vision = multiprocessing.Process(target=vision_server.listen)
        vision.start()
        print('Started...')
        comms.join()
        control.join()
        vision.join()
    except KeyboardInterrupt:
        print('Quitting...')
