from server import Server
from controller import Controller
from sensors import Sensors
from vision import Vision
import multiprocessing
multiprocessing.freeze_support()

if __name__ == '__main__':
    try:
        attitude = multiprocessing.Array('d', [0.0, 0.0, 0.0])
        server = Server()
        server_conn, controller_conn = multiprocessing.Pipe(duplex=True)
        comms = multiprocessing.Process(
            target=server.listen, args=(server_conn,))
        controller = Controller()
        control = multiprocessing.Process(target=controller.run, args=(
            controller_conn, attitude,))
        sensors_reader = Sensors()
        sensors = multiprocessing.Process(
            target=sensors_reader.run, args=(attitude,))
        comms.start()
        sensors.start()
        control.start()
        vision_server = Vision()
        vision = multiprocessing.process(target=vision_server.listen)
        vision.start()
        print('Started...')
        comms.join()
        control.join()
        sensors.join()
        vision.join()
    except KeyboardInterrupt:
        print('Quitting...')
