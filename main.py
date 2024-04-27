from display import Display
from server import Server
from controller import Controller
from sensors import Sensors
from vision import Vision
from speech import Speech
import multiprocessing
import argparse
import os
multiprocessing.freeze_support()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Firmware for Sharo')
    parser.add_argument(
        '-m', '--mock', action='store_true', help='Mock hardware.')
    parser.add_argument(
        '-pa', '--print-angles', action='store_true',
        help='Print servo angles when setting them.')
    args = parser.parse_args()
    server = Server()
    server_to_controller_pipe, controller_to_server_pipe = multiprocessing.Pipe(
        duplex=True)
    speech_to_controller_pipe, controller_to_speech_pipe = multiprocessing.Pipe()
    server_to_sensors_pipe, sensors_to_server_pipe = multiprocessing.Pipe()
    server_to_speech_pipe, speech_to_server_pipe = multiprocessing.Pipe()
    controller_to_sensors_pipe, sensors_to_controller_pipe = multiprocessing.Pipe()
    vision_to_server_pipe, server_to_vision_pipe = multiprocessing.Pipe()
    comms = multiprocessing.Process(
        target=server.listen,
        args=(server_to_controller_pipe, server_to_sensors_pipe, server_to_speech_pipe,
              server_to_vision_pipe))
    vision_to_display_pipe, display_to_vision_pipe = multiprocessing.Pipe()
    display = Display(args.mock)
    display_process = multiprocessing.Process(
        target=display.run, args=(display_to_vision_pipe,))
    controller = Controller(mock=args.mock, to_print=args.print_angles)
    control = multiprocessing.Process(target=controller.run, args=(
        controller_to_server_pipe, controller_to_sensors_pipe, controller_to_speech_pipe,))
    sensors_reader = Sensors(mock=args.mock)
    speech = Speech()
    speech_process = multiprocessing.Process(target=speech.listen, args=(speech_to_controller_pipe, speech_to_server_pipe,))
    sensors = multiprocessing.Process(
        target=sensors_reader.run,
        args=(sensors_to_server_pipe, sensors_to_controller_pipe))
    vision_server = Vision(mock=args.mock)
    vision = multiprocessing.Process(
        target=vision_server.run, args=(vision_to_server_pipe,
                                        vision_to_display_pipe))
    try:
        os.system('free -h')
        display_process.start()
        comms.start()
        sensors.start()
        speech_process.start()
        control.start()
        vision.start()
        print('Started...')

        display_process.join()
        comms.join()
        control.join()
        sensors.join()
        vision.join()
    except KeyboardInterrupt:
        comms.join()
        control.join()
        sensors.join()
        vision.join()
        print('Quitting...')
