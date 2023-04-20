import board
from PIL import Image, ImageDraw, ImageFont, ImageOps
import adafruit_ssd1306
from server import Server
from controller import Controller
from sensors import Sensors
from vision import Vision
import multiprocessing
import argparse
multiprocessing.freeze_support()

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description='Firmware for Sharo')
        parser.add_argument(
            '-m', '--mock', action='store_true', help='Mock hardware.')
        parser.add_argument(
            '-pa', '--print-angles', action='store_true',
            help='Print servo angles when setting them.')
        args = parser.parse_args()
        if not args.mock:
            WIDTH = 128
            HEIGHT = 64
            BORDER = 4
            i2c = board.I2C()
            oled = adafruit_ssd1306.SSD1306_I2C(
                WIDTH, HEIGHT, i2c, addr=0x3C)
            x = []
            y = []
            for i in range(9):
                x.append(i * (oled.width - BORDER * 4) // 8 + BORDER)
                y.append(oled.height // 2 + ((i % 2) * 2 - 1) * 10)

            loading_image = Image.new('1', (oled.width, oled.height))
            font = ImageFont.truetype('ethnocentric.otf', size=24)
            face_image = Image.new('1', (oled.width, oled.height))
            face_draw = ImageDraw.Draw(face_image)
            face_draw.line(list(zip(x, y)), fill=1, width=3)
            oled.fill(0)
            oled.image(ImageOps.flip(face_image))
            oled.show()
        server = Server()
        server_to_controller_pipe, controller_to_server_pipe = multiprocessing.Pipe(
            duplex=True)
        server_to_sensors_pipe, sensors_to_server_pipe = multiprocessing.Pipe()
        controller_to_sensors_pipe, sensors_to_controller_pipe = multiprocessing.Pipe()
        comms = multiprocessing.Process(
            target=server.listen,
            args=(server_to_controller_pipe, server_to_sensors_pipe))
        controller = Controller(mock=args.mock, to_print=args.print_angles)
        control = multiprocessing.Process(target=controller.run, args=(
            controller_to_server_pipe, controller_to_sensors_pipe,))
        sensors_reader = Sensors(mock=args.mock)
        sensors = multiprocessing.Process(
            target=sensors_reader.run,
            args=(sensors_to_server_pipe, sensors_to_controller_pipe))
        comms.start()
        sensors.start()
        control.start()
        vision_server = Vision(mock=args.mock)
        vision = multiprocessing.Process(target=vision_server.listen)
        vision.start()
        print('Started...')

        comms.join()
        control.join()
        sensors.join()
        vision.join()
    except KeyboardInterrupt:
        print('Quitting...')
