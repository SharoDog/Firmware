import board
from PIL import Image, ImageDraw, ImageFont
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
            for i in range(5):
                x.append(i * (oled.width - BORDER * 4) // 8 + BORDER)
                y.append(oled.height // 2 + ((i % 2) * 2 - 1) * 15)

            loading_image = Image.new('1', (oled.width, oled.height))
            font = ImageFont.truetype('ethnocentric.otf', size=24)
            text = "Loading..."
            (font_width, font_height) = font.getsize(text)
            loading_draw = ImageDraw.Draw(loading_image)
            loading_draw.text(
                (oled.width // 2 - font_width // 2,
                 oled.height // 2 - font_height // 2),
                text,
                font=font,
                fill=255,
            )
            face_image = Image.new('1', (oled.width, oled.height))
            face_draw = ImageDraw.Draw(face_image)
            face_draw.line(list(zip(x, y)), fill=1, width=3)
            oled.fill(0)
            loading_image.flip()
            oled.image(loading_image)
            oled.show()
        attitude = multiprocessing.Array('d', [0.0, 0.0, 0.0])
        server = Server()
        server_conn, controller_conn = multiprocessing.Pipe(duplex=True)
        comms = multiprocessing.Process(
            target=server.listen, args=(server_conn,))
        controller = Controller(mock=args.mock, to_print=args.print_angles)
        control = multiprocessing.Process(target=controller.run, args=(
            controller_conn, attitude,))
        sensors_reader = Sensors(mock=args.mock)
        sensors = multiprocessing.Process(
            target=sensors_reader.run, args=(attitude,))
        comms.start()
        sensors.start()
        control.start()
        vision_server = Vision(mock=args.mock)
        vision = multiprocessing.Process(target=vision_server.listen)
        vision.start()
        print('Started...')
        if not args.mock:
            oled.fill(0)
            face_image.flip()
            oled.image(face_image)
            oled.show()

        comms.join()
        control.join()
        sensors.join()
        vision.join()
    except KeyboardInterrupt:
        print('Quitting...')
