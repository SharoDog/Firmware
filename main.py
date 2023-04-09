import board
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
from server import Server
from controller import Controller
from sensors import Sensors
from vision import Vision
import multiprocessing
multiprocessing.freeze_support()

if __name__ == '__main__':
    try:
        WIDTH = 128
        HEIGHT = 64
        BORDER = 4
        i2c = board.I2C()
        oled = adafruit_ssd1306.SSD1306_I2C(
            WIDTH, HEIGHT, i2c, addr=0x3C)
        x = []
        y = []
        for i in range(9):
            x.append(i * (oled.width - BORDER * 2) // 8 + BORDER)
            y.append(oled.height // 2 + ((i % 2) * 2 - 1) * 15)

        loading_image = Image.new('1', (oled.width, oled.height))
        font = ImageFont.load_default()
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
        oled.image(loading_image)
        oled.show()
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
        vision = multiprocessing.Process(target=vision_server.listen)
        vision.start()
        print('Started...')
        oled.fill(0)
        oled.image(face_image)
        oled.show()

        comms.join()
        control.join()
        sensors.join()
        vision.join()
    except KeyboardInterrupt:
        print('Quitting...')
