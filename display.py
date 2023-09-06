import time
from PIL import Image, ImageDraw, ImageFont, ImageOps
from multiprocessing.connection import Connection
import numpy as np
# hw
try:
    import board
    import adafruit_ssd1306
except ImportError:
    pass
# simulation
try:
    import cv2
except ImportError:
    pass


class Display:
    def __init__(self, mock):
        self.mock = mock
        self.emotion = ''
        self.prev_emotion = ''
        self.BORDER = 4
        self.WIDTH = 128
        self.HEIGHT = 64
        if not self.mock:
            self.i2c = board.I2C()
            self.oled = adafruit_ssd1306.SSD1306_I2C(
                self.WIDTH, self.HEIGHT, self.i2c, addr=0x3C)
        x = []
        y = []
        for i in range(9):
            x.append(i * (self.WIDTH - self.BORDER * 2) //
                     8 + self.BORDER)
            y.append(self.HEIGHT // 2 + ((i % 2) * 2 - 1) * 10)

        self.default_image = Image.new(
            '1', (self.WIDTH, self.HEIGHT))
        default_draw = ImageDraw.Draw(self.default_image)
        font = ImageFont.truetype('ethnocentric.otf', size=14)
        _, _, w, h = default_draw.textbbox((0, 0), 'SHARO', font=font)
        default_draw.text(((self.WIDTH - w) / 2, (self.HEIGHT - h) / 2),
                          text='SHARO', font=font, fill=1)
        self.neutral_image = Image.new(
            '1', (self.WIDTH, self.HEIGHT))
        neutral_draw = ImageDraw.Draw(self.neutral_image)
        neutral_draw.rectangle(((20, self.HEIGHT / 2 - 5),
                               (self.WIDTH - 20, self.HEIGHT / 2 + 5)), fill=1)
        self.happy_image = Image.new('1', (self.WIDTH, self.HEIGHT))
        happy_draw = ImageDraw.Draw(self.happy_image)
        happy_draw.arc(((25, 10), (self.WIDTH - 25, self.HEIGHT - 10)),
                       20, 160, width=3, fill=1)
        self.sad_image = Image.new('1', (self.WIDTH, self.HEIGHT))
        sad_draw = ImageDraw.Draw(self.sad_image)
        sad_draw.arc(((25, 10 + self.HEIGHT / 2),
                      (self.WIDTH - 25, 3 * (self.HEIGHT - 10) / 2)),
                     200, 340, width=3, fill=1)
        self.fear_image = Image.new(
            '1', (self.WIDTH, self.HEIGHT))
        fear_draw = ImageDraw.Draw(self.fear_image)
        fear_draw.line(list(zip(x, y)), fill=1, width=3)
        self.surprise_image = Image.new('1', (self.WIDTH, self.HEIGHT))
        surprise_draw = ImageDraw.Draw(self.surprise_image)
        surprise_radius = (min(self.WIDTH, self.HEIGHT) * 8 / 10) / 2
        surprise_draw.ellipse(
            ((self.WIDTH / 2 - surprise_radius,
              self.HEIGHT / 2 - surprise_radius),
             (self.WIDTH / 2 + surprise_radius,
              self.HEIGHT / 2 + surprise_radius)), fill=1)
        surprise_radius -= 2
        surprise_draw.ellipse(
            ((self.WIDTH / 2 - surprise_radius,
              self.HEIGHT / 2 - surprise_radius),
             (self.WIDTH / 2 + surprise_radius,
              self.HEIGHT / 2 + surprise_radius)), fill=0)
        self.images = {
            'angry': self.fear_image,
            'happy': self.happy_image,
            'sad': self.sad_image,
            'neutral': self.neutral_image,
            'fearful': self.fear_image,
            'disgust': self.fear_image,
            'surprised': self.surprise_image}
        if not self.mock:
            #  self.default_image = ImageOps.flip(self.default_image)
            for key, val in self.images.items():
                self.images[key] = ImageOps.flip(val)
        self.image = self.default_image
        if not self.mock:
            self.oled.fill(0)
            self.oled.image(self.image)
            self.oled.show()
        self.dt = 1
        self.last_switch = time.time() 

    def run(self, vision_pipe: Connection):
        while True:
            try:
                # prevent pipe hang
                if vision_pipe.readable and vision_pipe.poll():
                    self.emotion: str = vision_pipe.recv()
                    if not self.prev_emotion:
                        self.prev_emotion = self.emotion
                if time.time() - self.last_switch > self.dt and self.emotion in self.images:
                    self.image = self.images[self.emotion]
                    self.last_switch = time.time()
                if not self.mock:
                    if self.emotion != self.prev_emotion:
                        self.oled.fill(0)
                        self.oled.image(self.image)
                        self.oled.show()
                else:
                    cv2.imshow(
                        'OLED', np.array(self.image,
                                         dtype=np.uint8) * 255,
                    )
                    cv2.pollKey()
                self.prev_emotion = self.emotion
                time.sleep(0.02)
            except KeyboardInterrupt:
                print('Killing display...')
                return
