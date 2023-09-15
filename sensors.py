import time
import numpy as np
from ahrs.filters import Mahony
from ahrs import Quaternion
from multiprocessing.connection import Connection

try:
    import board
    import serial
    import adafruit_lsm9ds1
    import adafruit_gps
    from digitalio import DigitalInOut, Direction
except ImportError:
    pass


class Sensors:
    def __init__(self, mock):
        self.mock = mock
        self.enabled = True
        if not self.mock:
            self.spi = board.SPI()
            self.csag = DigitalInOut(board.D23)
            self.csag.direction = Direction.OUTPUT
            self.csag.value = True
            self.csm = DigitalInOut(board.D24)
            self.csm.direction = Direction.OUTPUT
            self.csm.value = True
            self.imu = adafruit_lsm9ds1.LSM9DS1_SPI(
                self.spi, self.csag, self.csm)

            self.imu_ready = DigitalInOut(board.D25)
            self.imu_ready.switch_to_input()
            print('Calibrating gyro and accelerometer...')
            acc_data = []
            gyro_data = []
            for i in range(1000):
                while not self.imu_ready.value:
                    pass
                acc_data.append(self.transform_acc_or_gyro_data(
                    np.fromiter(self.imu.acceleration, dtype=np.double)))
                gyro_data.append(self.transform_acc_or_gyro_data(
                    np.fromiter(self.imu.gyro, dtype=np.double)))
            acc_data = np.transpose(acc_data)
            self.acc_offsets = [sum(acc_data[0]) / len(acc_data[0]),
                                sum(acc_data[1]) / len(acc_data[1]),
                                sum(acc_data[2]) / len(acc_data[2]) - 9.81]
            self.acc_offsets = [0, 0, 0]
            gyro_data = np.transpose(gyro_data)
            self.gyro_offsets = [sum(gyro_data[0]) / len(gyro_data[0]),
                                 sum(gyro_data[1]) / len(gyro_data[1]),
                                 sum(gyro_data[2]) / len(gyro_data[2])]

            time.sleep(1)
            acc = []
            gyro = []
            #  mag = []
            start_time = time.time()
            # get 10 samples to estimate initial attitude
            for i in range(10):
                # wait for sensors to be ready
                while not self.imu_ready.value:
                    pass
                acc.append(self.transform_acc_or_gyro_data(
                    np.fromiter(self.imu.acceleration,
                                dtype=np.double) - self.acc_offsets))
                gyro.append(self.transform_acc_or_gyro_data(
                    np.fromiter(self.imu.gyro,
                                dtype=np.double) - self.gyro_offsets))
                #  mag.append(np.fromiter(self.imu.magnetic,
                #  dtype=np.double) * [1 / 100, 1 / 100, 1 / 100])
                self.last_calc = time.time()
            self.filter = Mahony(acc=np.asarray(acc),
                                 gyr=np.asarray(gyro),
                                 #  mag=np.asarray(mag),
                                 Dt=(self.last_calc - start_time) / 10.0)
            # initial attitude
            self.Q = Quaternion(self.filter.Q[-1])
            self.acc = acc[-1]
            self.gyro = gyro[-1]
            uart = serial.Serial('/dev/serial0', baudrate=9600, timeout=3000)
            self.gps = adafruit_gps.GPS(uart)
            # only minimal info
            self.gps.send_command(
                b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
            self.gps.send_command(b'PMTK220,1000')
            self.ultrasonic = serial.Serial(
                '/dev/ttyUSB1', baudrate=115200, timeout=100)

    def transform_acc_or_gyro_data(self, data):
        return [data[1], data[0], data[2]]

    def run(self, server_pipe: Connection, controller_pipe: Connection):
        try:
            while True:
                if self.enabled:
                    if not self.mock:
                        self.gps.update()
                        if self.imu_ready.value:
                            self.acc = self.transform_acc_or_gyro_data(
                                np.fromiter(self.imu.acceleration,
                                            dtype=np.double)
                                - self.acc_offsets)
                            self.gyro = self.transform_acc_or_gyro_data(
                                np.fromiter(self.imu.gyro, dtype=np.double)
                                - self.gyro_offsets)
                            #  mag = np.fromiter(self.imu.magnetic,
                            # dtype=np.double) / [100, 100, 100]
                        self.filter.Dt = (time.time() - self.last_calc)
                        self.last_calc = time.time()
                        self.Q = Quaternion(self.filter.updateIMU(
                            q=self.Q.to_array(), acc=self.acc,
                            gyr=self.gyro,
                            #  mag=mag
                        ))
                        server_pipe.send(
                            'IMU: ' + ';'.join(map(str, np.degrees(self.Q.to_angles()))))
                        if self.gps.has_fix:
                            server_pipe.send(
                                f'GPS: {self.gps.latitude};{self.gps.longitude};{self.gps.altitude_m + 440}')
                        else:
                            server_pipe.send(
                                'GPS: ' + ';'.join(map(str, [None, None, None])))
                        if self.ultrasonic.readable():
                            dist = self.ultrasonic.readline().decode().strip()
                            if dist and int(dist) != 0:
                                controller_pipe.send(
                                    f'US: {dist}')
                    else:
                        # mock IMU and GPS
                        server_pipe.send(
                            'IMU: ' + ';'.join(map(str, [0.0, 0.0, 0.0])))
                        server_pipe.send(
                            'GPS: ' + ';'.join(map(str, [None, None, None])))
                        time.sleep(0.5)
                if server_pipe.readable and server_pipe.poll():
                    self.enabled = bool(server_pipe.recv())

                time.sleep(0.02)

        except KeyboardInterrupt:
            print('Killing sensors...')
