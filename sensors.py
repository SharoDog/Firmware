import time
import board
import adafruit_lsm9ds1
import numpy as np
from digitalio import DigitalInOut, Direction
from ahrs.filters import Mahony
from ahrs import Quaternion


class Sensors:
    def __init__(self, mock):
        self.mock = mock
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
                acc_data.append(np.fromiter(
                    self.imu.acceleration, dtype=np.double))
                gyro_data.append(np.fromiter(self.imu.gyro, dtype=np.double))
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
                acc.append((np.fromiter(self.imu.acceleration,
                           dtype=np.double) - self.acc_offsets))
                #  mag.append(np.fromiter(self.imu.magnetic,
                #  dtype=np.double) * [1 / 100, 1 / 100, 1 / 100])
                gyro.append((np.fromiter(self.imu.gyro,
                            dtype=np.double) - self.gyro_offsets))
                self.last_calc = time.time()
            self.filter = Mahony(acc=np.asarray(acc),
                                 gyr=np.asarray(gyro),
                                 #  mag=np.asarray(mag),
                                 Dt=(self.last_calc - start_time) / 10.0,
                                 k_P=2.0,
                                 k_I=0.1)
            # initial attitude
            self.Q = Quaternion(self.filter.Q[-1])
            self.acc = acc[-1]
            self.gyro = gyro[-1]

    def run(self, attitude):
        try:
            while True:
                if not self.mock:
                    if self.imu_ready.value:
                        self.acc = (np.fromiter(self.imu.acceleration,
                                                dtype=np.double)
                                    - self.acc_offsets)
                        self.gyro = (np.fromiter(
                            self.imu.gyro, dtype=np.double)
                            - self.gyro_offsets)
                        #  mag = np.fromiter(self.imu.magnetic,
                        # dtype=np.double) * [1 / 100, 1 / 100, 1 / 100]
                        #  print(acc, gyro, mag)
                    self.filter.Dt = (time.time() - self.last_calc)
                    self.last_calc = time.time()
                    self.Q = Quaternion(self.filter.updateIMU(
                        q=self.Q.to_array(), acc=self.acc,
                        gyr=self.gyro,
                        #  mag=mag
                    ))
                    attitude[:] = self.Q.to_angles()
                else:
                    attitude[:] = [0.0, 0.0, 0.0]
                time.sleep(0)

        except KeyboardInterrupt:
            print('Killing sensors...')
