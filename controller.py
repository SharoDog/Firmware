import random
import rutils
import time
import math
import numpy as np
import bezier
from multiprocessing.connection import Connection
from multiprocessing import Process, Pipe, Value
import ctypes

try:
    from adafruit_servokit import ServoKit
except ImportError:
    pass


class Controller():
    def __init__(self, mock, to_print):
        print('Initializing kinematics...')
        self.femur_initial = -135
        self.tibia_initial = 90
        self.shoulder_offset = {'fr': 75, 'fl': 90, 'bl': 85, 'br': 85}
        self.femur_offset = {'fr': 85, 'fl': 90, 'bl': 90, 'br': 95}
        self.tibia_offset = {'fr': 75, 'fl': 100, 'bl': 100, 'br': 85}
        self.speed = 1.0
        self.steering = 0.0
        self.angles = [[0, 0, math.radians(self.femur_initial), math.radians(
            self.tibia_initial), 0] for i in range(4)]
        self.default_steps = 25
        self.points = {
            'lie': [[[0.0, -4.0, 0.0] for _ in range(4)]],
            'sit': [[[-5.0, -17.5, 0.0] for _ in range(2)]
                    + [[0.0, -4.0, 0.0] for _ in range(2)]],
            'stand': [[[0.0, -15.0, 0.0] for _ in range(4)]],
            'forward': self.define_forward_walk(0.0, 1.0),
            'backward': self.define_backward_walk(0.0, 1.0),
            'clockwise': self.define_rotate(True, 1.0),
            'counterclockwise': self.define_rotate(False, 1.0),
            'right': self.define_sidestep(False, 1.0),
            'left': self.define_sidestep(True, 1.0),
            'steer right': self.define_steer(False, 1.0),
            'steer left': self.define_steer(True, 1.0),
            'off road': self.define_off_road_forward_walk(1.0),
            'emote1': self.define_emote1(),
            'emote2': self.define_emote2(),
            'emote3': self.define_emote3(),
            'emote4': self.define_emote4(),
        }
        self.calc_paths()
        if mock:
            self.kit = MockServoKit(to_print)
        else:
            self.kit = ServoKit(channels=16)
        for i in range(12):
            self.kit.servo[i].set_pulse_width_range(500, 2500)
        self.code_process = None
        # set lying position for sensors calibration
        self.move_to(self.paths['lie'][0][0], self.paths['lie'][0]
                     [1], self.paths['lie'][0][2], self.paths['lie'][0][3])
        # communication with user code
        self.code_pipe, self.code_to_controller_pipe = Pipe(duplex=False)
        self.distance = Value(ctypes.c_int, 0)
        self.pitch = Value(ctypes.c_double, 0)
        self.roll = Value(ctypes.c_double, 0)

    def define_forward_walk(self, steering: float, speed: float):
        num_points = int(self.default_steps // speed)
        # walk
        front_line_nodes = np.asfortranarray([
            [7.0 * speed, -5.0 * speed],
            [-16.0, -17.0],
            [0.0, 0.0]
        ])
        front_line = bezier.Curve(front_line_nodes, degree=1)
        front_line_path = np.transpose(front_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        front_curve_nodes = np.asfortranarray([
            [-5.0 * speed, 2.0, 7.0 * speed],
            [-17.0, -13.0, -16.0],
            [0.0, 0.0, 0.0],
        ])

        front_curve = bezier.Curve(front_curve_nodes, degree=2)
        front_curve_path = np.transpose(
            front_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))

        back_line_nodes = np.asfortranarray([
            [3.0 * speed, -9.0 * speed],
            [-15.5, -16.5],
            [0.0, 0.0]
        ])
        back_line = bezier.Curve(back_line_nodes, degree=1)
        back_line_path = np.transpose(back_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        back_curve_nodes = np.asfortranarray([
            [-9.0 * speed, -2.0, 3.0 * speed],
            [-16.5, -13.0, -15.5],
            [0.0, 0.0, 0.0],
        ])

        back_curve = bezier.Curve(back_curve_nodes, degree=2)
        back_curve_path = np.transpose(
            back_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        return np.transpose([np.concatenate(
            [front_line_path, front_curve_path]),
            np.concatenate(
            [front_curve_path, front_line_path]),
            np.concatenate(
            [back_curve_path, back_line_path]),
            np.concatenate(
            [back_line_path, back_curve_path]),
        ], axes=[1, 0, 2])

    def define_backward_walk(self, steering: float, speed: float):
        num_points = int(self.default_steps // speed)
        # walk
        front_line_nodes = np.asfortranarray([
            [-3.0 * speed, 8.0 * speed],
            [-17.0, -16.0],
            [0.0, 0.0]
        ])
        front_line = bezier.Curve(front_line_nodes, degree=1)
        front_line_path = np.transpose(front_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        front_curve_nodes = np.asfortranarray([
            [8.0 * speed, 2.5 * speed, -3.0 * speed],
            [-16.0, -12.0, -17.0],
            [0.0, 0.0, 0.0],
        ])

        front_curve = bezier.Curve(front_curve_nodes, degree=2)
        front_curve_path = np.transpose(
            front_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))

        back_line_nodes = np.asfortranarray([
            [-4.0 * speed, 6.0 * speed],
            [-16.0, -15.0],
            [0.0, 0.0]
        ])
        back_line = bezier.Curve(back_line_nodes, degree=1)
        back_line_path = np.transpose(back_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        back_curve_nodes = np.asfortranarray([
            [6.0 * speed, 0.0 * speed, -4.0 * speed],
            [-15.0, -13.0, -16.0],
            [0.0, 0.0, 0.0],
        ])

        back_curve = bezier.Curve(back_curve_nodes, degree=2)
        back_curve_path = np.transpose(
            back_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        return np.transpose([np.concatenate(
            [front_line_path, front_curve_path]),
            np.concatenate(
            [front_curve_path, front_line_path]),
            np.concatenate(
            [back_curve_path, back_line_path]),
            np.concatenate(
            [back_line_path, back_curve_path]),
        ], axes=[1, 0, 2])

    def define_off_road_forward_walk(self, speed: float):
        num_points = int(25 // speed)
        # walk
        front_line_nodes = np.asfortranarray([
            [7.0, -5.0],
            [-16.0, -17.0],
            [2.0, 2.0]
        ])
        front_line = bezier.Curve(front_line_nodes, degree=1)
        front_line_path = np.transpose(front_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        front_curve_nodes = np.asfortranarray([
            [-5.0, 2.0, 7.0],
            [-17.0, -5.0, -16.0],
            [2.0, 2.0, 2.0],
        ])

        front_curve = bezier.Curve(front_curve_nodes, degree=2)
        front_curve_path = np.transpose(
            front_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))

        back_line_nodes = np.asfortranarray([
            [3.0, -9.0],
            [-15.0, -16.0],
            [2.0, 2.0]
        ])
        back_line = bezier.Curve(back_line_nodes, degree=1)
        back_line_path = np.transpose(back_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        back_curve_nodes = np.asfortranarray([
            [-9.0, -2.0, 3.0],
            [-16.0, -5.0, -15.0],
            [2.0, 2.0, 2.0],
        ])

        back_curve = bezier.Curve(back_curve_nodes, degree=2)
        back_curve_path = np.transpose(
            back_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        return np.transpose([np.concatenate(
            [front_line_path, front_curve_path]),
            np.concatenate(
            [front_curve_path, front_line_path]),
            np.concatenate(
            [back_curve_path, back_line_path]),
            np.concatenate(
            [back_line_path, back_curve_path]),
        ], axes=[1, 0, 2])

    def define_sidestep(self, left: bool, speed: float):
        num_points = int(self.default_steps // speed)
        # fi
        fi_line_nodes = np.asfortranarray([
            [-2.0, -2.0],
            [-16.0, -17.0],
            [5.0, -3.0],
        ])
        fi_line = bezier.Curve(fi_line_nodes, degree=1)
        fi_line_path = np.transpose(fi_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        fi_curve_nodes = np.asfortranarray([
            [-2.0, -2.0, -2.0],
            [-17.0, -13.0, -16.0],
            [-3.0, 1.0, 5.0],
        ])

        fi_curve = bezier.Curve(fi_curve_nodes, degree=2)
        fi_curve_path = np.transpose(
            fi_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        # fo
        fo_line_nodes = np.asfortranarray([
            [-2.0, -2.0],
            [-16.0, -17.0],
            [-3.0, 5.0],
        ])
        fo_line = bezier.Curve(fo_line_nodes, degree=1)
        fo_line_path = np.transpose(fo_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        fo_curve_nodes = np.asfortranarray([
            [-2.0, -2.0, -2.0],
            [-17.0, -13.0, -16.0],
            [5.0, 1.0, -3.0],
        ])

        fo_curve = bezier.Curve(fo_curve_nodes, degree=2)
        fo_curve_path = np.transpose(
            fo_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        # bi
        bi_line_nodes = np.asfortranarray([
            [-2.0, -2.0],
            [-15.0, -16.0],
            [5.0, -3.0],
        ])
        bi_line = bezier.Curve(bi_line_nodes, degree=1)
        bi_line_path = np.transpose(bi_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        bi_curve_nodes = np.asfortranarray([
            [-2.0, -2.0, -2.0],
            [-16.0, -13.0, -15.0],
            [-3.0, 1.0, 5.0],
        ])

        bi_curve = bezier.Curve(bi_curve_nodes, degree=2)
        bi_curve_path = np.transpose(
            bi_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        # bo
        bo_line_nodes = np.asfortranarray([
            [-2.0, -2.0],
            [-15.0, -16.0],
            [-3.0, 5.0],
        ])
        bo_line = bezier.Curve(bo_line_nodes, degree=1)
        bo_line_path = np.transpose(bo_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        bo_curve_nodes = np.asfortranarray([
            [-2.0, -2.0, -2.0],
            [-16.0, -13.0, -15.0],
            [5.0, 1.0, -3.0],
        ])

        bo_curve = bezier.Curve(bo_curve_nodes, degree=2)
        bo_curve_path = np.transpose(
            bo_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        if left:
            return np.transpose([np.concatenate(
                [fi_line_path, fi_curve_path]),
                np.concatenate(
                [fo_curve_path, fo_line_path]),
                np.concatenate(
                [bi_curve_path, bi_line_path]),
                np.concatenate(
                [bo_line_path, bo_curve_path]),
            ], axes=[1, 0, 2])
        else:
            return np.transpose([np.concatenate(
                [fo_line_path, fo_curve_path]),
                np.concatenate(
                [fi_curve_path, fi_line_path]),
                np.concatenate(
                [bo_curve_path, bo_line_path]),
                np.concatenate(
                [bi_line_path, bi_curve_path]),
            ], axes=[1, 0, 2])

    def define_steer(self, left, speed: float):
        num_points = int(25 // speed)
        # fi
        fi_line_nodes = np.asfortranarray([
            [5.0, -3.0],
            [-16.0, -17.0],
            [-3.0, -3.0]
        ])
        fi_line = bezier.Curve(fi_line_nodes, degree=1)
        fi_line_path = np.transpose(fi_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        fi_curve_nodes = np.asfortranarray([
            [-3.0, 2.0, 5.0],
            [-17.0, -13.0, -16.0],
            [-3.0, -3.0, -3.0],
        ])

        fi_curve = bezier.Curve(fi_curve_nodes, degree=2)
        fi_curve_path = np.transpose(
            fi_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        # fo
        fo_line_nodes = np.asfortranarray([
            [8.0, -6.0],
            [-16.0, -17.0],
            [0.0, 0.0]
        ])
        fo_line = bezier.Curve(fo_line_nodes, degree=1)
        fo_line_path = np.transpose(fo_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        fo_curve_nodes = np.asfortranarray([
            [-6.0, 2.0, 8.0],
            [-17.0, -13.0, -16.0],
            [0.0, 0.0, 0.0],
        ])

        fo_curve = bezier.Curve(fo_curve_nodes, degree=2)
        fo_curve_path = np.transpose(
            fo_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        # bi
        bi_line_nodes = np.asfortranarray([
            [1.0, -7.0],
            [-15.0, -16.0],
            [-3.0, -3.0]
        ])
        bi_line = bezier.Curve(bi_line_nodes, degree=1)
        bi_line_path = np.transpose(bi_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        bi_curve_nodes = np.asfortranarray([
            [-7.0, -2.0, 1.0],
            [-16.0, -12.0, -15.0],
            [-3.0, -3.0, -3.0],
        ])

        bi_curve = bezier.Curve(bi_curve_nodes, degree=2)
        bi_curve_path = np.transpose(
            bi_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        # bo
        bo_line_nodes = np.asfortranarray([
            [4.0, -10.0],
            [-15.0, -16.0],
            [-1.0, -1.0]
        ])
        bo_line = bezier.Curve(bo_line_nodes, degree=1)
        bo_line_path = np.transpose(bo_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        bo_curve_nodes = np.asfortranarray([
            [-10.0, -2.0, 4.0],
            [-16.0, -12.0, -15.0],
            [-1.0, -1.0, -1.0],
        ])

        bo_curve = bezier.Curve(bo_curve_nodes, degree=2)
        bo_curve_path = np.transpose(
            bo_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        if left:
            return np.transpose([np.concatenate(
                [fi_line_path, fi_curve_path]),
                np.concatenate(
                [fo_curve_path, fo_line_path]),
                np.concatenate(
                [bi_curve_path, bi_line_path]),
                np.concatenate(
                [bo_line_path, bo_curve_path]),
            ], axes=[1, 0, 2])
        else:
            return np.transpose([np.concatenate(
                [fo_line_path, fo_curve_path]),
                np.concatenate(
                [fi_curve_path, fi_line_path]),
                np.concatenate(
                [bo_curve_path, bo_line_path]),
                np.concatenate(
                [bi_line_path, bi_curve_path]),
            ], axes=[1, 0, 2])

    def define_rotate(self, clockwise, speed):
        num_points = int(self.default_steps // speed)
        first_line_nodes = np.asfortranarray([[0.0, 0.0],
                                              [-16.0, -17.0],
                                              [-5.0, 2.0],
                                              ])
        first_line = bezier.Curve(first_line_nodes, degree=1)
        first_line_path = np.transpose(first_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        first_curve_nodes = np.asfortranarray([
            [0.0, 0.0, 0.0],
            [-17.0, -13.0, -16.0],
            [2.0, -1.0, -5.0],
        ])

        first_curve = bezier.Curve(first_curve_nodes, degree=2)
        first_curve_path = np.transpose(first_curve.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))
        second_line_nodes = np.asfortranarray([
            [0.0, 0.0],
            [-17.0, -16.0],
            [2.0, -5.0],
        ])
        second_line = bezier.Curve(second_line_nodes, degree=1)
        second_line_path = np.transpose(second_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        second_curve_nodes = np.asfortranarray([
            [0.0, 0.0, 0.0],
            [-16.0, -13.0, -17.0],
            [-5.0, -1.0, 2.0],
        ])

        second_curve = bezier.Curve(second_curve_nodes, degree=2)
        second_curve_path = np.transpose(
            second_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        if clockwise:
            return np.transpose([np.concatenate(
                [first_line_path, first_curve_path]),
                np.concatenate(
                [second_curve_path, second_line_path]),
                np.concatenate(
                [second_curve_path, second_line_path]),
                np.concatenate(
                [first_line_path, first_curve_path])],
                axes=[1, 0, 2])
        else:
            return np.transpose([np.concatenate(
                [second_line_path, second_curve_path]),
                np.concatenate(
                [first_curve_path, first_line_path]),
                np.concatenate(
                [first_curve_path, first_line_path]),
                np.concatenate(
                [second_line_path, second_curve_path])],
                axes=[1, 0, 2])

    def define_emote1(self):
        return [[[14.0, -7.0, 1.0], [14.0, -7.0, 1.0]]
                + [[0.0, -18.0, 1.0], [0.0, -18.0, 1.0]]]

    def define_emote2(self):
        num_points = 25
        # sidestep
        front_line_nodes = np.asfortranarray([
            [-2.0, -2.0],
            [-16.0, -17.0],
            [7.0, -5.0],
        ])
        front_line = bezier.Curve(front_line_nodes, degree=1)
        front_line_path = np.transpose(front_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        front_curve_nodes = np.asfortranarray([
            [-2.0, -2.0, -2.0],
            [-17.0, -13.0, -16.0],
            [-5.0, 2.0, 7.0],
        ])

        front_curve = bezier.Curve(front_curve_nodes, degree=2)
        front_curve_path = np.transpose(
            front_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))

        back_line_nodes = np.asfortranarray([
            [-2.0, -2.0],
            [-15.0, -16.0],
            [3.0, -9.0],
        ])
        back_line = bezier.Curve(back_line_nodes, degree=1)
        back_line_path = np.transpose(back_line.evaluate_multi(np.linspace(
            0.0, 1.0, num_points)))

        back_curve_nodes = np.asfortranarray([
            [-2.0, -2.0, -2.0],
            [-16.0, -12.0, -15.0],
            [-9.0, -2.0, 3.0],
        ])

        back_curve = bezier.Curve(back_curve_nodes, degree=2)
        back_curve_path = np.transpose(
            back_curve.evaluate_multi(np.linspace(0.0, 1.0, num_points)))
        return np.transpose([np.concatenate(
            [front_line_path, front_curve_path]),
            np.concatenate(
            [front_curve_path, front_line_path]),
            np.concatenate(
            [back_curve_path, back_line_path]),
            np.concatenate(
            [back_line_path, back_curve_path]),
        ], axes=[1, 0, 2])

    def define_emote3(self):
        return [[[10.0, -6.0, 0.0], [-8.0, -15.0, -4.0]]
                + [[0.0, -10.0, 0.0], [0.0, -4.0, 0.0]]]

    def define_emote4(self):
        return [[[-8.0, -15.0, -4.0], [10.0, -4.0, 0.0]]
                + [[0.0, -4.0, 0.0], [0.0, -10.0, 0.0]]]

    def calc_paths(self, command=None):
        if command:
            self.paths[command] = rutils.path_ik(self.points[command])
        else:
            self.paths = {}
            for cmd, points in self.points.items():
                self.paths[cmd] = rutils.path_ik(points)

    def transition(self, curr_angls, new_path):
        angls = np.asarray(curr_angls)
        ind = 0
        best_score = None
        for (i, p) in enumerate(new_path):
            pos = np.asarray(p)
            curr_score = sum(
                [ang ** 2 for ang in np.ravel(pos - angls)])
            if not best_score or curr_score < best_score:
                ind = i
                best_score = curr_score
        return ind
    
    def set_speed(self, speed):
        speed = 1 + (self.speed - 1) * 0.5
        self.points['forward'] = self.define_forward_walk(
            self.steering, speed)
        self.calc_paths('forward')
        self.points[
            'backward'] = self.define_backward_walk(
            self.steering, speed)
        self.calc_paths('backward')
        self.points[
            'off road'] = self.define_off_road_forward_walk(speed)
        self.calc_paths('off road')
        self.points[
            'left'] = self.define_sidestep(True, speed)
        self.calc_paths('left')
        self.points[
            'right'] = self.define_sidestep(False, speed)
        self.calc_paths('right')
        self.points[
            'steer left'] = self.define_steer(True, speed)
        self.calc_paths('steer left')
        self.points[
            'steer right'] = self.define_steer(False, speed)
        self.calc_paths('steer right')
        self.points[
            'clockwise'] = self.define_rotate(True, speed)
        self.calc_paths('clockwise')
        self.points[
            'counterclockwise'] = self.define_rotate(False, speed)
        self.calc_paths('counterclockwise')

    def run(self, server_pipe: Connection, sensors_pipe: Connection):
        try:
            ind = 0
            curr_cmd = 'lie'
            while True:
                try:
                    if self.code_process:
                        if self.code_process.is_alive() or (self.code_pipe.readable and self.code_pipe.poll()):
                            if self.code_pipe.readable and self.code_pipe.poll():
                                msg: str = self.code_pipe.recv()
                                if msg.startswith('speed'):
                                    self.speed = float(msg.split(':')[1].strip())     
                                    self.set_speed(self.speed)
                                    server_pipe.send('speed: ' + str(self.speed))
                                else:
                                    if msg.startswith('steer'):
                                        if msg == 'steer left':
                                            server_pipe.send('steering: -0.5')
                                        else:
                                            server_pipe.send('steering: 0.5')
                                    new_cmd = msg
                                    if curr_cmd != new_cmd and new_cmd in self.paths:
                                        ind = self.transition(
                                            self.angles, self.paths[new_cmd])
                                        curr_cmd = new_cmd
                                        server_pipe.send('command: ' + new_cmd)
                        elif not self.code_process.is_alive():
                            self.code_process = None
                            server_pipe.send('codestop')
                    if server_pipe.readable and server_pipe.poll():
                        msg: str = server_pipe.recv()
                        if msg.startswith('code:'): 
                            code = compile(msg[len('code:'):], 'usercode', 'exec')
                            args = {
                                'command': self.code_to_controller_pipe,
                                'distance': self.distance,
                                'pitch': self.pitch,
                                'roll': self.pitch
                            }
                            self.code_process = Process(target=exec, args=(code, None, args))
                            self.code_process.start()
                            server_pipe.send('code')
                        elif msg.startswith('speed'):
                            self.speed = float(msg.split(':')[1].strip())     
                            self.set_speed(self.speed)
                            server_pipe.send('speed: ' + str(self.speed))
                        else:
                            if msg.startswith('steering'):
                                self.steering = float(msg.split(':')[1].strip())
                                if self.steering < 0.0:
                                    new_cmd = 'steer left'
                                elif self.steering > 0.0:
                                    new_cmd = 'steer right'
                                else:
                                    new_cmd = 'forward'
                                server_pipe.send('steering: ' + str(self.steering))
                            else:
                                new_cmd = msg
                            if self.code_process and self.code_process.is_alive():
                                self.code_process.kill()
                                self.code_process.join()
                                self.code_process = None
                                server_pipe.send('codestop')
                            if curr_cmd != new_cmd and new_cmd in self.paths:
                                ind = self.transition(
                                    self.angles, self.paths[new_cmd])
                                curr_cmd = new_cmd
                                server_pipe.send('command: ' + new_cmd)
                    if sensors_pipe.readable and sensors_pipe.poll():
                        # prevent pipe hang
                        msg: str = sensors_pipe.recv()
                        if msg.startswith('US'):
                            dist = int(msg.split(':')[1].strip())
                            self.distance.value = dist
                            if not self.code_process and dist < 15 and (curr_cmd == 'forward' or
                                              curr_cmd == 'off road'):
                                ind = 0
                                curr_cmd = 'stand'
                                server_pipe.send('command: stand')
                except Exception as e:
                    print(e)
                try:
                    self.angles = self.paths[curr_cmd][ind]
                    ind = (ind + 1) % len(self.paths[curr_cmd])
                    self.move_to(self.angles[0], self.angles[1],
                                 self.angles[2], self.angles[3])
                except Exception:
                    pass
        except KeyboardInterrupt:
            print('Killing controller...')
            if self.code_process and self.code_process.is_alive():
                print('Killing user code...')
                self.code_process.kill()
                self.code_process.join()
            return

    def move_to(self, fl_angles, fr_angles, bl_angles, br_angles):
        # fl
        self.kit.servo[0].angle = -self.femur_initial + \
            math.degrees(
            fl_angles[1]) + self.femur_offset['fl']
        self.kit.servo[4].angle = - self.femur_initial + \
            math.degrees(fl_angles[1]) +\
            math.degrees(fl_angles[2]) -\
            self.tibia_initial + self.tibia_offset['fl']
        self.kit.servo[8].angle = math.degrees(fl_angles[0]) +\
            self.shoulder_offset['fl']
        # fr
        self.kit.servo[1].angle = self.femur_initial - \
            math.degrees(
            fr_angles[1]) + self.femur_offset['fr']
        self.kit.servo[5].angle = self.femur_initial -\
            math.degrees(fr_angles[1]) - \
            math.degrees(fr_angles[2]) + \
            self.tibia_initial + self.tibia_offset['fr']
        self.kit.servo[9].angle = - math.degrees(fr_angles[0]) +\
            self.shoulder_offset['fr']
        # bl
        self.kit.servo[2].angle = -self.femur_initial + \
            math.degrees(
            bl_angles[1]) + self.femur_offset['bl']
        self.kit.servo[6].angle = - self.femur_initial +\
            math.degrees(bl_angles[1]) + \
            math.degrees(bl_angles[2]) - \
            self.tibia_initial + self.tibia_offset['bl']
        self.kit.servo[10].angle = -math.degrees(bl_angles[0]) + \
            self.shoulder_offset['bl']
        # br
        self.kit.servo[3].angle = self.femur_initial - \
            math.degrees(
            br_angles[1]) + self.femur_offset['br']
        self.kit.servo[7].angle = self.femur_initial - \
            math.degrees(br_angles[1]) - \
            math.degrees(br_angles[2]) + \
            self.tibia_initial + self.tibia_offset['br']
        self.kit.servo[11].angle = math.degrees(br_angles[0]) + \
            self.shoulder_offset['br']


class MockServoKit():
    def __init__(self, to_print):
        self.servo = [
            MockServo('fl_femur', to_print),
            MockServo('fr_femur', to_print),
            MockServo('bl_femur', to_print),
            MockServo('br_femur', to_print),
            MockServo('fl_tibia', to_print),
            MockServo('fr_tibia', to_print),
            MockServo('bl_tibia', to_print),
            MockServo('br_tibia', to_print),
            MockServo('fl_shoulder', to_print),
            MockServo('fr_shoulder', to_print),
            MockServo('bl_shoulder', to_print),
            MockServo('br_shoulder', to_print)
        ]


class MockServo:
    def __init__(self, role, to_print):
        self._angle = 0
        self.role = role
        self.to_print = to_print

    @ property
    def angle(self) -> int:
        return self._angle

    @ angle.setter
    def angle(self, value: int):
        self._angle = value
        if self.to_print:
            print(f'{self.role}: {self._angle}')

    def set_pulse_width_range(self, lower_bound: int, upper_bound: int):
        print(f'{self.role} Pulse width range: {lower_bound} - {upper_bound}')
