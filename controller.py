from ikpy.chain import Chain
import math
from adafruit_servokit import ServoKit
import numpy as np
import bezier


class Controller():
    def __init__(self):
        self.chain = Chain.from_urdf_file('leg.urdf', name='leg',
                                          active_links_mask=[
                                              False, True, True, True, False],
                                          last_link_vector=[9, 0, 0])
        self.femur_initial = -135
        self.tibia_initial = 90
        self.shoulder_offset = {'fr': 70, 'fl': 80, 'bl': 80, 'br': 75}
        self.femur_offset = {'fr': 80, 'fl': 85, 'bl': 80, 'br': 85}
        self.tibia_offset = {'fr': 90, 'fl': 80, 'br': 90, 'bl': 80}
        self.angles = [[0, 0, math.radians(self.femur_initial), math.radians(
            self.tibia_initial), 0] for i in range(4)]
        # walk
        front_line_nodes = np.asfortranarray([
            [7.0, -3.0],
            [-17.0, -17.0],
            [0.0, 0.0]
        ])
        front_line = bezier.Curve(front_line_nodes, degree=1)
        front_line_path = np.transpose(front_line.evaluate_multi(np.linspace(
            0.0, 1.0, 50)))

        front_curve_nodes = np.asfortranarray([
            [-3.0, 2.0, 7.0],
            [-17.0, -13.0, -17.0],
            [0.0, 0.0, 0.0],
        ])

        front_curve = bezier.Curve(front_curve_nodes, degree=2)
        front_curve_path = np.transpose(
            front_curve.evaluate_multi(np.linspace(0.0, 1.0, 50)))

        back_line_nodes = np.asfortranarray([
            [3.0, -9.0],
            [-15.0, -16.0],
            [0.0, 0.0]
        ])
        back_line = bezier.Curve(back_line_nodes, degree=1)
        back_line_path = np.transpose(back_line.evaluate_multi(np.linspace(
            0.0, 1.0, 50)))

        back_curve_nodes = np.asfortranarray([
            [-9.0, -2.0, 3.0],
            [-16.0, -12.0, -15.0],
            [0.0, 0.0, 0.0],
        ])

        back_curve = bezier.Curve(back_curve_nodes, degree=2)
        back_curve_path = np.transpose(
            back_curve.evaluate_multi(np.linspace(0.0, 1.0, 50)))
        self.points = {
            'lie': [[[0.0, -4.0, 0.0] for _ in range(4)]],
            'sit': [[[-5.0, -17.5, 0.0] for _ in range(2)]
                    + [[0.0, -4.0, 0.0] for _ in range(2)]],
            'stand': [[[0.0, -15.0, 0.0] for _ in range(4)]],
            'forward': np.transpose([np.concatenate(
                [front_line_path, front_curve_path]),
                np.concatenate(
                [front_curve_path, front_line_path]),
                np.concatenate(
                [back_curve_path, back_line_path]),
                np.concatenate(
                [back_line_path, back_curve_path]),
            ], axes=[1, 0, 2]),
            'backward': np.transpose([np.flip(np.concatenate(
                [front_line_path, front_curve_path]), axis=0),
                np.flip(np.concatenate(
                    [front_curve_path, front_line_path]), axis=0),
                np.flip(np.concatenate(
                    [back_curve_path, back_line_path]), axis=0),
                np.flip(np.concatenate(
                    [back_line_path, back_curve_path]), axis=0),
            ], axes=[1, 0, 2]),
            'right': np.transpose([np.flip(np.concatenate(
                [front_line_path, front_curve_path]), axis=1),
                np.flip(np.concatenate(
                    [front_curve_path, front_line_path]), axis=1),
                np.flip(np.concatenate(
                    [back_curve_path, back_line_path]), axis=1),
                np.flip(np.concatenate(
                    [back_line_path, back_curve_path]), axis=1),
            ], axes=[1, 0, 2]),
            'left': np.transpose([np.flip(np.concatenate(
                [front_line_path, front_curve_path])),
                np.flip(np.concatenate(
                    [front_curve_path, front_line_path])),
                np.flip(np.concatenate(
                    [back_curve_path, back_line_path])),
                np.flip(np.concatenate(
                    [back_line_path, back_curve_path])),
            ], axes=[1, 0, 2])
        }
        self.calc_paths()
        self.kit = ServoKit(channels=16)
        for i in range(16):
            self.kit.servo[i].set_pulse_width_range(500, 2500)

    def calc_paths(self):
        self.paths = {}
        for cmd, points in self.points.items():
            self.paths[cmd] = []
            angles = [[0, 0, math.radians(self.femur_initial), math.radians(
                self.tibia_initial), 0] for i in range(4)]
            for p in points:
                for i in range(4):
                    angles[i] = self.chain.inverse_kinematics(
                        p[i], initial_position=angles[i])
                self.paths[cmd].append(angles.copy())

    def run(self, msg_queue, quit_event):
        ind = 0
        curr_cmd = 'stand'
        while True:
            if quit_event.is_set():
                print('Killing controller...')
                return
            try:
                new_cmd = msg_queue.get_nowait()
                if curr_cmd != new_cmd:
                    # TODO: Transition
                    ind = 0
                    curr_cmd = new_cmd
            except Exception:
                pass
            self.angles = self.paths[curr_cmd][ind]
            ind = (ind + 1) % len(self.paths[curr_cmd])
            self.move_to(self.angles[0], self.angles[1],
                         self.angles[2], self.angles[3])

    def move_to(self, fl_angles, fr_angles, bl_angles, br_angles):
        # fl
        self.kit.servo[0].angle = -self.femur_initial + \
            math.degrees(
            fl_angles[2]) + self.femur_offset['fl']
        self.kit.servo[4].angle = - self.femur_initial + \
            math.degrees(fl_angles[2]) + \
            math.degrees(fl_angles[3]) - \
            self.tibia_initial + self.tibia_offset['fl']
        self.kit.servo[8].angle = - math.degrees(fl_angles[1]) + \
            self.shoulder_offset['fl']
        # fr
        self.kit.servo[1].angle = self.femur_initial - \
            math.degrees(
            fr_angles[2]) + self.femur_offset['fr']
        self.kit.servo[5].angle = self.femur_initial -\
            math.degrees(fr_angles[2]) - \
            math.degrees(fr_angles[3]) + \
            self.tibia_initial + self.tibia_offset['fr']
        self.kit.servo[9].angle = - math.degrees(fr_angles[1]) + \
            self.shoulder_offset['fr']
        # bl
        self.kit.servo[2].angle = -self.femur_initial + \
            math.degrees(
            bl_angles[2]) + self.femur_offset['bl']
        self.kit.servo[6].angle = - self.femur_initial + \
            math.degrees(bl_angles[2]) + \
            math.degrees(bl_angles[3]) - \
            self.tibia_initial + self.tibia_offset['bl']
        self.kit.servo[10].angle = math.degrees(bl_angles[1]) + \
            self.shoulder_offset['bl']
        # br
        self.kit.servo[3].angle = self.femur_initial - \
            math.degrees(
            br_angles[2]) + self.femur_offset['br']
        self.kit.servo[7].angle = self.femur_initial - \
            math.degrees(br_angles[2]) - \
            math.degrees(br_angles[3]) + \
            self.tibia_initial + self.tibia_offset['br']
        self.kit.servo[11].angle = math.degrees(br_angles[1]) + \
            self.shoulder_offset['br']
