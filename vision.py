import cv2
import numpy as np
from multiprocessing.connection import Connection
import struct
import time
import socket

try:
    from picamera2 import Picamera2
except ImportError:
    pass


class Vision():
    def __init__(self, mock):
        self.mock = mock
        self.face_detector = cv2.FaceDetectorYN_create(
            'yunet.onnx', '', (0, 0))
        self.emotion_classifier = cv2.dnn.readNet('emotion_classifier.onnx') 
        self.fire_detector = cv2.CascadeClassifier('fire_detection.xml')
        self.emotion_labels = ['angry', 'disgust',
                               'fearful', 'happy', 'neutral', 'sad', 'surprised']
        self.line_thickness = 2
        if self.mock:
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            self.camera = Picamera2()

            camera_config = self.camera.create_video_configuration(
                main={"size": (640, 480)}, raw={"size": (3280, 2464)})
            self.camera.configure(camera_config)
            self.camera.start()
        self.addr = None
        self.port = 9999
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65527)

    def run(self, server_pipe: Connection, display_pipe: Connection):
        try:
            while True:
                if server_pipe.readable and server_pipe.poll():
                    msg = server_pipe.recv()
                    if msg == 'dc':
                        self.addr = None
                    else:
                        self.addr = msg
                if self.mock:
                    _, frame = self.camera.read()
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                else:
                    frame = self.camera.capture_array()
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
                # CV
                self.face_detector.setInputSize((640, 480))
                _, faces = self.face_detector.detect(frame)
                faces = faces if faces is not None else []
                emotion = 'neutral'
                biggest_area = 0
                for face in faces:
                    box = list(map(int, face[:4]))
                    color = (0, 255, 0)
                    thickness = 2
                    cv2.rectangle(frame, box, color,
                                  thickness, cv2.LINE_AA)
                    if all(x >= 0 for x in box):
                        try:
                            # preprocessing
                            face = frame[
                            box[1]:box[1] + box[3], box[0]:box[0] + box[2]]
                            face = cv2.resize(
                            face,
                            (112, 112),
                            interpolation=cv2.INTER_AREA)
                            face = face.astype(np.float32, copy=False) / 255.0
                            face -= np.array([0.5, 0.5, 0.5])[np.newaxis, np.newaxis, :]
                            face /= np.array([0.5, 0.5, 0.5])[np.newaxis, np.newaxis, :]
                            input_blob = cv2.dnn.blobFromImage(face)
                            self.emotion_classifier.setInput(input_blob, 'data')
                            output_blob = self.emotion_classifier.forward(['label'])
                            emotion_label_arg = np.argmax(output_blob[0], axis=1).astype(np.uint8)[0]
                            label = self.emotion_labels[emotion_label_arg]
                            label_position = (box[0], box[1])
                            cv2.putText(
                            frame, label, label_position,
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                            if box[2] * box[3] > biggest_area:
                                emotion = label
                                biggest_area = box[2] * box[3]
                        except Exception as e:
                            print(e)
                        display_pipe.send(emotion)

                fire = self.fire_detector.detectMultiScale(frame, 1.2, 5)
                for (x, y, w, h) in fire:
                    cv2.rectangle(frame, (x-20, y-20),
                    (x+w+20, y+h+20),
                    (255, 0, 0), self.line_thickness,
                    cv2.LINE_AA)
                if self.addr:
                    # send
                    _, buffer = cv2.imencode(
                        '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
                    message = struct.pack('B' * len(buffer), *buffer)
                    self.socket.sendto(message,
                                       (self.addr, self.port))
                time.sleep(0)
        except KeyboardInterrupt:
            print('Killing vision server...')
            self.socket.close()
            if not self.mock:
                self.camera.stop()
            else:
                self.camera.release()
            return
