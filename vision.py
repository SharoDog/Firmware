import numpy as np
from multiprocessing.connection import Connection
import struct
import time
import socket
tflite = None
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow as tf
    tflite = tf.lite

try:
    from picamera2 import Picamera2
except ImportError:
    pass
try:
    import cv2
except ImportError:
    pass


class Vision():
    def __init__(self, mock):
        self.mock = mock
        self.face_detector = cv2.FaceDetectorYN_create(
            'yunet.onnx', '', (0, 0))
        self.fire_detector = cv2.CascadeClassifier('fire_detection.xml')
        self.emotions = ['Angry', 'Happy', 'Neutral', 'Sad', 'Surprise']
        self.emotion_labels = ['angry', 'disgust',
                               'fear', 'happy', 'sad', 'surprise', 'neutral']
        self.line_thickness = 2
        self.addr = None
        self.port = 9999
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65527)

    def run(self, server_pipe: Connection):
        # initialize camera and model in the new process
        # prevents access by multiple processes to the resource
        self.emotion_classifier = tflite.Interpreter(
            model_path='emotion_detection_model.tflite')
        self.emotion_classifier.allocate_tensors()
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
        try:
            while True:
                if server_pipe.readable and server_pipe.poll():
                    msg = server_pipe.recv()
                    if msg == 'dc':
                        self.addr = None
                    else:
                        self.addr = msg
                if self.addr:
                    if self.mock:
                        _, frame = self.camera.read()
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    else:
                        frame = self.camera.capture_array()
                    # CV
                    self.face_detector.setInputSize((640, 480))
                    _, faces = self.face_detector.detect(frame)
                    faces = faces if faces is not None else []
                    for face in faces:
                        box = list(map(int, face[:4]))
                        color = (0, 255, 0)
                        thickness = 2
                        cv2.rectangle(frame, box, color,
                                      thickness, cv2.LINE_AA)
                        try:
                            # preprocessing
                            gray_face = cv2.cvtColor(frame,
                                                     cv2.COLOR_RGB2GRAY)[
                                box[1]:box[1] + box[3], box[0]:box[0] + box[2]]
                            gray_face = cv2.resize(
                                gray_face,
                                (self.emotion_classifier.get_input_details()[0]
                                 ['shape'][1:3]),
                                interpolation=cv2.INTER_AREA)
                            gray_face = gray_face.astype('float32') / 255.0
                            gray_face = gray_face - 0.5
                            gray_face = gray_face * 2.0
                            gray_face = np.expand_dims(gray_face, 0)
                            gray_face = np.expand_dims(gray_face, -1)
                            self.emotion_classifier.set_tensor(
                                self.emotion_classifier.get_input_details()[0]
                                ['index'], gray_face)
                            self.emotion_classifier.invoke()
                            emotion_prediction = self.emotion_classifier.get_tensor(
                                self.emotion_classifier.get_output_details()[0]
                                ['index'])
                            emotion_label_arg = np.argmax(emotion_prediction)
                            label = self.emotion_labels[emotion_label_arg]
                            label_position = (box[0], box[1])
                            cv2.putText(
                                frame, label, label_position,
                                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                        except Exception as e:
                            print(e)

                    fire = self.fire_detector.detectMultiScale(frame, 1.2, 5)
                    for (x, y, w, h) in fire:
                        cv2.rectangle(frame, (x-20, y-20),
                                      (x+w+20, y+h+20),
                                      (255, 0, 0), self.line_thickness,
                                      cv2.LINE_AA)

                    # send
                    _, buffer = cv2.imencode(
                        '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
                    message = struct.pack('B' * len(buffer), *buffer)
                    self.socket.sendto(message,
                                       (self.addr, self.port))
                time.sleep(0.02)
        except KeyboardInterrupt:
            print('Killing vision server...')
            self.socket.close()
            if not self.mock:
                self.camera.stop()
            else:
                self.camera.release()
            return
