from queue import Queue
from time import sleep
import numpy as np
import re
import os
from multiprocessing.connection import Connection

import json
from vosk import Model, KaldiRecognizer

class Speech():
    def __init__(self):
        self.regex = re.compile('[^a-zA-Z0-9]')
        self.enabled = True

    def listen(self, controller_pipe: Connection, server_pipe: Connection):
        try:
            import sounddevice as sd
            device = 'pulse' 
            device_info = sd.query_devices(device, 'input')
            self.samplerate = int(device_info['default_samplerate']) 
            audio_model = Model(lang="en-us")
            rec = KaldiRecognizer(audio_model, self.samplerate)
            q = Queue()
            print('Vosk loaded')
            print("Model loaded.\n")

            def callback(indata, frames, time, status):
                if status:
                    print(status, file=sys.stderr)
                q.put(bytes(indata))
            with sd.RawInputStream(samplerate=self.samplerate, blocksize = 8000, device=device,
                    dtype="int16", channels=1, callback=callback):
                while True:
                    # Pull raw recorded audio from the queue.
                    if self.enabled and not q.empty():
                        data = q.get()
                        if rec.AcceptWaveform(data):
                            text: str = self.regex.sub('', json.loads(rec.Result())['text']).lower()
                            if text == 'stan':
                                text = 'stand'
                            if text == 'dance':
                                text = 'emote3'
                            if text == 'giveleft':
                                text = 'emote2'
                            if text == 'giveright':
                                text = 'emote4'
                            if text == 'bow':
                                text = 'emote1'
                            controller_pipe.send(text) 
                    if server_pipe.readable and server_pipe.poll():
                        self.enabled = bool(server_pipe.recv())
                    sleep(0.01)
        except KeyboardInterrupt:
            print('Killing speech...')
            return
        except Exception:
            print('Voice commands not available.')
            return

