import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import sys
import cv2

sounddevice_path = "/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/src/"
sys.path.append(sounddevice_path)
from sounddevice_ros.msg import AudioInfo, AudioData
from collections import deque
import librosa
import librosa.display

if __name__ == '__main__':
    AUDIO_FILE = '/home/jianrenw/Documents/skilled/audio_test.wav'
    samples, sample_rate = librosa.load(AUDIO_FILE, sr=None)
    plt.figure(figsize=(14, 5))
    sgram = librosa.stft(samples[int(4*sample_rate):int(8.5*sample_rate)])
    sgram_mag, _ = librosa.magphase(sgram)
    mel_scale_sgram = librosa.feature.melspectrogram(S=sgram_mag, sr=sample_rate)
    mel_sgram = librosa.amplitude_to_db(mel_scale_sgram, ref=np.min)
    #mel_sgram = cv2.resize(mel_sgram, (640, 480), interpolation=cv2.INTER_AREA)
    plt.imshow(mel_sgram)
    #librosa.display.specshow(mel_sgram, sr=sample_rate, x_axis='time', y_axis='mel')
    plt.colorbar(format='%+2.0f dB')
    plt.show()