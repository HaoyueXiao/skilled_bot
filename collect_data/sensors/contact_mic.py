import numpy as np
from collections import deque
from scipy.signal import spectrogram
import matplotlib.pyplot as plt
import librosa
import librosa.display
from audio_common_msgs.msg import AudioData

class ContactMicBuffer:
    def __init__(self, sample_rate: int, window_dur: int):
        self.sample_rate = sample_rate
        self.window_dur = window_dur
        self.buffer_size = sample_rate * window_dur  # Total samples to store
        self.window = deque(maxlen=self.buffer_size)
        print(f"[INFO] ContactMicBuffer initialized with buffer size: {self.buffer_size} samples")

    def add_audio_frame(self, audio_frame: np.ndarray):
        """
        Add an audio frame to the buffer.

        Args:
            audio_frame (np.ndarray): The audio frame to add to the buffer.
        """
        self.window.extend(audio_frame)
        print(f"[INFO] Added audio frame of size: {len(audio_frame)}")
        print(f"[INFO] Current buffer size: {len(self.window)}")

    def get_obs(self) -> dict:
        """
        Return the current buffer as an observation.

        Returns:
            dict: observation dictionary with key 'audio' and value as
            array of audio frames shape (num_samples,)
        """
        obs = {'audio': None}
        
        if len(self.window) > 0:
            all_audio = np.array(list(self.window))
            if len(all_audio) != self.buffer_size:
                # Left pad with zeros if buffer is not full
                padding = np.zeros(self.buffer_size - len(all_audio))
                all_audio = np.concatenate((padding, all_audio))
            obs['audio'] = all_audio
        else:
            print("[WARNING] No audio frames in observation")

        return obs

    def get_spectrogram(self):
        """
        Compute and return the Mel spectrogram of the stored audio data.

        Returns:
            mel_scale_sgram (ndarray): The Mel spectrogram of the audio data.
        """
        if len(self.window) == 0:
            print("[WARNING] No audio frames in buffer to compute spectrogram")
            return None

        # Get the current buffer as an array
        all_audio = np.array(list(self.window))

        # Compute the STFT
        sgram = librosa.stft(all_audio)
        sgram_mag, _ = librosa.magphase(sgram)
        
        # Compute the Mel spectrogram
        mel_scale_sgram = librosa.feature.melspectrogram(S=sgram_mag, sr=self.sample_rate)
        
        return mel_scale_sgram
    
def ros_audio_callback(msg, buffer):
    # Assuming the audio data is mono (single channel)
    audio_frame = np.frombuffer(msg.data, dtype=np.uint8).astype(np.float32) - 128
    buffer.add_audio_frame(audio_frame)

if __name__ == "__main__":
    import rospy

    rospy.init_node('contact_mic_buffer_node', anonymous=True)
    sample_rate = 16000
    window_dur = 2
    contact_mic_buffer = ContactMicBuffer(sample_rate=sample_rate, window_dur=window_dur)
    
    def callback(msg):
        ros_audio_callback(msg, contact_mic_buffer)

    rospy.Subscriber('/audio/audio', AudioData, callback)
    
    # Example of getting the observation and spectrogram
    rate = rospy.Rate(28)  # Callback rate in Hz
    plt.figure(figsize=(14, 5))
    while not rospy.is_shutdown():
        obs = contact_mic_buffer.get_obs()
        if obs['audio'] is not None:
            # Compute and display the Mel spectrogram
            mel_scale_sgram = contact_mic_buffer.get_spectrogram()
            if mel_scale_sgram is not None:
                print("Mel spectrogram shape:", mel_scale_sgram.shape)
                librosa.display.specshow(librosa.power_to_db(mel_scale_sgram, ref=np.max), sr=sample_rate, y_axis='mel', x_axis='time')
                plt.title("Mel Spectrogram")
                plt.colorbar(format='%+2.0f dB')
                plt.show()

        rate.sleep()