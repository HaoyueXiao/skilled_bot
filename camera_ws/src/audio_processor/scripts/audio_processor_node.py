#!/usr/bin/env python

import rospy
from audio_common_msgs.msg import AudioData
import wave

class AudioProcessor:
    def __init__(self):
        self.audio_data = bytearray()
        self.subscriber = rospy.Subscriber('/audio', AudioData, self.audio_callback)
        self.sample_rate = 44100  # Adjust as needed
        self.channels = 1  # Adjust as needed
        self.output_file = wave.open('/tmp/contact_mic_recording.wav', 'wb')
        self.output_file.setnchannels(self.channels)
        self.output_file.setsampwidth(2)  # Assuming 16-bit audio
        self.output_file.setframerate(self.sample_rate)

    def audio_callback(self, msg):
        self.audio_data.extend(msg.data)

    def save_audio(self):
        self.output_file.writeframes(self.audio_data)
        self.output_file.close()

if __name__ == '__main__':
    rospy.init_node('audio_processor_node', anonymous=True)
    processor = AudioProcessor()
    rospy.on_shutdown(processor.save_audio)
    rospy.spin()
