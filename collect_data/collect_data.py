# -- coding: UTF-8
import os
import time
import numpy as np
import h5py
import argparse
import dm_env

import collections
from collections import deque
from sounddevice_ros.msg import AudioInfo, AudioData
import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import sys
import select
import threading
import cv2

exit_flag = False


def key_capture_thread():
    global exit_flag
    while True:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line:
                exit_flag = True
                return


# 保存数据函数
def save_data(args, timesteps, actions, dataset_path, chunk_size=100):
    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/action': [],
        '/base_action': [],
    }
    
    for cam_name in args.camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []
        if args.use_depth_image:
            data_dict[f'/observations/images_depth/{cam_name}'] = []
    if args.use_zed_depth and not args.use_depth_image:
        data_dict[f'/observations/images_depth/{args.camera_names[0]}'] = []
    if args.use_audio:
            data_dict['/observations/audio/audio_l'] = []
            data_dict['/observations/audio/audio_r'] = []

    # Open the file and create datasets
    t0 = time.time()
    print("start saving...")
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        root.attrs['compress'] = False

        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in args.camera_names:
            _ = image.create_dataset(cam_name, (0, 480, 640, 3), maxshape=(None, 480, 640, 3), dtype='uint8',
                                     chunks=(1, 480, 640, 3), compression='gzip', compression_opts=9)

        if args.use_zed_depth and not args.use_depth_image:
            image_depth = obs.create_group('images_depth')
            _ = image_depth.create_dataset(args.camera_names[0], (0, 480, 640, 1), maxshape=(None, 480, 640, 1), dtype='float32',
                                           chunks=(1, 480, 640, 1), compression='gzip', compression_opts=9)

        if args.use_depth_image:
            image_depth = obs.create_group('images_depth')
            for cam_name in args.camera_names:
                _ = image_depth.create_dataset(cam_name, (0, 480, 640), maxshape=(None, 480, 640), dtype='uint8',
                                               chunks=(1, 480, 640), compression='gzip', compression_opts=9)

        _ = obs.create_dataset('qpos', (0, 14), maxshape=(None, 14))
        _ = obs.create_dataset('qvel', (0, 14), maxshape=(None, 14))
        _ = obs.create_dataset('effort', (0, 14), maxshape=(None, 14))
        _ = root.create_dataset('action', (0, 14), maxshape=(None, 14))
        _ = root.create_dataset('base_action', (0, 2), maxshape=(None, 2))

        if args.use_audio:
            audio = obs.create_group('audio')

                audio_length_l = len(data_dict['/observations/audio/audio_l'][0])
                _ = audio.create_dataset('audio_l', (0, audio_length_l), maxshape=(None, audio_length_l), dtype='float32',
                                         chunks=(1, audio_length_l), compression='gzip', compression_opts=9)
            audio_length_r = len(data_dict['/observations/audio/audio_r'][0])
            _ = audio.create_dataset('audio_r', (0, audio_length_r), maxshape=(None, audio_length_r), dtype='float32',
                                        chunks=(1, audio_length_r), compression='gzip', compression_opts=9)

        # Function to resize and append data to the dataset
        def append_data_to_dataset(dataset_name, data):
            dataset = root[dataset_name]
            current_size = dataset.shape[0]
            new_size = current_size + len(data)
            dataset.resize(new_size, axis=0)
            dataset[current_size:new_size] = data

        # Loop to process data in chunks
        while actions:
            for _ in range(min(chunk_size, len(actions))):
                action = actions.pop(0)
                ts = timesteps.pop(0)

                data_dict['/observations/qpos'].append(ts.observation['qpos'])
                data_dict['/observations/qvel'].append(ts.observation['qvel'])
                data_dict['/observations/effort'].append(ts.observation['effort'])
                data_dict['/action'].append(action)
                data_dict['/base_action'].append(ts.observation['base_vel'])

                for cam_name in args.camera_names:
                    data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])
                    if args.use_depth_image:
                        data_dict[f'/observations/images_depth/{cam_name}'].append(ts.observation['images_depth'][cam_name])
                if args.use_zed_depth and not args.use_depth_image:
                    data_dict[f'/observations/images_depth/{args.camera_names[0]}'].append(ts.observation['images_depth'][args.camera_names[0]])

                if args.use_audio:
                    data_dict['/observations/audio/audio_l'].append(ts.observation["audio_data"]["audio_l"])
                    data_dict['/observations/audio/audio_r'].append(ts.observation["audio_data"]["audio_r"])

            # Convert lists to numpy arrays for saving
            for name, array in data_dict.items():
                if name.startswith('/observations/images_depth/'):
                    array = np.expand_dims(np.array(array), axis=-1)
                else:
                    array = np.array(array)

                if len(array) > 0:
                    append_data_to_dataset(name, array)
                data_dict[name] = []  # Reset list after appending

    print(f'Saving: {time.time() - t0:.1f} secs', dataset_path)

class RosOperator:
    def __init__(self, args):
        self.robot_base_deque = None
        self.puppet_arm_right_deque = None
        self.puppet_arm_left_deque = None
        self.master_arm_right_deque = None
        self.master_arm_left_deque = None
        self.img_front_deque = None
        self.img_right_deque = None
        self.img_left_deque = None

        self.touch_left_deque = None
        self.touch_right_deque = None

        self.audio_left_deque = None
        self.audio_right_deque = None

        self.img_front_depth_deque = None
        self.img_right_depth_deque = None
        self.img_left_depth_deque = None
        self.bridge = None
        self.args = args

        self.init()
        self.init_ros()

    def init(self):
        self.bridge = CvBridge()
        self.img_left_deque = deque()
        self.img_right_deque = deque()
        self.img_front_deque = deque()
        self.img_left_depth_deque = deque()
        self.img_right_depth_deque = deque()
        self.img_front_depth_deque = deque()
        self.touch_left_deque = deque()
        self.touch_right_deque = deque()
        self.master_arm_left_deque = deque()
        self.master_arm_right_deque = deque()
        self.puppet_arm_left_deque = deque()
        self.puppet_arm_right_deque = deque()
        self.robot_base_deque = deque()

    def get_frame(self):

        if len(self.img_left_deque) == 0 or len(self.img_right_deque) == 0 or len(self.img_front_deque) == 0 or \
                (self.args.use_depth_image and (len(self.img_left_depth_deque) == 0 or len(self.img_right_depth_deque) == 0 or len(self.img_front_depth_deque) == 0)) or \
                (self.args.use_touch and (len(self.touch_left_deque) == 0 or len(self.touch_right_deque) == 0)):
            return False
        if self.args.use_zed_depth and len(self.img_front_depth_deque) == 0:
            return False
        if self.args.use_depth_image:
            frame_time = min([self.img_left_deque[-1].header.stamp.to_sec(), self.img_right_deque[-1].header.stamp.to_sec(), self.img_front_deque[-1].header.stamp.to_sec(),
                              self.img_left_depth_deque[-1].header.stamp.to_sec(), self.img_right_depth_deque[-1].header.stamp.to_sec(), self.img_front_depth_deque[-1].header.stamp.to_sec()])
        elif self.args.use_zed_depth:
            frame_time = min([self.img_left_deque[-1].header.stamp.to_sec(), self.img_right_deque[-1].header.stamp.to_sec(), self.img_front_deque[-1].header.stamp.to_sec(),
                              self.img_front_depth_deque[-1].header.stamp.to_sec()])
        elif self.args.use_touch:
            frame_time = min([self.img_left_deque[-1].header.stamp.to_sec(), self.img_right_deque[-1].header.stamp.to_sec(), self.img_front_deque[-1].header.stamp.to_sec(),
                              self.touch_left_deque[-1].header.stamp.to_sec(), self.touch_right_deque[-1].header.stamp.to_sec()])
        else:
            frame_time = min([self.img_left_deque[-1].header.stamp.to_sec(), self.img_right_deque[-1].header.stamp.to_sec(), self.img_front_deque[-1].header.stamp.to_sec()])

        # tolerance = 1.0 / self.args.frame_rate
        # def is_within_tolerance(deque, frame_time):
        #     return abs(deque[-1].header.stamp.to_sec() - frame_time) <= tolerance


        if len(self.img_left_deque) == 0 or self.img_left_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.img_right_deque) == 0 or self.img_right_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.img_front_deque) == 0 or self.img_front_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.master_arm_left_deque) == 0 or self.master_arm_left_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.master_arm_right_deque) == 0 or self.master_arm_right_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.puppet_arm_left_deque) == 0 or self.puppet_arm_left_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.puppet_arm_right_deque) == 0 or self.puppet_arm_right_deque[-1].header.stamp.to_sec() < frame_time:
            return False

        if self.args.use_touch and (len(self.touch_left_deque) == 0 or self.touch_left_deque[-1].header.stamp.to_sec() < frame_time):
            return False
        if self.args.use_touch and (len(self.touch_right_deque) == 0 or self.touch_right_deque[-1].header.stamp.to_sec() < frame_time):
            return False

        if self.args.use_depth_image and (len(self.img_left_depth_deque) == 0 or self.img_left_depth_deque[-1].header.stamp.to_sec() < frame_time):
            return False
        if self.args.use_depth_image and (len(self.img_right_depth_deque) == 0 or self.img_right_depth_deque[-1].header.stamp.to_sec() < frame_time):
            return False
        if self.args.use_depth_image and (len(self.img_front_depth_deque) == 0 or self.img_front_depth_deque[-1].header.stamp.to_sec() < frame_time):
            return False
        if self.args.use_zed_depth and (len(self.img_front_depth_deque) == 0 or self.img_front_depth_deque[-1].header.stamp.to_sec() < frame_time):
            return False
        if self.args.use_robot_base and (len(self.robot_base_deque) == 0 or self.robot_base_deque[-1].header.stamp.to_sec() < frame_time):
            return False


        while self.img_left_deque[0].header.stamp.to_sec() < frame_time:
            self.img_left_deque.popleft()
        img_left = self.bridge.imgmsg_to_cv2(self.img_left_deque.popleft(), 'passthrough')
        img_left = cv2.rotate(img_left, cv2.ROTATE_180) ## rotate 180 degree
        # print("img_left:", img_left.shape)

        while self.img_right_deque[0].header.stamp.to_sec() < frame_time:
            self.img_right_deque.popleft()
        img_right = self.bridge.imgmsg_to_cv2(self.img_right_deque.popleft(), 'passthrough')

        """ while self.img_left_deque[0].header.stamp.to_sec() < frame_time:
            self.img_left_deque.popleft()
        img_left = self.bridge.imgmsg_to_cv2(self.img_left_deque.popleft(), 'passthrough')
        img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2RGB)
        # Resize the image to 480x640
        img_left = cv2.resize(img_left, (640, 480), interpolation=cv2.INTER_AREA)
        # Ensure the image has exactly 3 channels
        if img_left.shape[2] > 3:
            img_left = img_left[:, :, :3]

        while self.img_right_deque[0].header.stamp.to_sec() < frame_time:
            self.img_right_deque.popleft()
        img_right = self.bridge.imgmsg_to_cv2(self.img_right_deque.popleft(), 'passthrough')
        img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2RGB)
        img_right = cv2.resize(img_right, (640, 480), interpolation=cv2.INTER_AREA)
        if img_right.shape[2] > 3:
            img_right = img_right[:, :, :3] """


        while self.img_front_deque[0].header.stamp.to_sec() < frame_time:
            self.img_front_deque.popleft()
        img_front = self.bridge.imgmsg_to_cv2(self.img_front_deque.popleft(), 'passthrough')
        img_front = cv2.cvtColor(img_front, cv2.COLOR_BGR2RGB)
        img_front = cv2.resize(img_front, (640, 480), interpolation=cv2.INTER_AREA)
        if img_front.shape[2] > 3:
            img_front = img_front[:, :, :3]

        touch_left_img = None
        if self.args.use_touch:
            while self.touch_left_deque[0].header.stamp.to_sec() < frame_time:
                self.touch_left_deque.popleft()
            touch_left_img = self.bridge.imgmsg_to_cv2(self.touch_left_deque.popleft(), 'passthrough')
            #touch_left_img = cv2.cvtColor(touch_left_img, cv2.COLOR_BGR2RGB)
            touch_left_img = cv2.resize(touch_left_img, (640, 480), interpolation=cv2.INTER_AREA)
            if touch_left_img.shape[2] > 3:
                touch_left_img = touch_left_img[:, :, :3]
        
        touch_right_img = None
        if self.args.use_touch:
            while self.touch_right_deque[0].header.stamp.to_sec() < frame_time:
                self.touch_right_deque.popleft()
            touch_right_img = self.bridge.imgmsg_to_cv2(self.touch_right_deque.popleft(), 'passthrough')
            #touch_right_img = cv2.cvtColor(touch_right_img, cv2.COLOR_BGR2RGB)
            touch_right_img = cv2.resize(touch_right_img, (640, 480), interpolation=cv2.INTER_AREA)
            if touch_right_img.shape[2] > 3:
                touch_right_img = touch_right_img[:, :, :3]

        while self.master_arm_left_deque[0].header.stamp.to_sec() < frame_time:
            self.master_arm_left_deque.popleft()
        master_arm_left = self.master_arm_left_deque.popleft()

        while self.master_arm_right_deque[0].header.stamp.to_sec() < frame_time:
            self.master_arm_right_deque.popleft()
        master_arm_right = self.master_arm_right_deque.popleft()

        while self.puppet_arm_left_deque[0].header.stamp.to_sec() < frame_time:
            self.puppet_arm_left_deque.popleft()
        puppet_arm_left = self.puppet_arm_left_deque.popleft()

        while self.puppet_arm_right_deque[0].header.stamp.to_sec() < frame_time:
            self.puppet_arm_right_deque.popleft()
        puppet_arm_right = self.puppet_arm_right_deque.popleft()

        img_left_depth = None
        if self.args.use_depth_image:
            while self.img_left_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_left_depth_deque.popleft()
            img_left_depth = self.bridge.imgmsg_to_cv2(self.img_left_depth_deque.popleft(), 'passthrough')
            top, bottom, left, right = 40, 40, 0, 0
            img_left_depth = cv2.copyMakeBorder(img_left_depth, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

        img_right_depth = None
        if self.args.use_depth_image:
            while self.img_right_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_right_depth_deque.popleft()
            img_right_depth = self.bridge.imgmsg_to_cv2(self.img_right_depth_deque.popleft(), 'passthrough')
        top, bottom, left, right = 40, 40, 0, 0
        img_right_depth = cv2.copyMakeBorder(img_right_depth, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

        img_front_depth = None
        if self.args.use_depth_image or self.args.use_zed_depth:
            while self.img_front_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_front_depth_deque.popleft()
            img_front_depth = self.bridge.imgmsg_to_cv2(self.img_front_depth_deque.popleft(), '32FC1')
        top, bottom, left, right = 40, 40, 0, 0
        img_front_depth = cv2.copyMakeBorder(img_front_depth, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

        robot_base = None
        if self.args.use_robot_base:
            while self.robot_base_deque[0].header.stamp.to_sec() < frame_time:
                self.robot_base_deque.popleft()
            robot_base = self.robot_base_deque.popleft()


        audio_l = None
        audio_r = None
        if self.args.use_audio:
            # Process left channel
            audio_l = np.array(list(self.audio_left_deque)).flatten()
            sample_rate_l = self.audio_sample_rate[0]
            audio_length_l = int(self.args.audio_len * sample_rate_l)
            if len(audio_l) < audio_length_l:
                pad_len = audio_length_l - len(audio_l)
                audio_l = np.pad(audio_l, (0, pad_len), 'constant')

            # Process right channel
            audio_r = np.array(list(self.audio_right_deque)).flatten()
            sample_rate_r = self.audio_sample_rate[1]
            audio_length_r = int(self.args.audio_len * sample_rate_r)
            if len(audio_r) < audio_length_r:
                pad_len = audio_length_r - len(audio_r)
                audio_r = np.pad(audio_r, (0, pad_len), 'constant')

        return (img_front, img_left, img_right, touch_left_img, touch_right_img, img_front_depth, img_left_depth, img_right_depth,
                puppet_arm_left, puppet_arm_right, master_arm_left, master_arm_right, robot_base, audio_l, audio_r)

    def img_left_callback(self, msg):
        if len(self.img_left_deque) >= 2000:
            self.img_left_deque.popleft()
        self.img_left_deque.append(msg)

    def img_right_callback(self, msg):
        if len(self.img_right_deque) >= 2000:
            self.img_right_deque.popleft()
        self.img_right_deque.append(msg)

    def img_front_callback(self, msg):
        if len(self.img_front_deque) >= 2000:
            self.img_front_deque.popleft()
        self.img_front_deque.append(msg)

    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= 2000:
            self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)

    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= 2000:
            self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)

    def img_front_depth_callback(self, msg):
        if len(self.img_front_depth_deque) >= 2000:
            self.img_front_depth_deque.popleft()
        self.img_front_depth_deque.append(msg)

    def master_arm_left_callback(self, msg):
        if len(self.master_arm_left_deque) >= 2000:
            self.master_arm_left_deque.popleft()
        self.master_arm_left_deque.append(msg)

    def master_arm_right_callback(self, msg):
        if len(self.master_arm_right_deque) >= 2000:
            self.master_arm_right_deque.popleft()
        self.master_arm_right_deque.append(msg)

    def puppet_arm_left_callback(self, msg):
        if len(self.puppet_arm_left_deque) >= 2000:
            self.puppet_arm_left_deque.popleft()
        self.puppet_arm_left_deque.append(msg)

    def puppet_arm_right_callback(self, msg):
        if len(self.puppet_arm_right_deque) >= 2000:
            self.puppet_arm_right_deque.popleft()
        self.puppet_arm_right_deque.append(msg)

    def robot_base_callback(self, msg):
        if len(self.robot_base_deque) >= 2000:
            self.robot_base_deque.popleft()
        self.robot_base_deque.append(msg)

    def touch_left_callback(self, msg):
        if len(self.touch_left_deque) >= 2000:
            self.touch_left_deque.popleft()
        self.touch_left_deque.append(msg)

    def touch_right_callback(self, msg):
        if len(self.touch_right_deque) >= 2000:
            self.touch_right_deque.popleft()
        self.touch_right_deque.append(msg)
    
    def audio_left_callback(self, msg):
        self.audio_left_deque.extend(np.asarray(msg.data).reshape((-1,self.audio_num_channels[0])))

    def audio_right_callback(self, msg):
        self.audio_right_deque.extend(np.asarray(msg.data).reshape((-1,self.audio_num_channels[1])))
    
    def init_ros(self):
        rospy.init_node('record_episodes', anonymous=True)
        rospy.Subscriber(self.args.img_left_topic, Image, self.img_left_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.img_right_topic, Image, self.img_right_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.img_front_topic, Image, self.img_front_callback, queue_size=1000, tcp_nodelay=True)
        if self.args.use_touch:
            rospy.Subscriber(self.args.touch_left_topic, Image, self.touch_left_callback, queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.touch_right_topic, Image, self.touch_right_callback, queue_size=1000, tcp_nodelay=True)
        if self.args.use_audio:
            audio_info_left = rospy.wait_for_message(self.args.audio_left_topic+'_info', AudioInfo)
            audio_info_right = rospy.wait_for_message(self.args.audio_right_topic+'_info', AudioInfo)
            self.audio_sample_rate = (audio_info_left.sample_rate, audio_info_right.sample_rate)
            self.audio_num_channels = (audio_info_left.num_channels, audio_info_right.num_channels)
            rospy.Subscriber(self.args.audio_left_topic, AudioData, self.audio_left_callback, queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.audio_right_topic, AudioData, self.audio_right_callback, queue_size=1000, tcp_nodelay=True)
            self.audio_left_deque = deque(maxlen=int(self.args.audio_len*self.audio_sample_rate[0]))
            self.audio_right_deque = deque(maxlen=int(self.args.audio_len*self.audio_sample_rate[1]))


        if self.args.use_depth_image:
            rospy.Subscriber(self.args.img_left_depth_topic, Image, self.img_left_depth_callback, queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.img_right_depth_topic, Image, self.img_right_depth_callback, queue_size=1000, tcp_nodelay=True)
            rospy.Subscriber(self.args.img_front_depth_topic, Image, self.img_front_depth_callback, queue_size=1000, tcp_nodelay=True)
        elif self.args.use_zed_depth and  not self.args.use_depth_image:
            rospy.Subscriber(self.args.img_front_depth_topic, Image, self.img_front_depth_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.master_arm_left_topic, JointState, self.master_arm_left_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.master_arm_right_topic, JointState, self.master_arm_right_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.puppet_arm_left_topic, JointState, self.puppet_arm_left_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.puppet_arm_right_topic, JointState, self.puppet_arm_right_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.robot_base_topic, Odometry, self.robot_base_callback, queue_size=1000, tcp_nodelay=True)

    def process(self):
        timesteps = []
        actions = []
        # 图像数据
        #zed_image = np.random.randint(0, 255, size=(360, 640, 4), dtype=np.uint8)
        image_format = np.random.randint(0, 255, size=(480, 640, 3), dtype=np.uint8)
        image_dict = dict()
        #image_dict[self.args.camera_names[0]] = zed_image
        for cam_name in self.args.camera_names:
            image_dict[cam_name] = image_format
        count = 0
        
        # input_key = input("please input s:")
        # while input_key != 's' and not rospy.is_shutdown():
        #     input_key = input("please input s:")

        rate = rospy.Rate(self.args.frame_rate)
        print_flag = True
        while (count < self.args.max_timesteps + 1) and not rospy.is_shutdown():
            # 2 收集数据
            result = self.get_frame()
            if not result:
                if print_flag:
                    print("syn fail", time.time())
                    print_flag = False
                rate.sleep()
                continue
            print_flag = True
            count += 1
            (img_front, img_left, img_right, touch_left_img, touch_right_img, img_front_depth, img_left_depth, img_right_depth,
                puppet_arm_left, puppet_arm_right, master_arm_left, master_arm_right, robot_base, audio_l, audio_r) = result
            # 2.1 图像信息
            image_dict = dict()
            image_dict[self.args.camera_names[0]] = img_front
            image_dict[self.args.camera_names[1]] = img_left
            image_dict[self.args.camera_names[2]] = img_right
            if self.args.use_touch:
                image_dict[self.args.camera_names[3]] = touch_left_img
                image_dict[self.args.camera_names[4]] = touch_right_img
            
            if self.args.use_audio:
                audio_dict = dict()
                audio_dict["audio_l"] = audio_l
                audio_dict['audio_r'] = audio_r

            # 2.2 从臂的信息从臂的状态 机械臂示教模式时 会自动订阅
            obs = collections.OrderedDict()  # 有序的字典
            obs['images'] = image_dict
            if self.args.use_depth_image:
                image_dict_depth = dict()
                image_dict_depth[self.args.camera_names[0]] = img_front_depth
                image_dict_depth[self.args.camera_names[1]] = img_left_depth
                image_dict_depth[self.args.camera_names[2]] = img_right_depth
                obs['images_depth'] = image_dict_depth
            elif self.args.use_zed_depth and  not self.args.use_depth_image:
                image_dict_depth = dict()
                image_dict_depth[self.args.camera_names[0]] = img_front_depth
                obs['images_depth'] = image_dict_depth
            if self.args.use_audio:
                obs["audio_data"] = audio_dict
            obs['qpos'] = np.concatenate((np.array(puppet_arm_left.position), np.array(puppet_arm_right.position)), axis=0)
            obs['qvel'] = np.concatenate((np.array(puppet_arm_left.velocity), np.array(puppet_arm_right.velocity)), axis=0)
            obs['effort'] = np.concatenate((np.array(puppet_arm_left.effort), np.array(puppet_arm_right.effort)), axis=0)
            if self.args.use_robot_base:
                obs['base_vel'] = [robot_base.twist.twist.linear.x, robot_base.twist.twist.angular.z]
            else:
                obs['base_vel'] = [0.0, 0.0]

            # 第一帧 只包含first， fisrt只保存StepType.FIRST
            if count == 1:
                ts = dm_env.TimeStep(
                    step_type=dm_env.StepType.FIRST,
                    reward=None,
                    discount=None,
                    observation=obs)
                timesteps.append(ts)
                continue

            # 时间步
            ts = dm_env.TimeStep(
                step_type=dm_env.StepType.MID,
                reward=None,
                discount=None,
                observation=obs)

            # 主臂保存状态
            action = np.concatenate((np.array(master_arm_left.position), np.array(master_arm_right.position)), axis=0)
            actions.append(action)
            timesteps.append(ts)
            print("Frame data: ", count, " ", time.time())
            if rospy.is_shutdown():
                exit(-1)
            rate.sleep()

        print("len(timesteps): ", len(timesteps))
        print("len(actions)  : ", len(actions))
        return timesteps, actions



def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', action='store', type=str, help='Dataset_dir.',
                        default="/home/jianrenw/Documents/skilled/data", required=False)
    parser.add_argument('--use_zed_front', action='store', type=bool, help='Use Zed Cam as the front Camera.',
                        default=True, required=False)
    parser.add_argument('--use_zed_depth', action='store', type=bool, help='Use Zed Cam depth.',
                        default=False, required=False)
    parser.add_argument('--task_name', action='store', type=str, help='Task name.',
                        default="pouring_bimanual", required=False)
    parser.add_argument('--episode_idx', action='store', type=int, help='Episode index.',
                        default=0, required=False)
    parser.add_argument('--max_timesteps', action='store', type=int, help='Max_timesteps.',
                        default=200, required=False)

    parser.add_argument('--camera_names', action='store', type=str, help='camera_names',
                        default=['cam_high', 'cam_left_wrist', 'cam_right_wrist', 'touch_left_wrist', 'touch_right_wrist'], 
                        required=False)
    parser.add_argument('--img_front_topic', action='store', type=str, help='img_front_topic',
                        default='/zed/zed_node/rgb_raw/image_raw_color', required=False)
    parser.add_argument('--img_left_topic', action='store', type=str, help='img_left_topic',
                        default='/camera_l/color/image_raw', required=False)
    parser.add_argument('--img_right_topic', action='store', type=str, help='img_right_topic',
                        default='/camera_r/color/image_raw', required=False)
    
    parser.add_argument('--use_touch', action='store', type=bool, help='Use skilled touch sensor',
                        default=True, required=False)
    
    parser.add_argument('--touch_left_topic', action='store', type=str, help='topic of the left gelsight cam',
                        default='/gelsight_l/image_raw', required=False)
    parser.add_argument('--touch_right_topic', action='store', type=str, help='topic of the right gelsight cam',
                        default='/gelsight_r/image_raw', required=False)
    
    parser.add_argument('--use_audio', action='store', type=bool, help='Use skilled hear sensor',
                        default=True, required=False)
    parser.add_argument('--audio_len', action='store', type=float, help='the length of past audio to use',
                        default=2, required=False)
    
    parser.add_argument('--audio_left_topic', action='store', type=str, help='topic of the left contact mic',
                        default='/audio_l', required=False)
    parser.add_argument('--audio_right_topic', action='store', type=str, help='topic of the right contact mic',
                        default='/audio_r', required=False)
    

    parser.add_argument('--img_front_depth_topic', action='store', type=str, help='img_front_depth_topic',
                        default='/zed/zed_node/depth/depth_registered', required=False)
    parser.add_argument('--img_left_depth_topic', action='store', type=str, help='img_left_depth_topic',
                        default='/camera_l/depth/image_raw', required=False)
    parser.add_argument('--img_right_depth_topic', action='store', type=str, help='img_right_depth_topic',
                        default='/camera_r/depth/image_raw', required=False)
    
    parser.add_argument('--master_arm_left_topic', action='store', type=str, help='master_arm_left_topic',
                        default='/master/joint_left', required=False)
    parser.add_argument('--master_arm_right_topic', action='store', type=str, help='master_arm_right_topic',
                        default='/master/joint_right', required=False)
    parser.add_argument('--puppet_arm_left_topic', action='store', type=str, help='puppet_arm_left_topic',
                        default='/puppet/joint_left', required=False)
    parser.add_argument('--puppet_arm_right_topic', action='store', type=str, help='puppet_arm_right_topic',
                        default='/puppet/joint_right', required=False)
    parser.add_argument('--robot_base_topic', action='store', type=str, help='robot_base_topic',
                        default='/odom_raw', required=False)
    parser.add_argument('--use_robot_base', action='store', type=bool, help='use_robot_base',
                        default=False, required=False)
    parser.add_argument('--use_depth_image', action='store', type=bool, help='use_depth_image',
                        default=False, required=False)
    parser.add_argument('--frame_rate', action='store', type=int, help='frame_rate',
                        default=30, required=False)
    parser.add_argument('--use_keyboard_end', action='store', type=bool, help='use_keyboard_end',
                        default=False, required=False)
    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
    ros_operator = RosOperator(args)
    if args.use_keyboard_end:
        thread = threading.Thread(target=key_capture_thread)
        thread.start()
    timesteps, actions = ros_operator.process()
    dataset_dir = os.path.join(args.dataset_dir, args.task_name)
    if not os.path.exists(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, "episode_" + str(args.episode_idx))
    save_data(args, timesteps, actions, dataset_path)


if __name__ == '__main__':
    main()

# python scripts/record_data.py --dataset_dir ~/data --max_timesteps 500 --episode_idx 0
