from cv_bridge import CvBridge
import sys
import select
import threading
import cv2
import os
import h5py
import numpy as np
from tqdm import tqdm

def main():
    data_dir = '/home/jianrenw/Documents/skilled/cobot_magic/data/pouring/'
    dim = (500, 480, 640, 4)
    topic_names = ['images/cam_left_wrist', 'images/cam_right_wrist', 'images/cam_high']
    for i in tqdm(range(50)):
        file_path = os.path.join(data_dir, f'episode_{i}.hdf5')
        new_datasets = {}
        with h5py.File(file_path, 'r') as f:
            for cam in topic_names:
                dataset_name = f'observations/{cam}'
                cam_image = f[dataset_name][:]
                padded_img = pad_images(cam_image, dim)
                new_datasets[dataset_name] = padded_img

        with h5py.File(file_path, 'a') as f:
            for cam in topic_names:
                dataset_name = f'observations/{cam}'
                if dataset_name in f:
                    del f[f'observations/{cam}']
                f.create_dataset(dataset_name, data=new_datasets[dataset_name])
        print(f"padding for episode_{i} finished")

            
def pad_images(images, target_shape):
    _, height, width, channels = images.shape
    _, target_height, target_width, target_channels = target_shape

    # Calculate padding for height and width
    height_padding = (target_height - height)
    width_padding = (target_width - width)
    channel_padding = (target_channels - channels)

    top_padding = height_padding // 2
    bottom_padding = height_padding - top_padding
    left_padding = width_padding // 2
    right_padding = width_padding - left_padding

    # Pad height and width
    padded_images = np.pad(images, 
                           ((0, 0), (top_padding, bottom_padding), (left_padding, right_padding), (0, channel_padding)), 
                           mode='constant', constant_values=0)
    return padded_images

if __name__ == '__main__':
    main()