#coding=utf-8
import os
import numpy as np
import cv2
import h5py
import argparse
import matplotlib.pyplot as plt
import robobuf as rb
from tqdm import tqdm
import pickle as pkl

DT = 0.02
# JOINT_NAMES = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
STATE_NAMES = JOINT_NAMES + ["gripper"]
BASE_STATE_NAMES = ["linear_vel", "angular_vel"]

def load_hdf5(dataset_dir, dataset_name):
    dataset_path = os.path.join(dataset_dir, dataset_name + '.hdf5')
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        is_sim = root.attrs['sim']
        compressed = root.attrs.get('compress', False)
        qpos = root['/observations/qpos'][()]
        qvel = root['/observations/qvel'][()]
        if 'effort' in root.keys():
            effort = root['/observations/effort'][()]
        else:
            effort = None
        action = root['/action'][()]
        base_action = root['/base_action'][()]
        image_dict = dict()
        for cam_name in root[f'/observations/images/'].keys():
            image_dict[cam_name] = root[f'/observations/images/{cam_name}'][()]
        if compressed:
            compress_len = root['/compress_len'][()]
            
    if compressed:
        for cam_id, cam_name in enumerate(image_dict.keys()):
            # un-pad and uncompress
            padded_compressed_image_list = image_dict[cam_name]
            image_list = []
            for frame_id, padded_compressed_image in enumerate(padded_compressed_image_list): # [:1000] to save memory
                image_len = int(compress_len[cam_id, frame_id])
                compressed_image = padded_compressed_image
                image = cv2.imdecode(compressed_image, 1)
                image_list.append(image)
            image_dict[cam_name] = image_list

    return qpos, qvel, effort, action, base_action, image_dict

def dump_to_file(replay_buffer, filename):
        # Dump to file
        with open(filename, "wb") as f:
            pkl.dump(replay_buffer.to_traj_list(), f)

def main(args):
    dataset_dir = args['dataset_dir']
    # episode_idx = args['episode_idx']
    task_name   = args['task_name']
    all_trajs = []
    buf = rb.ReplayBuffer()
    for episode_idx in tqdm(range(25)):
        traj = []
        dataset_name = f'episode_{episode_idx}'
        qpos, qvel, effort, action, base_action, image_dict = load_hdf5(os.path.join(dataset_dir, task_name), dataset_name)
        traj_len = len(qpos)
        print(f"traj_len: {traj_len}")
        if traj_len > 600:
            traj_len = 600
        for idx in range(traj_len):
            obs = {}
            obs['state'] = np.append(qpos[idx], qvel[idx])
            # obs['state'] = qpos[idx]
            for key in image_dict:
                obs[key] = image_dict[key][idx]
            trans = (obs, action[idx], 0)
            traj.append(trans)
        all_trajs.append(traj)
    buf.append_traj_list(all_trajs)
    
    # all_trajs = buf.to_traj_list()
    print(f"save all trajs into pkl file: {dump_to_file(buf, 'food_sorting_full_state.pkl')}")
    print(f"len of buf baby: {len(buf)}")
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', action='store', type=str, help='Dataset dir.', required=True)
    parser.add_argument('--task_name', action='store', type=str, help='Task name.',
                        default="aloha_mobile_dummy", required=False)
    parser.add_argument('--episode_idx', action='store', type=int, help='Episode index.',default=0, required=False)
    
    main(vars(parser.parse_args()))
