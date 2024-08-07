from utils import load_data


def main():

    DATA_DIR = '/home/jianrenw/Documents/skilled/data/test'
    train_dataloader, val_dataloader, stats, _ = load_data(DATA_DIR, 3, 0, False, True, False, 
                                                           ['cam_high', 'cam_left_wrist', 'cam_right_wrist', 'touch_left_wrist', 'touch_right_wrist'],
                                                           1, 1)
    for batch_idx, data in enumerate(train_dataloader):
        image_data, image_depth_data, qpos_data, action_data, action_is_pad, audio_data = data
        print(audio_data.shape)

if __name__ == '__main__':
    main()