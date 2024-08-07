import h5py
import matplotlib.pyplot as plt
import numpy as np
import cv2
import librosa
from matplotlib.animation import FuncAnimation

def print_hdf5_structure(file_path):
    with h5py.File(file_path, 'r') as f:
        def print_attrs(name, obj):
            print(f"{name}:")
            for key, val in obj.attrs.items():
                print(f"  {key}: {val}")
            if isinstance(obj, h5py.Dataset):
                print(f"  Shape: {obj.shape}, Type: {obj.dtype}")
                if len(obj.shape) == 4 and obj.shape[-1] == 4:
                    print(f"  Note: Dataset has 4 channels.")
                elif len(obj.shape) == 4 and obj.shape[-1] == 3:
                    print(f"  Note: Dataset has 3 channels.")

        f.visititems(print_attrs)

def inspect_hdf5_data(file_path, dataset_name, start_frame=0, end_frame=10):
    with h5py.File(file_path, 'r') as f:
        data = f[dataset_name][:]
        print(f"Dataset {dataset_name} shape: {data.shape}, dtype: {data.dtype}")

        # Ensure the frame range is within the dataset bounds
        end_frame = min(end_frame, data.shape[0])

        # Loop through the specified range of frames
        for i in range(start_frame, end_frame):
            frame = data[i]
            plt.imshow(frame.astype(np.uint8))
            plt.title(f"Frame {i} in {dataset_name}")
            plt.show()

            # Wait for a key press to move to the next frame (optional)
            input("Press Enter to continue to the next frame...")

def play_hdf5_video(file_path, dataset_name, fps=30, depth=False):
    with h5py.File(file_path, 'r') as f:
        data = f[dataset_name][:]
        print(f"Dataset {dataset_name} shape: {data.shape}, dtype: {data.dtype}")
        
        # Ensure the data has the correct shape (n, height, width, channels)
        if len(data.shape) == 4:
            window_name = "HDF5 Video Playback"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            
            for i in range(data.shape[0]):
                frame = data[i]

                # Convert depth data to a displayable format
                if depth:
                    frame = (frame / np.max(frame) * 255).astype(np.uint8)
                    frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
                else:
                    # Handle single-channel or multi-channel data
                    if frame.shape[2] == 1:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    elif frame.shape[2] == 3:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                cv2.imshow(window_name, frame)
                
                # Add delay based on FPS
                key = cv2.waitKey(int(1000 / fps))
                
                # If 'q' is pressed, break from the loop
                if key == ord('q'):
                    break

            cv2.destroyAllWindows()
        else:
            print("The dataset does not contain image data in the expected format.")

def play_hdf5_last_channel(file_path, dataset_name, depth=False):
    with h5py.File(file_path, 'r') as f:
        data = f[dataset_name][:]
        print(f"Dataset {dataset_name} shape: {data.shape}, dtype: {data.dtype}")
        type = np.float32 if depth else np.uint8
        # Ensure the data has the correct shape (n, height, width, channels)
        if len(data.shape) == 4:
            window_name = "HDF5 Video Playback"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            
            for i in range(data.shape[0]):
                frame = data[i]
                # Extract the last channel
                last_channel = frame[:, :, -2]
                cv2.imshow(window_name, last_channel.astype(type))
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
            cv2.destroyAllWindows()
        else:
            print("The dataset does not contain image data in the expected format.")

def inspect_hdf5_obs(file_path, type = 'action'):
    with h5py.File(file_path, 'r') as f:
        actions = f[type][7:]
        key = ""
        i = 0
        while i < len(actions) and not key == "q":
            key = input("press c to continue; q to quit")
            data = actions[i,7:]
            print(data)
            i += 1

def inspect_audio_freq_plot(file_path, dataset_name, sr=16000, interval=500):
    with h5py.File(file_path, 'r') as f:
        audio_data = f[dataset_name][:]
        print(f"Dataset {dataset_name} shape: {audio_data.shape}, dtype: {audio_data.dtype}")

        # Ensure the dataset has the correct shape
        if len(audio_data.shape) == 2:
            fig, ax = plt.subplots(figsize=(10, 4))
            ani = FuncAnimation(fig, plot_audio_spectrogram, frames=audio_data.shape[0],
                                fargs=(audio_data, sr, ax), interval=interval)
            plt.show()
        else:
            print("The dataset does not contain audio data in the expected format.")

def plot_audio_spectrogram(timestep, audio_data, sr, ax):
    ax.clear()
    audio = audio_data[timestep]
    
    # Compute the spectrogram
    S = librosa.stft(audio)
    S_db = librosa.amplitude_to_db(np.abs(S), ref=np.max)
    
    librosa.display.specshow(S_db, sr=sr, x_axis='time', y_axis='log', ax=ax)
    ax.set(title=f'Spectrogram (Timestep {timestep})')
    plt.tight_layout()

def main():
    path = '/home/jianrenw/Documents/skilled/data/pouring_bimanual/episode_0.hdf5'
    print_hdf5_structure(path)
    #inspect_audio_freq_plot(path, 'observations/audio/audio_r', interval=100)
    #inspect_hdf5_obs(path)
    play_hdf5_video(path, 'observations/images/cam_high', depth = False)
    #play_hdf5_last_channel(path, 'observations/images/cam_high', depth = False)
    #play_hdf5_video(path, 'observations/images/touch_right_wrist', depth = False)
    #play_hdf5_video(path, 'observations/images_depth/cam_high', depth = True)

if __name__ == '__main__':
    main()