import os
import re

def main():
    # Define the path to the directory
    directory = 'target_shapes/ep_000'

    files = os.listdir(directory)
    indices = sorted(set(int(re.findall(r'\d+', f)[0]) for f in files if re.match(r'\d+(_cam_1\.png|_state\.npz|\.h5)', f)))
    for old_index, new_index in zip(indices, range(len(indices))):
        if old_index != new_index:
            for ext in ['.h5', '_rgb_cam_1.png', '_state.npz']:
                old_filename = f"{old_index:03d}{ext}"
                new_filename = f"{new_index:03d}{ext}"
                if old_filename in files:
                    os.rename(os.path.join(directory, old_filename), os.path.join(directory, new_filename))

if __name__ == "__main__":
    main()
