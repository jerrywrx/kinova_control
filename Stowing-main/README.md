# Stowing

Stowing is a robotic project for inserting the object in a clustered bin with discontinuous space. 

## Installation

The code was tested on Ubuntu 20.04, with Python 3.7. Higher versions should be possible with some accuracy difference. 

```bash
pip install -r requirements.txt
```

## Usage

```python
```

## Citation


## Check multiple mp4 video
- Navigate to the folder containing the MP4 videos you want to view using `cd` command.
- Type mpv `*.mp4` and press Enter. This will open all the MP4 videos in the folder in separate windows.
- You can use the `f` key to toggle between full screen and windowed mode, and the `q` key to exit the player.

ffmpeg -i planned.mp4 -ss 00:00:00 -to 00:00:02 -c copy clip.mp4





```
python perception/scripts/auto_patch_make.py
```

In three terminals, run the following:
```
python dynamics/scripts/hyperparameter_insert.py
```

```
python dynamics/scripts/hyperparameter_sweep.py
```

```
python dynamics/scripts/hyperparameter_push.py
```