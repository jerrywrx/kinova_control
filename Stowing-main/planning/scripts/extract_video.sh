#!/bin/bash
for dir in $(ls -d $HOME/Projects/Stowing/dump/control/001/control_close_max*); do
    base=$(basename "$dir")
    cp "$dir/planned.mp4" "$HOME/Projects/Stowing/dump/control/001/planned_videos/${base}.mp4"
done
