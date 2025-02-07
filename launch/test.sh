#!/bin/bash

# Read the number of windows to create
echo "Enter the number of windows:"
read N

# Start a new tmux session named 'my_session' (or any name you'd like)
tmux new-session -d -s my_session

for ((i=1; i<N; i++))
do
    # Split horizontally
    tmux split-window -h -t my_session:0
    # Split vertically
    tmux split-window -v -t my_session:0
  
done

# Attach to the tmux session
tmux attach-session -t my_session