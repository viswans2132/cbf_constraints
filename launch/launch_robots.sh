#!/bin/bash

pairs=3
# Check if at least one argument is provided
if [ $# -eq 0 ]; then
    echo "Number of robot pairs not specified. Proceeding with the default number of robot pairs: $pairs"
else
	pairs=$1
	# Check if the argument is an integer
	if ! [[ "$pairs" =~ ^[0-9]+$ ]]; then
		pairs=3
	    echo "Invalid argument. Proceeding with the default number of robot pairs: $pairs"
	elif [ $pairs -lt 3 ]; then
		pairs=3
		echo "Number of robot pairs less than 3. Defaulting the value to: $pairs"
	else
		echo "Number of robot pairs: $pairs"
	fi

	# Initialize variables
	n=$((pairs * 2))
	s=2

	# While loop to find the appropriate value of s
	while [ $(echo "$s^2 < $n" | bc) -eq 1 ]; do
	    s=$((s + 1))
	done

	# Initialize arrays
	X=()
	Y=()
	m=$((s % 2))



	# Generate ln array and calculate X and Y
	for ((i=0; i<n; i++)); do
	    ln=$i
	    X[$i]=$(echo "$ln % $s" | bc)
	done

	max_X=$(echo "${X[@]}" | tr ' ' '\n' | sort -nr | head -n1)
	min_X=$(echo "${X[@]}" | tr ' ' '\n' | sort -n | head -n1)
	for ((i=0; i<n; i++)); do
	    X[$i]=$(echo "${X[$i]} - ($max_X + $min_X) / 2" | bc -l)
	done

	for ((i=0; i<n; i++)); do
	    ln=$i
	    Y[$i]=$(echo "$ln / $s" | bc)
	done

	max_Y=$(echo "${Y[@]}" | tr ' ' '\n' | sort -nr | head -n1)
	min_Y=$(echo "${Y[@]}" | tr ' ' '\n' | sort -n | head -n1)
	for ((i=0; i<n; i++)); do
	    Y[$i]=$(echo "${Y[$i]} - ($max_Y + $min_Y) / 2" | bc -l)
	done

	# # Print the results
	# echo "X: ${X[@]}"
	# echo "Y: ${Y[@]}"

	# Start a new tmux session
	tmux new-session -d -s gazebo_spawns
 
	# tmux send-keys 'roslaunch cbf_constraints gazebo_world.launch' C-m

	# Create another new window and run the second ROS launch file
	# tmux split-window -v
	X="${X[0]}"
	ns1="dcf1"
	z1="0.2"
	tmux send-keys "sleep 2; roslaunch cbf_constraints spawn_uav.launch namespace:=$ns1 init_x:=$X init_y:=$X init_z:=$z1" C-m

	# Select the first pane
	tmux select-pane -t 1

	# Attach to the tmux session
	tmux attach-session -t gazebo_spawns



	
fi
