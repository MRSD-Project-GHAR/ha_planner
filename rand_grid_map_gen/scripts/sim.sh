#!/usr/bin/env bash

cd ~/ghar_ws

# Create a tmux session
session_name="planner_$(date +%s)"
tmux new-session -d -s $session_name 

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 2    # select the first (1) pane
tmux splitw -h -p 50 # split it into two halves

tmux select-pane -t 0
tmux send-keys "roslaunch mbf_rrts_planner planner_sim.launch" Enter

tmux select-pane -t 1
tmux send-keys "sleep 1 && roslaunch rand_grid_map_gen filter_robot_a.launch" Enter

tmux select-pane -t 2
tmux send-keys "sleep 2 && roslaunch rand_grid_map_gen random_map_rviz.launch" Enter

tmux select-pane -t 3
tmux send-keys "sleep 3 && roslaunch rand_grid_map_gen filter_robot_b.launch" Enter


tmux -2 attach-session -t $session_name -c ~/ghar_ws