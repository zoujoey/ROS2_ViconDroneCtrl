session="my580"

tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window 'BG'
tmux send-keys -t $session:$window "$HOME/Downloads/QGroundControl.AppImage" C-m

tmux split-window -v
tmux send-keys -t $session:$window 'source install/local_setup.bash; ros2 launch vicon_position_bridge graphing_launch.py' C-m

window=1
tmux new-window -t $session:$window -n 'Setup'
tmux send-keys -t $session:$window "echo 'run in below minicom: 1. 2xenter, 2. lep'" C-m
tmux send-keys -t $session:$window 'sudo minicom -D /dev/ttyACM0' C-m

tmux split-window -v

tmux send-keys -t $session:$window 'source install/local_setup.bash' C-m 
tmux send-keys -t $session:$window 'ros2 run decawave_pose_tracker listener_pose_xy' C-m

tmux split-window -v
tmux send-keys -t $session:$window 'ssh rob498@10.42.0.135' C-m


window=2 
tmux new-window -t $session:$window -n 'Demo'
tmux send-keys -t $session:$window 'source install/local_setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch drone_keyboard_controller control_launch.py'

tmux select-window -t 1

tmux attach-session -t $session
