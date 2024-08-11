source ~/RoverFlake2/install/setup.bash

exec i3-msg 'workspace 1; exec ros2 launch rover_qt_gui arm_display.launch.py'
sleep 0.4
i3-msg -t command workspace 1
i3-msg -t command fullscreen
