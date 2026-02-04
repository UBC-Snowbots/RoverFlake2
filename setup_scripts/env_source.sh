# this is for daemons and startup stuff. Assume no enviroment before this.
source /opt/ros/humble/setup.bash
source /home/${USER}/RoverFlake2/install/setup.bash
source /home/${USER}/RoverFlake2/network_stuff/cyclonedds_env.sh


# gui apps
export DISPLAY=:0
export XAUTHORITY="/run/user/1001/gdm"