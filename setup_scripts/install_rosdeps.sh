echo Run from RoverFlake2 root dir
rosdep update
rosdep install --from-paths src --ignore-src -r --skip-keys="serial moteus_msgs" -y --rosdistro humble