# RoverFlake2
2023 - Present ROS2 repo for rover
RoverFlake1 is old.

### Setting up this repo on your computer
_Reccomended/Required: Ubuntu 22.04_

SSH authentication is the easiest way to get authorized to push code.

To set up SSH keys on your computer, follow this tutorial: 
[Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) 

Then, set up the newly generated SSH in github with this tutorial: [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

Now, you can clone the repo, with SSH, **NOT HTTPS**:
> `#Install into your home directory`
> 
> `git clone --recurse-submodules git@github.com:UBC-Snowbots/RoverFlake2.git`

_Obviously you are welcome to use HTTPS or another form of authentication (like GitHub Desktop) if you prefer_

Then cd in, and we can use setup scripts from here. 

Before using setup scripts, you need to set your `ROVERFLAKE_ROOT` enviroment variable. Its best to throw this in your .bashrc file so that it sets permanently. 
One liner to do that:
> `echo "export ROVERFLAKE_ROOT=<path_to_roverflake>" >> ~/.bashrc`

If the repo is in your home directory, youd do:
> `echo "export ROVERFLAKE_ROOT=/home/<your_user>/Roverflake2" >> ~/.bashrc`

> `bash setup_scripts/setup_everything_common.sh`

This script will install ros2, as well as other common dependencies. 

After that script finishes, try to build!
From the root of RoverFlake2:
> `colcon build`
(it should automatically use --symlink-install as set in `colcon_defaults.yaml`)

If you get an error... panic, scream, and hurl insults at your computer. Then look at common issues and ask for help in the discord. 

### COMMON ISSUES & TROUBLESHOOTING
arm_hardware_interface fails to build:
 
> Could not find a package configuration file provided by "serial" with any of the following names:

> serialConfig.cmake

> ...

**if src/external_pkgs/serial is empty, you need to update your git submodules:**

> 'git submodule init'
> 'git submodule update'

_This is only needed if you did not clone recursivley._

'serial' is a ros2 package, its also a git repository. Instead of just copying the code into our repository, git submodules makes it easier to manage different git repositories.

This error means CMake cannot find a package, specifically a ROS2 package. In the above error example CMake cannot find the serial package, which is an external package we use to communicate over USB connections.
If you get this error for another package, you may just need to install it:

> 'sudo apt install ros-humble-<package_name>'
 
