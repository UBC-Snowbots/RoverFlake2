# RoverFlake2
2023 - Present ROS2 repo for rover

### Setting up this repo on your computer
_Reccomended/Required: Ubuntu 24.04_

Use the [rovercli](https://github.com/UBC-Snowbots/rovercli) tool to set up this repo

SSH authentication is the easiest way to get authorized to push code.

To set up SSH keys on your computer, follow this tutorial: 
[Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) 

Then, set up the newly generated SSH in github with this tutorial: [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

_Obviously you are welcome to use HTTPS or another form of authentication (like GitHub Desktop) if you prefer_

Once the repo is set up, try to build!

From the root of RoverFlake2:
> `colcon build`
(it should automatically use --symlink-install as set in `colcon_defaults.yaml`)

If you get an error... panic, scream, and hurl insults at your computer. Then look at common issues and ask for help in the discord. 

### Unit Tests
We use Catch2 for CPP unit tests. To keep it simple, we don't integrate with colcon tests, we have a seperate CMakeLists.txt and Makefile at the root, to handle all unit tests.
To build and run tests: (the -j16 is just to compile faster by specifying how many cores to use)
> `make -j16 test` 
To add tests, open up the CMakeLists.txt and manually add paths to test files. Follow the comments within the file.
Catch2 is powerful, lightweight, and easy to use. It has a lot of built in macros to help with testing but even just the basics is enough to get by.

### COMMON ISSUES & TROUBLESHOOTING
arm_hardware_interface fails to build:
 
> Could not find a package configuration file provided by "serial" with any of the following names:

> serialConfig.cmake

> ...

**if src/external_pkgs/serial is empty, you need to update your git submodules:**

> 'git submodule init'
> 'git submodule update'

'serial' is a ros2 package, its also a git repository. Instead of just copying the code into our repository, git submodules makes it easier to manage different git repositories.

This error means CMake cannot find a package, specifically a ROS2 package. In the above error example CMake cannot find the serial package, which is an external package we use to communicate over USB connections.
If you get this error for another package, you may just need to install it:

> 'sudo apt install ros-$ROS_DISRTO-<package_name>'
 
