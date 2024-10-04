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

> `git clone --recurse-submodules git@github.com:UBC-Snowbots/RoverFlake2.git`


Then cd in, and we can use setup scripts from here. 

> `bash setup_scripts/install_ros2.sh`
