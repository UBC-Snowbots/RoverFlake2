sudo apt update
sudo apt upgrade -y


#for motor controllers
curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo -E bash -
sudo apt-get install -y libphidget22
sudo apt-get install -y libphidget22-dev



# for libcamera
sudo apt install -y libcamera-dev

# for gscam
sudo apt-get install -y gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev 
sudo apt install -y ros-humble-gscam 
