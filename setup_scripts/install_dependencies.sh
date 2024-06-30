sudo apt update
sudo apt upgrade


#for motor controllers
curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo -E bash -
sudo apt-get install -y libphidget22
sudo apt-get install -y libphidget22-dev

