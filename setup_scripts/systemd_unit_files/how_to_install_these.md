copy files to /etc/systemd/system/

then 

sudo systemctl daemon-reload
sudo systemctl enable your_service_file.service
sudo systemctl start your_service_file.service
