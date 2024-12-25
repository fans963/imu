# imu
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", MODE="0666"' | sudo tee /etc/udev/rules.d/95-rmcs-imu.rules 
sudo udevadm control --reload-rules && sudo udevadm trigger 