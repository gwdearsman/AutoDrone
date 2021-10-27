# AutoDrone
Gavin Dearsman

The beginnings of a Project that involves connecting a drone with a Pixhawk flight controller with a raspberry pi 3B. I will attempt to conncet these with mission planner
and eventually openCV to create an autonomous flight vehicle.

Installed Packages:
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get install python-pip
  sudo apt-get install python-dev
  sudo apt-get install screen wxgtk libxml libxslt
  sudo pip install pyserial
  sudo pip install dronekit
  sudo pip install MAVProxy
  sudo pip install future
  sudo pip install opencv
  
  sudo nano /boot/config.txt
    add "dtoverlay=disable-bt" to configure serial port
    
to enter mavproxy:
  mavproxy.py --master=/dev/serial0 --baudrate 921600

