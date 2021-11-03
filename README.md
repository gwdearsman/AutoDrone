# AutoDrone
Gavin Dearsman

The beginnings of a Project that involves connecting a drone with a Pixhawk flight controller with a raspberry pi 3B. I will attempt to conncet these with mission planner
and eventually openCV to create an autonomous flight vehicle.

Installed Packages:
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get install python3-pip
  sudo apt-get install python3-dev
  sudo apt-get install screen wxgtk libxml libxslt
  sudo pip3 install pyserial
  sudo pip3 install dronekit
  sudo pip3 install MAVProxy
  sudo pip3 install future
  sudo apt install fswebcam
  
For opencv:
https://www.pyimagesearch.com/2018/09/26/install-opencv-4-on-your-raspberry-pi/
  raspi-config expand file system
  videolibraries
  sudo apt-get install build-essential cmake unzip pkg-config
  sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
  sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
  sudo apt-get install libxvidcore-dev libx264-dev
  sudo apt-get install libgtk-3-dev
  sudo apt-get install libcanberra-gtk*
  sudo apt-get install libatlas-base-dev gfortran
 
  $ cd ~
  wget -O opencv.zip https://github.com/opencv/opencv/archive/4.0.0.zip
  wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.0.0.zip
  unzip opencv.zip
  unzip opencv_contrib.zip
  renaming:
  mv opencv-4.0.0 opencv
  mv opencv_contrib-4.0.0 opencv_contrib
  sudo pip3 install virtualenv virtualenvwrapper
  sudo rm -rf ~/get-pip.py ~/.cache/pip
  
  echo -e "\n# virtualenv and virtualenvwrapper" >> ~/.profile
  echo "export WORKON_HOME=$HOME/.virtualenvs" >> ~/.profile
  echo "export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3" >> ~/.profile
  echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.profile
  source ~/.profile
  
Creating virtual environment:
  mkvirtualenv cv -p python3
  workon cv
  pip install numpy
  cd ~/opencv
  mkdir build
  cd build
  cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D ENABLE_NEON=ON \
    -D ENABLE_VFPV3=ON \
    -D BUILD_TESTS=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF ..
  sudo nano /etc/dphys-swapfile
    change CONF_SWAPSIZE=2048
  sudo /etc/init.d/dphys-swapfile stop
  sudo /etc/init.d/dphys-swapfile start
  
Compile Opencv:
  make
  sudo make install
  sudo ldconfig
  sudo nano /etc/dphys-swapfile
    change CONF_SWAPSIZE=100
  sudo /etc/init.d/dphys-swapfile stop
  sudo /etc/init.d/dphys-swapfile start
  
LINK OPENCV with python:(tab completion)
  cd ~/.virtualenvs/cv/lib/python3.5/site-packages/
  ln -s /usr/local/python/cv2/python-3.5/cv2.cpython-35m-arm-linux-gnueabihf.so cv2.so
  cd ~
  
  
   
  
  
  
  sudo pip install opencv
  sudo apt install fswebcam
  
  
sudo apt install libatlas-base-dev -y
sudo apt install libjasper-dev -y
sudo apt install libqtgui4 -y
sudo apt install python3-pyqt5 -y
sudo apt install libqt4-test -y
sudo apt install libhdf5-dev libhdf5-serial-dev -y
sudo pip3 install opencv-contrib-python==4.1.0.25
  
  sudo nano /boot/config.txt
    add "dtoverlay=disable-bt" to configure serial port
    
to enter mavproxy:
  mavproxy.py --master=/dev/serial0 --baudrate 921600

