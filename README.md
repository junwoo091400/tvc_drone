# TVC Drone
ROS workspace containing the packages and tools for the drone TVC prototype

## Description

### Packages
- **drone_gnc**: Contains the GNC algorithms to control the drone. This is the only package that needs to be copied on the Raspberry Pi for real tests.
- **drone_utils**: Provides many utilities, as well as the launch files to run the code.
- **real_time_simulator**: Real-time rocket simulator and GUI. See original repo for more information.
- **drone_simulator_interface**: Interface to use the real time simulator with the drone. Transforms drone topics to rocket topics and vice versa.
- **rqt_ez_publisher**: External tool used in the GUI.

Each package contains its own readme file with more information.

## Getting Started

### Dependencies installation

* Install ROS Melodic (Ubuntu 18.04 required)
  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt install curl
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo apt update
  sudo apt install ros-melodic-desktop-full
  echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
  sudo rosdep init
  rosdep update
  ```

* Install Eigen
  ```
  cd ~ && git clone --single-branch --branch 3.4-rc1 https://gitlab.com/libeigen/eigen
  cd eigen && mkdir build && cd build
  cmake ..
  sudo make install
  ```

* Install Osqp
  ```
  cd ~ && git clone --recursive https://github.com/oxfordcontrol/osqp
  cd osqp && mkdir build && cd build
  cmake -G "Unix Makefiles" ..
  sudo cmake --build . --target install
  ```

* Install Osqp-Eigen
  ```
  cd ~ && git clone https://github.com/robotology/osqp-eigen.git
  cd osqp-eigen && mkdir build && cd build
  cmake ..
  make
  sudo make install
  export OsqpEigen_DIR=/usr/local/include/OsqpEigen
  ```

* Install the latest cmake version and select it automatically in the path
  ```
  sudo snap install cmake --classic
  echo "export PATH=/snap/bin:$PATH" >> ~/.bashrc
  ```

* Install VRPN (for optitrack)
  ```
  sudo apt-get install ros-melodic-vrpn-client-ros
  ```

* Install MAVROS (for pixhawk)
  ```
  sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
  sudo bash ./install_geographiclib_datasets.sh
  ```

* Install python dependencies
  ```
  sudo apt install python3-pip
  pip3 install rospkg matplotlib pyqtgraph scipy dataclasses pycryptodomex gnupg seaborn
  sudo apt install python-pip
  pip install pyqtgraph
  ```

### Setup and first build

* Clone the content of the repository into ~/drone_ws/src
  ```
  cd ~ && mkdir drone_ws && cd drone_ws
  git clone https://github.com/EPFLRocketTeam/tvc_drone.git src
  ```
* Rviz config issue: The config of the rviz plugin in rqt will not load properly because it only stores the absolute path in the .perspective file. This is fixed by writing your absolute ROS workspace path in _drone_simulator_interface/drone_GUI.perspective_ at the line:
    ```
    "repr": "u'*your ROS workspace absolute path*/src/drone_simulator_interface/GUI/rocket_config.rviz'"
    ```

* Build the project
  ```
  source ~/.bashrc
  cd ~/drone_ws
  catkin_make
  ```

* Setup automatic sourcing of the project for new shell instances
  ```
  echo "source ~/drone_ws/devel/setup.bash" >> ~/.bashrc
  ```

### Raspberry Pi setup
* Connect to the same Wi-Fi network as the Raspberry Pi
* Setup passwordless login
  ```
  ssh-keygen -t rsa
  cat .ssh/id_rsa.pub | ssh drone@ert.local 'cat >> .ssh/authorized_keys'
  ```

* Add the Raspberry Pi to the known hosts
  ```
  ssh-keyscan -H ert.local >> ~/.ssh/known_hosts
  ```


## Launching the code

* Build the project:
  ```
  cd ~/drone_ws
  catkin_make
  ```

* Run the simulation:
    ```
    roslaunch drone_utils simu_drone.launch
    ```
    Start the simulation by publishing an empty string on `/commands/data`, by pressing the "Publish" button.
    You can then control the target apogee using the sliders.
    
    To enable the guidance algorithm, run instead:
    ```
    roslaunch drone_utils simu_drone.launch use_guidance:=true
    ```

* Run remotely on the Raspberry Pi over Wi-Fi:
    ```
    roscd drone_utils/bash_scripts
    ./remote_drone_launch.sh
    ```


## Authors

Raphaël Linsen (raphael.linsen@epfl.ch)