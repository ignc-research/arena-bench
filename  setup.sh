#!/bin/bash

# setup ROS-noetic

echo ""
echo "[Note] Target OS version  >>> Ubuntu 20.04.x (Focal Fossa) or Linux Mint 21.x"
echo "[Note] Target ROS version >>> ROS Noetic Ninjemys"
echo "[Note] Catkin workspace   >>> $HOME/arena_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target OS, ROS version and name of catkin workspace]"

# Install Python packages (preferably in your virtual environment):
if [ -n "`$SHELL -c 'echo $ZSH_VERSION'`" ]; then
    # assume Zsh
    CURSHELL=zsh
    echo "Currently using zsh."
elif [ -n "`$SHELL -c 'echo $BASH_VERSION'`" ]; then
    # assume Bash
    CURSHELL=bash
    echo "Currently using bash."
else
    # assume something else
    echo "Currently only Bash and ZSH are supported for an automatic install. Please refer to the manual installation if you use any other shell."
fi

case $(lsb_release -sc) in
  focal)
    ROS_NAME_VERSION=noetic
    ;;

  bionic)
    ROS_NAME_VERSION=melodic
    ;;

  *)
    echo "Currently only Ubuntu Bionic Beaver and Focal Fossa are supported for an automatic install. Please refer to the manual installation if you use any Linux release or version."
    exit 1
    ;;
esac


name_os_version=${name_os_version:="focal"}
name_ros_version=${name_ros_version:="noetic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

echo "[Update the package lists]"
sudo apt update -y

sudo apt-get update && sudo apt-get upgrade

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt install -y chrony ntpdate curl build-essential
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo "[Update the package lists]"
sudo apt update -y

echo "[Install ros-desktop-full version of Noetic"
sudo apt install -y ros-$name_ros_version-desktop-full

echo "[Install RQT & Gazebo]"
sudo apt install -y ros-$name_ros_version-rqt-* ros-$name_ros_version-gazebo-*

echo "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git

echo "[Install rosdep and Update]"
sudo apt install python3-rosdep

echo "[Initialize rosdep and Update]"
sudo sh -c "rosdep init"
rosdep update

echo "[Set the ROS evironment]"
sh -c "echo \"source /opt/ros/$name_ros_version/setup.${CURSHELL}\" >> ~/.${CURSHELL}rc"
sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.${CURSHELL}rc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.${CURSHELL}rc"

source $HOME/.${CURSHELL}rc



# install required dependencies + virtual env
sudo apt-get update && sudo apt-get install -y \
libopencv-dev \
liblua5.2-dev \
screen \
python3-rosdep \
python3-rosinstall \
python3-rosinstall-generator \
build-essential \
python3-rospkg-modules \
ros-noetic-navigation \
ros-noetic-teb-local-planner \
ros-noetic-mpc-local-planner \
libarmadillo-dev \
ros-noetic-nlopt \
ros-noetic-turtlebot3-description \
ros-noetic-turtlebot3-navigation \
ros-noetic-lms1xx \
ros-noetic-velodyne-description 
sudo apt install -y python3-pip
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv virtualenvwrapper
cd $HOME && mkdir python_env   # create a venv folder in your home directory
echo "export WORKON_HOME=$HOME/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.${CURSHELL}rc
source ~/.${CURSHELL}rc

source /opt/ros/${ROS_NAME_VERSION}/setup.${CURSHELL}
source ~/catkin_ws/devel/setup.${CURSHELL}
source /usr/local/bin/virtualenvwrapper.sh
workon rosnav

pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip3 install pyyaml catkin_pkg netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml numpy scikit-image Pillow rospkg tensorflow
pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed \
pip3 install pyyaml catkin_pkg netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml 
pip install PyQt5 --upgrade


sudo apt-get -y update && apt-get install -y \
software-properties-common \
wget \
curl \
apt-utils\
gnutls-bin \
vim \
git \
original-awk \
python3-pip \
screen \
libopencv-dev \
liblua5.2-dev \
&& add-apt-repository ppa:rock-core/qt4 \
&& apt-get install -y libqtcore4


sudo apt-get -y update && apt-get install -y \
python3-rosdep \
python3-rosinstall \
python3-rosinstall-generator \
build-essential \
python3-rospkg-modules \
ros-noetic-navigation \
ros-noetic-teb-local-planner \
ros-noetic-mpc-local-planner \
libarmadillo-dev \
ros-noetic-nlopt \
ros-noetic-turtlebot3-description \
ros-noetic-turtlebot3-navigation \
ros-noetic-lms1xx \
ros-noetic-velodyne-description \
python3-catkin-pkg-modules \
python3-rospkg-modules \
python3-empy \
python3-setuptools \
libarmadillo-dev \
ros-noetic-pcl-conversions\
ros-noetic-pcl-ros\
ros-noetic-desktop-full


## 4.1. Install base arena-rosnav-3d
cd $HOME 
mkdir -p catkin_ws/src && cd catkin_ws/src 
git clone https://github.com/ignc-research/arena-bench -b main 
cd arena-rosnav-3D
rosws update 
cd ../.. 
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 
source ~/.${CURSHELL}r
echo $'\n \
source $HOME/catkin_ws/devel/setup.sh \n\
export PYTHONPATH=$HOME/catkin_ws/src/arena-rosnav-3D:${PYTHONPATH}' >> ~/.${CURSHELL}rc 

## 4.2. Include the actor-collision plugin
cd $HOME
git clone https://github.com/eliastreis/ActorCollisionsPlugin.git
cd ActorCollisionsPlugin
mkdir build
cd build
cmake ..
make
echo
"export GAZEBO_PLUGIN_PATH=$HOME/ActorCollisionsPlugin/build "
>> ~/.${CURSHELL}rc 
source ~/.${CURSHELL}rc

## 4.3. Install Pedsim
sudo apt install python3-rosdep python3-rospkg
cd ~/catkin_ws/src/arena-rosnav-3D
rosws update
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
cd ~/catkin_ws/src/forks/pedsim_ros
git submodule update --init --recursive 
cd ../../.. && catkin_make --only-pkg-with-deps spencer_tracking_rviz_plugin 
catkin_make -DCATKIN_WHITELIST_PACKAGES="" 