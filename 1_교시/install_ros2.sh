#!/bin/bash
set -e

# 1. Fix Hash Sum mismatch (APT cache cleaning)
sudo rm -rf /var/lib/apt/lists/*
sudo apt clean
sudo apt update

# 2. Set Locale
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Add ROS 2 APT repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Install ROS 2 (Humble)
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

# 5. Setup rosdep
sudo apt install python3-rosdep -y
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
fi
sudo rosdep init
rosdep update

# 6. Configure environment variables (.bashrc)
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc; then
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
fi

echo "install pcl"

sudo apt install libpcl-dev pcl-tools -y