# Raspberry Pi 5 setup

Install rpi-imager, and install Raspberry OS Lite (64 Bit). Use hostname `trajectory-estimator` and username `project` for consistency.

```bash
sudo apt install rpi-imager
rpi-imager
# 1. Enable SSH
# 2. Setup local Wi-Fi network
# 3. Setup timezone Europe/Helsinki
```

## Initial setup

```bash
ssh project@trajectory-estimator.local -o PreferredAuthentications=password
sudo apt update
sudo apt upgrade
sudo apt full-upgrade

wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
bash Miniconda3-latest-Linux-aarch64.sh
rm Miniconda3-latest-Linux-aarch64.sh
```

## Install ROS Rolling

ROS Rolling cannot seemingly be installed on the Raspberry Pi 5 through any supported means (or OS). Instructions for building ROS are modified from [https://lkseng.github.io/posts/2024/01/07/ros-2-humble-on-raspberrypi-5.html](https://lkseng.github.io/posts/2024/01/07/ros-2-humble-on-raspberrypi-5.html)

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures

mkdir -p ~/ros2_rolling/src
cd ~/ros2_rolling
vcs import --input https://raw.githubusercontent.com/ros2/ros2/rolling/ros2.repos src

sudo apt update && sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" --os=ubuntu:jammy --rosdistro=rolling
colcon build --symlink-install # ~2h 30min

git clone https://github.com/ros-drivers/nmea_msgs.git -b ros2 src/nmea_msgs
git clone https://github.com/tilk/rtcm_msgs src/rtcm_msgs
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select nmea_msgs rtcm_msgs
```
