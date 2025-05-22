# Raspberry Pi 4 setup

Install rpi-imager, and install Ubuntu Server 22.04 (64 Bit). Use hostname `trajectory-estimator` and username `project` for consistency.

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

## Install ROS Humble

Instructions for building ROS are modified from [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
sudo apt install libopenblas-dev
```

## Increase boot time

Reduce the cloud wait service to reduce boot time significantly.

```bash
sudo mkdir -p /etc/systemd/system/systemd-networkd-wait-online.service.d/
cat <<EOF | sudo tee /etc/systemd/system/systemd-networkd-wait-online.service.d/timeout.conf
[Service]
TimeoutStartSec=5
EOF

sudo systemctl daemon-reload
sudo systemctl restart systemd-networkd-wait-online.service
```
