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

sudo apt install git
```

# Install ROS Rolling

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
colcon build --symlink-install
```

## SSH keys

```bash
ssh-keygen -t ed25519
scp -o PreferredAuthentications=password projectwork-rpi.pub project@trajectory-estimator.local:~/temp.pub
ssh project@trajectory-estimator.local -o PreferredAuthentications=password
mkdir -p ~/.ssh
echo temp.pub >> ~/.ssh/authorized_keys
```

## User setup

It is recommended to use separate users, however for this project the users only serve to setup the git configs for each user.

```bash
export NEW_USER=my_user
sudo adduser $NEW_USER
sudo usermod -aG sudo $NEW_USER

# sudo touch /home/$NEW_USER/user_setup.sh
sudo nano /home/$NEW_USER/user_setup.sh
```

Add the following content to the new script:

```bash
#!/bin/bash
export GIT_AUTHOR_NAME="..."
export GIT_COMMITTER_NAME="..."
export GIT_AUTHOR_EMAIL="..."
export GIT_COMMITTER_EMAIL="..."
exec sudo -E -u project env HOME=/home/project bash -c "cd ~ && exec bash --login"
```

Additionally, add the user to sudoers and modify the script permissions.

```bash
sudo chmod +x /home/$NEW_USER/user_setup.sh

sudo usermod --shell /home/$NEW_USER/user_setup.sh $NEW_USER

echo "$NEW_USER ALL=(project) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/"$NEW_USER"_project
sudo chmod 0440 /etc/sudoers.d/"$NEW_USER"_project
```

Test user setup:

```bash
ssh my_user@192.168.55.1
echo $GIT_AUTHOR_NAME
```

## Setup interactive user for nautilus access (optional)

```bash
sudo useradd -M interactive
sudo passwd -d interactive
sudo usermod -aG project interactive
sudo chmod -R g+rX /home/project

# adding keys without home directory
cat temp.pub | sudo tee -a /etc/ssh/interactive_authorized_keys
sudo chown interactive:interactive /etc/ssh/interactive_authorized_keys
sudo chmod 600 /etc/ssh/interactive_authorized_keys
sudo service ssh restart
```

Add the following to `/etc/ssh/sshd_config`

```
Match User interactive
    AuthorizedKeysFile /etc/ssh/interactive_authorized_keys
```

## Remove ability to SSH directly to main account

```bash
# create recovery account (optional)
sudo useradd recovery
sudo usermod -aG sudo recovery

# remove password from main account
sudo passwd -d project
```

Add to `/etc/ssh/sshd_config`:

```
Match User project
    PasswordAuthentication no
    PubkeyAuthentication no
```

## Peripheral rules

Find the vendor and product IDs for the devices. If the peripheral is connected to `/dev/ttyUSB0`, then run:

```bash
udevadm info -a -n /dev/ttyUSB0
```

If the device is not shown in the device tree, you can find the IDs by running `sudo dmesg`.

Create udev rules for the devices inside `/etc/udev/rules.d/99-peripheral-links.rules`:

```
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c332", SYMLINK+="optical"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="imu"
```

Update device rules:

```bash
sudo udevadm control --reload
sudo udevadm trigger
```

Test that devices are recognized:

```bash
ls /dev/imu /dev/optical
```
