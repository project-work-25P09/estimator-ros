# Setup

Setup the devices according to the device model:

- [Setting up Raspberry Pi 4](./SETUP-RPi4.md)
- [Setting up Raspberry Pi 5](./SETUP-RPi5.md)

## SSH keys

```bash
ssh-keygen -t ed25519
scp -o PreferredAuthentications=password projectwork-rpi.pub project@trajectory-estimator.local:~/temp.pub
ssh project@trajectory-estimator.local -o PreferredAuthentications=password
mkdir -p ~/.ssh
echo temp.pub >> ~/.ssh/authorized_keys
```

## User setup (if developing on target)

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

## Peripheral device rules (udev)

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
