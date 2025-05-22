# Setup P2P Ethernet (wired connection)

This document describes how you can get wired connection to the device (web interface and SSH). The ethernet device needs to be setup and assigned an IP on both the host and target devices.

## Target setup

Identify the ethernet device by running e.g. `ip link`, usually `eth0`. With the ethernet cable connected to the host and target, setup the target link and assign an IP.

```bash
sudo ip link set dev eth0 up
sudo ip addr add 192.168.100.2/24 dev eth0

ip addr show eth0
```

If you wish to have the ethernet setup persistant across boots, add this to e.g. `/etc/netplan/02-peer-ethernet.yaml`:

```bash
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0: # device name here
      dhcp4: no
      addresses:
        - 192.168.100.2/24
```

### Target firewall

```bash
sudo ufw status
sudo ufw enable # if inactive

sudo ufw allow in on eth0 to any port ssh
sudo ufw allow in on wlan0 to any port ssh # if you use wlan0
sudo ufw allow 8000/tcp # allow port 8000 for web interface
sudo ufw reload
```

## Host setup (Tested on Ubuntu 24.04)

Identify the device again, and assign a different IP.

```bash
sudo ip link set dev eth0 up
sudo ip addr add 192.168.100.1/24 dev eth0

ip addr show eth0
```

If the connection shows `UP` with an assigned IP, you should be able to SSH to the target.

```bash
ssh username@192.168.100.2
```

To make the connection persistant, add create a file to `/etc/netplan/02-peer-ethernet.yaml` with the following content:

```bash
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0: # device name here
      dhcp4: no
      addresses:
        - 192.168.100.1/24
```
