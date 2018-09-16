## Setting up
These instructions are for installing Raspbian with a desktop environment using a Windows PC. After the code has been finalized we may want to install the headless Raspbian Lite to reduce [electricity consumption](https://raspberrypi.stackexchange.com/a/39933).

**Installing Raspbian**

1. Follow the instructions on the official website to [download and flash Raspbian](https://www.raspberrypi.org/documentation/installation/installing-images/) on an SD card.

**Enabling SSH**

2. Add a file called `ssh` to the boot directory of the SD card.

**Connecting to the NUS Wifi**

> Note: I am currently not connecting to the NUS Wifi since its difficult to figure out the pi's IP address headlessly. Instead I have set up a hotspot using Connectify, which informs me of connected devices' addresses.

3. If you have an external monitor and keyboard, go to step 4. Otherwise, go to step 5.
4. Follow instructions on the [official docs](https://projects.raspberrypi.org/en/projects/raspberry-pi-getting-started/4). Then go to step 10.
5. Install Bonjour print services for windows for mDNS support. 
6. Using and HDMI cable for external display and a mouse (no keyboard needed), boot the pi. Connect to a hotspot (eg mobile, connectify) that has no password.
7. Ensure your laptop is also connected to the hotspot. Using Putty, ssh into `pi@raspberrypi.local`, default password `raspberry`.
8. Enter `passwd` to change password to `danceteam2`.
9. Install an on-screen keyboard by entering `sudo apt install matchbox-keyboard`. This will allow you to make small changes without needing to SSH.
10. Run `sudo apt update` and `sudo apt upgrade`, which should update kernel drivers. If you don't do this, the raspberry pi loses connection to the internet after about 2 minutes. It remains connected to NUS_STU_2-4GHz but cannot ping the router gateway, other devices on the network, or any website. This occurs with both dynamic and static IPs. Running `dmesg` seems to indicate a broadcom driver error (brcmfmac).
11. `sudo nano /etc/wpa_supplicant/wpa_supplicant.conf` to configure a connection to the NUS wifi network. First, hash your password by entering `echo -n plaintext_password_here | iconv -t utf16le | openssl md4` in a bash terminal. Then use the following as a template:
```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=SG

network={
        ssid="NUS_STU_2-4GHz"
        proto=RSN
        key_mgmt=WPA-EAP
        pairwise=CCMP
        auth_alg=OPEN
        eap=PEAP
        identity="nusstu\e0001111"
        password=hash:your-hashed-password
        phase1="peaplabel=0"
        phase2="auth=MSCHAPV2"
        priority=1
}
```

**Setting a static IP (not working)**

12. Follow [these instructions](https://raspberrypi.stackexchange.com/questions/37920/how-do-i-set-up-networking-wifi-static-ip-address/74428#74428) to set up a static IP address with dhcpcd. Example `/etc/dhcpcd.conf`:
```
interface wlan0
static ip_address=172.31.38.121/22
static routers=172.31.36.1
static domain_name_servers=172.19.215.140 172.19.50.39
```
13. Connect to the NUS_STU_2-4GHz network on your PC and ssh into `pi@<static-ip>`. You can now SSH into the pi as long as you are connected to NUS_STU_2-4GHz (probably).

**Graphical desktop sharing**

14. Follow instructions here to [set up VNC](https://www.realvnc.com/en/connect/docs/raspberry-pi.html#setting-up-your-raspberry-pi). Using this setup does not require a static IP, just an account with RealVNC and an internet-connected pi.


### client.py
See comments in the file for instructions and details.


### comm.py

