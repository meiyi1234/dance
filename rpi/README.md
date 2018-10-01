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

**Graphical desktop sharing**

14. Follow instructions here to [set up VNC](https://www.realvnc.com/en/connect/docs/raspberry-pi.html#setting-up-your-raspberry-pi). Using this setup does not require a static IP, just an account with RealVNC and an internet-connected pi.


### client.py
See comments in the file for instructions and details.


### comm.py
1. On the rpi, run `sudo nano /boot/config.txt`. Add the lines
```
enable_uart=1
dtoverlay=pi3-disable-bt
```
This disables bluetooth to reduce energy consumption. It also redirects the GPIO pins 14 and 15 to ttyAMA0/PL011 module, instead of ttyS0/mini UART. Among other things, this allows higher baud rates, stable baud rates and frame error detection. See [here](https://www.raspberrypi.org/documentation/configuration/uart.md).
Note that the config file does not support inline comments.
2. `sudo systemctl disable hciuart` to disable the bluetooth startup service.

`python comm.py` should work even if these commands are not run since it communicates with the `serial0` alias instead of `ttyAMA0`/`ttyS0` directly.
