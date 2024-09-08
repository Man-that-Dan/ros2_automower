#!/bin/bash
sudo apt update && sudo apt upgrade
sudo apt install arduino

# You may need to install python env manager package if your system doesn't already have it
sudo apt install python3-venv

# install platformio command line
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py

# add to path so you can run pio command from anywhere on RPi
echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
source $HOME/.bashrc

curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules


sudo cp 99-platformio-udev.rules /etc/udev/rules.d/99-platformio-udev.rules

sudo service udev restart

sudo apt install minicom