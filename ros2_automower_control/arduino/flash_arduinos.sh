#!/bin/bash

# build/compile the program
cd ./diff_drive_arduino
pio run

#  upload diff drive controller
pio run --target upload -e mega2560

cd ..

#upload blade motor controller
cd ./blade_motor_controller
pio run
pio run --target upload leonardo