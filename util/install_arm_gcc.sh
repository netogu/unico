#!/bin/bash

# Define toolchain dir name
DIR="arm-toolchain-14.2"

# Create symbolic links
sudo ln -vs /usr/share/${DIR}/bin/arm-none-eabi-gcc /usr/bin/arm-none-eabi-gcc
sudo ln -vs /usr/share/${DIR}/bin/arm-none-eabi-g++ /usr/bin/arm-none-eabi-g++
sudo ln -vs /usr/share/${DIR}/bin/arm-none-eabi-gdb /usr/bin/arm-none-eabi-gdb
sudo ln -vs /usr/share/${DIR}/bin/arm-none-eabi-size /usr/bin/arm-none-eabi-size
sudo ln -vs /usr/share/${DIR}/bin/arm-none-eabi-objcopy /usr/bin/arm-none-eabi-objcopy

echo "Symbolic links created for ${DIR}."
