#!/bin/bash
echo "nvidia" | sudo -S chmod 777 /dev/CH340_usb
# sudo -E su -p -l intelnuc <<BASH
whoami
cd /home/intelnuc/Documents/vgd_rm2023_vision_nn/build/
make -j8
while :
do
	/home/intelnuc/Documents/vgd_rm2023_vision_nn/build/run
done
# BASH

#gnome-terminal -x bash -c
#while : 
#do
#	/home/intelnuc/Documents/vgd_rm2023_vision_nn/build/run
#done

# su -p intelnuc <<BASH


