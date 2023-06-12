#!/bin/bash

echo "Set static IP for jetson nano"

host_ip=192.168.144.100
host_mask=255.255.255.0
dev_name=eth0


sudo ifconfig ${dev_name} down
sleep 3
sudo ifconfig ${dev_name} ${host_ip} netmask ${host_mask} up
sleep 1

