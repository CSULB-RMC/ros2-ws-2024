#!/bin/bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan bitrate 500000
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
