#!/bin/bash
sudo ip link set up can0 type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
