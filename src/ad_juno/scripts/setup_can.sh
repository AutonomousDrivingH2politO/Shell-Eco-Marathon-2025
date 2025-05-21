#!/bin/bash

modprobe can
modprobe mttcan
modprobe can_raw

ip link set can1 type can bitrate 1000000
ip link set can1 up

ip link set can0 type can bitrate 250000
ip link set can0 up
