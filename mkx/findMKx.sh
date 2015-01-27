#!/usr/bin/env bash
IPV4_RANGE=$(ip -4 addr show | grep global | awk '{print $2}')
sudo ip neigh flush all;
nmap -sP $IPV4_RANGE 2>&1 > /dev/null;
arp -n | grep "04:e5:48"
