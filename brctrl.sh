ifconfig eth0 0.0.0.0 promisc up
brctl addbr bridge
brctl setfd bridge 0
brctl sethello bridge 0
brctl stp bridge off
brctl addif bridge eth0
ifup bridge
dhclient bridge