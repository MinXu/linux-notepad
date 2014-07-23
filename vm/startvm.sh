#!/bin/bash

if [ $1 = create ];then
	qemu-img create -f qcow2 target.img 30G

elif [ $1 = install ];then
	sudo qemu-system-x86_64 -m 2048 -drive file=/home/xumin/qemu/target.img,if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -cdrom /home/xumin/ubuntu-14.04-server-amd64.iso.1 -smp 4 -soundhw es1370
elif [ $1 = run ];then
	sudo qemu-system-x86_64 -m 2048 -drive file=/home/xumin/qemu/target.img,if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -smp 4 -soundhw es1370
elif [ $1 = image ];then
#	sudo qemu-system-x86_64 -kernel ~/linux_source/linux-3.13.0/arch/x86/boot/bzImage  -initrd rootfs.img  -append root="/dev/ram0 rw"
#	sudo qemu-system-x86_64 -m 512 -kernel ~/www/bzImage -initrd ~/qemu/rootfs.img_d -append "root =/dev/ram0 rw" -enable-kvm -localtime -net nic,vlan=0,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot d -smp 2 -soundhw es1370
#	sudo qemu-system-x86_64 -m 2048 -kernel ~/www/bzImage -initrd ~/qemu/rootfs.img_d -append "root =/dev/ram0 rw",if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -smp 4 -soundhw es1370
	sudo qemu-system-x86_64 -m 2048 -kernel ~/www/bzImage -append "kgdboc=ttyAMA0 kgdbwait root=/dev/ram0 nfsroot=10.80.104.234:/home/xumin/cmc rw",if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -smp 4 -soundhw es1370
else
	echo "vm start.sh"
	echo "please input the paramter"
	echo "create:     create the image"
	echo "intall:     install the distrubuit OS"
	echo "run:        run the distrubuit OS"
	echo "image:      startup the debug kernel and rootfs"
fi
