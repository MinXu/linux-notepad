#!/bin/bash

KERNEL_PATH=/home/min/linux_source/linux-3.13.0/arch/x86/boot/bzImage
ROOTFS_PATH=/home/min/qemu/rootfs.img_d
TARGET_PATH=/home/min/qemu/target.img
ISO_PATH=/home/min/ubuntu-14.04-server-amd64.iso.1
HOSTNAME=workspace
XEN_DIR=/home/min/xenvm
#KERNEL_PATH=/boot/vmlinuz-3.13.11.4
if [ $1 = create ];then
	qemu-img create -f qcow2 target.img 30G

elif [ $1 = install ];then
	sudo qemu-system-x86_64 -m 2048 -drive file=$TARGET_PATH,if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -cdrom $ISO_PATH -smp 4 -soundhw es1370
elif [ $1 = run ];then
	sudo qemu-system-x86_64 -m 2048 -drive file=$TARGET_PATH,if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -smp 4 -soundhw es1370
elif [ $1 = image ];then
#	sudo qemu-system-x86_64 -net nic -net user,tftp="$(pwd)" -m 512 -kernel $KERNEL_PATH -initrd $ROOTFS_PATH  -append root="/dev/ram0 rw"
#	sudo qemu-system-x86_64 -m 512 -kernel $KERNEL_PATH -initrd $ROOTFS_PATH -append "console=ttyS0 root=/dev/ram0 rw",if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -smp 1 -soundhw es1370 -k en-us -s -serial stdio


#======================kbdb0============================append==========================================================================
#	sudo qemu-system-x86_64 -m 512 -kernel $KERNEL_PATH -initrd $ROOTFS_PATH -append "kgdboc=kms,kbd,ttyS0,115200 kgdbwait kgdbcon root =/dev/ram0 rw",if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -smp 1 -soundhw es1370 -k en-us -s -serial tcp::4321,server -alt-grab

#	gdb vmlinux
#	set remotebaud 115200
#	target remote localhost:4321
#	echo g > /proc/sysrq-trigge

#=======================kgdb1============================-s -S===========================================================================
#	sudo qemu-system-x86_64 -m 512 -kernel $KERNEL_PATH -initrd $ROOTFS_PATH -append "root =/dev/ram0 rw",if=virtio,cache=none -enable-kvm -localtime -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -smp 1 -soundhw es1370 -k en-us -s -S

#	gdb vmlinux
#       target remote tcp::1234

#=======================kgdb2============================nfs============================================================================
#    nfs...  gdboc=ttyS0,115200  kgdbwait kgdbcon 
#            -s -S
#	sudo qemu-system-x86_64 -m 512 -kernel $KERNEL_PATH -append "kgdboc=ttyS0,115200  kgdbwait kgdbcon console=ttyS0 root=/dev/nfs nfsroot=10.80.104.234:/home/share rw ip=10.80.104.253:10.80.104.234:10.80.104.1:255.255.255.0",if=virtio,cache=none -enable-kvm -localtime -serial stdio -net nic,vlan=0,model=virtio,macaddr=52-54-00-12-34-01 -net tap,vlan=0,ifname=tap0,script=no -boot c -smp 1 -soundhw es1370 -k en-us -serial tcp::4321,server -alt-grab
   #Developer
	#/etc/exports
	    #/home/share   *(rw,insecure,sync,no_root_squash)
	#sudo exportfs -av(rvf)
	#showmount
	#service rpcbind restart
	#service nfs-kernel-server restart
   #Target
	#make menuconfig
	#    build nfs in kernel
	#    build network driver(intel e1000) in kernel

#=======================kgdb3============================nfs==localhost mount===========================================================
#    nfs... localhost
	sudo qemu-system-x86_64 -m 512 -kernel $KERNEL_PATH -append "kgdboc=ttyS0,115200 console=ttyS0 root=/dev/nfs nfsroot=10.0.2.2:/home/share rw ip=10.0.2.15:10.0.2.2:10.0.2.1:255.255.255.0",if=virtio,cache=none -enable-kvm -localtime -serial stdio -net nic -net user -boot c -smp 1 -soundhw es1370 -k en-us #-serial tcp::4321,server -alt-grab
   #[ip] : [boot/root server ip] : [gateway ip] : [netmask]。 
      #eg.”ip=10.0.2.15:10.0.2.2:10.0.2.1:255.255.25.0”
      #默认情况下，local ip:10.0.2.15 host ip:10.0.2.2 DHCP server:10.0.2.15 DNS server:10.0.2.3

   #-redir [tcp|udp]:host-port:[guest-host]:guest-port 
      #1) qemu-system-arm -redir tcp :5555::23
          #telnet localhost 5555
      #2) qemu-system-arm -redir tcp :5555::22
          #ssh localhost -p 5555
      #3) qemu-system-arm -redir tcp :5555::3389
          #rdesktop localhost:5555
      #4) qemu-system-arm -smb /share_dir
	  #mount -t cifs -o username=min,password=min //10.0.2.2/min local_dir
      #5) qemu-system-arm -vnc :1 -daemonize -usbdevice tablet 
	  #vncview localhost:1
elif [ $1 = xp ];then
	sudo qemu-system-x86_64 -m 512 -hda /home/min/xp/xp.img  -enable-kvm -localtime  -boot d -redir tcp:53389::3389 -smp 2 -soundhw all -daemonize
     #rdesktop localhost:3389
#	sudo qemu-system-x86_64 -m 512 -hda /home/min/xp/xp.img  -enable-kvm -localtime  -boot d -redir tcp:53389::3389 -smp 2 -soundhw all -daemonize -vnc :1
     #vncview localhost:1
elif [ $1 = xen_i ];then
	#1) install:
	#python -m SimpleHTTPServer 9999
        sudo xen-create-image --hostname=$HOSTNAME --dist=trusty --dir=$XEN_DIR --mirror=http://mirrors.163.com/ubuntu --role=udev --verbose --pygrub --dhcp --passwd --force
	#2) ubuntu
	#sudo xen-create-image --hostname=$HOSTNAME --size=10Gb --swap=1gb --ip=10.80.104.123 --netmask=255.255.255.0 --gateway=10.80.104.1 --memory=512Mb --arch=amd64 --kernel=/boot/vmlinuz-3.13.11.4 --initrd=/boot/initrd.img-3.13.11.4 --dist=trusty  --dir=$XEN_DIR --mirror=http://mirrors.163.com/ubuntu  --role=udev,gdm --verbose --pygrub  --passwd --force
	#3) centos
	#sudo xen-create-image --hostname=$HOSTNAME --size=10Gb --swap=1gb --ip=10.80.104.123 --netmask=255.255.255.0 --gateway=10.80.104.1 --memory=512Mb --arch=amd64  --dist=centos-6 --install-method=rinse  --dir=$XEN_DIR --mirror=http://mirrors.163.com/centos/7.0.1406/os/x86_64/   --verbose --pygrub  --passwd --force
elif [ $1 = xen_cr ];then
	#1) creat configure:
        sudo xl create /etc/xen/$(HOSTNAME).cfg
        #修改IP
	#2) start os
        #sudo xl create /etc/xen/workstation.cfg
elif [ $1 = xen_co ];then
	#1) login os
        sudo xl console $HOSTNAME
	#2) logout os
        #Press “Ctrl+]” will get you out of the vm console.
elif [ $1 = xen_v ];then
	#vnc
	sudo xl vncviewer $HOSTNAME
elif [ $1 = xen_l ];then
	#list os
        sudo xl list
elif [ $1 = xen_d ];then
	#1) shutdown
	sudo xl shutdown $HOSTNAME
	#2) destroy
	#sudo xl destroy workstation
else
	echo "vm start.sh"
	echo "please input the paramter"
	echo "create:     create the image"
	echo "intall:     install the distrubuit OS"
	echo "run:        run the distrubuit OS"
	echo "image:      startup the debug kernel and rootfs"
	echo "xp:         startup the xp"
	echo "xen_i:      install the distrubuit os (pv)"
	echo "xen_cr:     create the os or cfg"
	echo "xen_co:     login the os via console"
	echo "xen_v:      login the os via vncviewer"
	echo "xen_l:      list the os"
	echo "xen_d:      shudown (destroy) the os"
fi
