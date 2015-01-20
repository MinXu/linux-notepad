#!/bin/sh -e

HOST=192.168.1.2
XLOADER=qoros_xloader.bin
BIOS=arm1176_U002-4010-09CC_T02.bin
UBOOT=u-boot.bin
KERNEL=uImage
PREFIX=$1/

download_all()
{
    echo "--------download all files--------"
    tftp -gr ${PREFIX}${XLOADER} ${HOST} || { echo "****Download ${XLOADER} FAILED!!!****"; exit; }
    tftp -gr ${PREFIX}${BIOS} ${HOST} || { echo "****Download ${BIOS} FAILED!!!****"; exit; }
    tftp -gr ${PREFIX}${UBOOT} ${HOST} || { echo "****Download ${UBOOT} FAILED!!!****"; exit; }
    tftp -gr ${PREFIX}${KERNEL} ${HOST} || { echo "****Download ${KERNEL} FAILED!!!****"; exit; }
}

update_xloader()
{
    echo "--------update X-LOADER: mtd0--------"
    flash_eraseall /dev/mtd0
    nandwrite -p /dev/mtd0 ${XLOADER}
}

update_bios()
{
    echo "--------update BIOS: mtd13--------"
    flash_eraseall /dev/mtd13
    nandwrite -p /dev/mtd13 ${BIOS}
}

update_kernel()
{
    echo "-------update kernel--------"
    flash_eraseall /dev/mtd4
    nandwrite -p /dev/mtd4 ${KERNEL}
}

update_kernel_backup()
{
    echo "-------update kernel backup1: mtd8--------"
    flash_eraseall /dev/mtd8
    nandwrite -p /dev/mtd8 ${KERNEL}
    echo "-------update kernel backup2: mtd11--------"
    flash_eraseall /dev/mtd11
    nandwrite -p /dev/mtd11 ${KERNEL}
}


update_uboot()
{
    echo "-------update uboot mtd3--------"
    flash_eraseall /dev/mtd3
    nandwrite -p /dev/mtd3 ${UBOOT}
}

update_uboot_backup()
{
    echo "-------update uboot backup1: mtd7--------"
    flash_eraseall /dev/mtd7
    nandwrite -p /dev/mtd7 ${UBOOT}
    echo "-------update uboot backup2: mtd10--------"
    flash_eraseall /dev/mtd10
    nandwrite -p /dev/mtd10 ${UBOOT}
}

download_all
update_xloader
update_bios
update_kernel
update_kernel_backup
update_uboot
update_uboot_backup
