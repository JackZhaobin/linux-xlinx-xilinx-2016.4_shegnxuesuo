#make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- xilinx_zynq_hzhy_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- menuconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- UIMAGE_LOADADDR=0x8000 uImage
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules

