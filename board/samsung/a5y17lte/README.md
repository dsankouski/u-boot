# Chain - loaded bootloader for Samsung A5 (2017)

## Roadmap
- [x] Boots, uart working via on-board debug connector
- [x] Pinctrl driver
- [x] GPIO driver
- [x] I2C-gpio driver
- [x] Uart USB debug cable
- [ ] Clock drivers
- [ ] EMMC driver
- [ ] USB driver
- [ ] Screen driver

## Instructions

### Building
```
export CROSS_COMPILE=aarch64-linux-gnu-
make a5y17lte_defconfig
make menuconfig # to enable additional u-boot features, etc., if needed 
make
```

### Flashing
You should create android boot image, with 
u-boot.bin as a payload instead of linux kernel, like.

```
mkbootimg --base 0x40000000 --kernel_offset 0x00000000 --ramdisk_offset 0x01000000 --tags_offset 0x00000100 --pagesize 2048 --second_offset 0x00f00000 --kernel <u-boot.bin path> -o uboot-test.img
```

Note, that stock Samsung bootloader ignores offsets, set in mkbootimg. Flash like regular android boot image with linux.

### Getting uart output via usb uart cable
> :warning: **Keep your device safe**: MUIC is controlled manually, 
> as opposed to micro-usb phones with 619K resistor in debug cable. Therefore, some precautions needed:
> - do NOT plug in any usb devices, while uart is multiplexed on usb port!
> - uart signal level is 1.8V, so you should either use 1.8V compatible uart board, or a level shifter.
>  
> :warning: **No responsibility for broken devices**

#### Assemble simple usb debug cable:
```
Phone                                      PC
USB type-c                                 UART
A6 (D+, Rx)--------------------------------Tx
A7 (D-, Tx)--------------------------------Rx
A1 and / or (B1) --------------------------GND
A4 and / or (A9, B4, B9)-------------------5V charging
```
This schematic lack any resistances, because phone multiplexes uart on usb
by user choice (holding specific key combination at boot).  
Cable can be made from simple usb type-c cable.

#### Build and flash u-boot with MUIC_MANUAL_SWITCH option

#### Attach cable, turn on phone, hold volume up and volume down buttons

### Load linux kernel
to be done
