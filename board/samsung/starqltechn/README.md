# WIP U-boot port for Samsung galaxy S9 (SM-G9600) aka starqltechn.

## Roadmap
- [x] Boots, console works (only tested with [kexec method](https://wiki.postmarketos.org/wiki/Bootloaders_porting_using_linux))
- [x] Read button statuses
- [ ] Test if it's possible to use uefi storage driver from stock BL
- [ ] Clock driver for uart
- [ ] Pinctrl driver for uart
- [ ] Setup pinctrl and clocks in uart driver
- [ ] Upstream and refactor uart driver 
- [ ] Upstream and refactor spmi V5 driver changes 
- [ ] Clocks and pinctrl for sdhci
- [ ] Introduce qcom v5 sdhci driver
- [ ] Get linux loaded from sd card
- [ ] Upstream clocks and pinctrl for SDM845
- [ ] Use pinctrl to get pin func to gpio in i2c driver
- [ ] (optional, low priority) Introduce SMBus based on i2c gpio driver.
- [ ] (optional, low priority) Uart on usb switch functionality 

## i2c notes
Pins used for i2c-gpio are not in gpio mode initially. 
To put it into gpio mode:   
mw 0x3921000 0 1  
mw 0x3922000 0 1

## uart notes
baudrates higher than 230400 requires clock to be adjusted (cuz baudrate oversampling is 32), so staying with that for now.

## pinctrl and gpio notes
Only [NORTH](https://github.com/torvalds/linux/blob/master/drivers/pinctrl/qcom/pinctrl-sdm845.c#L21) pins are present in device tree. Access to not `NORTH` pin numbers is invalid. 
See linux driver for `NORTH` 

## MUIC
Communication with muic is performed via SMBus(i2c-14). 
SMBus protocol over i2-gpio needs to be implemented in order send commands to MUIC