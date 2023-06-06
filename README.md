# Project-SmallBot-MCU-V3

小车计划主控代码 (V3)

## 编译

首先安装 RISC-V GCC，[手动安装指南](https://xpack.github.io/riscv-none-embed-gcc/install/#manual-install)。

> 我们推荐使用手动安装方式安装，并将其添加到系统环境变量中。

然后：
``` bash
make
```

## 烧录

推荐使用USB DFU方式烧录。按住主控板上的BOOT按钮，然后按下并松开Reset按钮，再松开Boot按钮，即可进入DFU模式。

### Linux 下

对于Linux系统，请安装[wchisp](https://github.com/ch32-rs/wchisp)。参考对应的README安装即可。

添加下面的 udev 规则到`/etc/udev/rules.d/50-wchisp.rules`：
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="4348", ATTRS{idProduct}=="55e0", MODE="0666"
```
添加完毕后，需要重新拔插你的USB设备。

对于预编译固件，使用以下命令行烧录：
```
wchisp flash main.bin
```

对于开发固件，使用以下命令行烧录：
```
make flash
```
### Windows 下

请到[官方下载页面](https://www.wch.cn/downloads/WCHISPTool_Setup_exe.html)，下载WCHISPTool。下载后运行，选择芯片 CH32V103，选择固件烧录即可。

