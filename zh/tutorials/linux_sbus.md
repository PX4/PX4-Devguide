# Linux系统下使用S.Bus驱动

这个驱动可以使飞控通过串口读取Sbus接收机的数据，最大支持16通道。  
兼容市面上的常见接收机，诸如frsky，乐迪，甚至sbus编码器。  
板载串口、USB串口、以及其他串口均可支持。  
仅支持linux平台。 
这个驱动位于 drivers/linux_sbus 下

## 必要组件

* NPN 三极管 * 1  
* 10K 电阻 * 1  
* 1K 电阻 * 1  

**注意：**上述器件型号不限,可从淘宝上任意购买，因为接收机的电流较小，所以三极管的型号任意。  

## 可选组件
usb转ttl *1  ,推荐PL2302

上述组件也可从淘宝购买,若不购买,则可使用板载串口替代。  

## 组装
请安下列方式对器件进行连线  

* S.bus信号线 &rarr; 1K电阻 &rarr; NPN三极管基级  
* NPN 三极管发射级 &rarr; GND  
* 3.3v  VCC &rarr; 10K电阻 &rarr; NPN三极管发射级集电极 &rarr; USB-to-TTY rxd  
* 5.0v  VCC &rarr; S.Bus vcc  
* GND &rarr; S.Bus GND  

## 电路图

![Signal inverter circuit diagram](../../assets/driver_sbus_signal_inverter_circuit_diagram.png)
这个电路是个倒相器，树莓派这类设备无法直接读取S.bus信号，必须借助倒相器辅助，将信号进行反转，才可读取。


## 安装实例

![Signal inverter breadboard](../../assets/driver_sbus_signal_inverter_breadboard.png)

## 启动
把 `linux_sbus start -d /dev/ttyUSB0 -c 8` 加入配置文件中，即可自动运行,并通过`/dev/ttyUSBO`监听8个通道  
原始配置文件位于Firmware下的posix-configs目录中，如果按照官方文档编译，做了make upload的操作，则文件会存放在
目标机的`/home/pi`目录中,修改你使用的文件即可。
