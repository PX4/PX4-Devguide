
# Linux系统下使用sbus驱动（drivers/linux_sbus）
这个驱动可以使飞控通过串口读取sbus接收机的数据，最大支持16通道。  
兼容市面上的常见接收机，诸如frsky，乐迪，甚至sbus编码器。  
板载串口、usb串口、以及其他串口均可支持。  
仅支持linux平台。 

## 必要组件

NPN 三极管 * 1  
10K 电阻 * 1  
1K 电阻 * 1  

## 可选组件
usb转ttl *1  

## 组装
请安下列方式对器件进行连线  

S.bus信号线 -> 1K电阻 ->NPN三极管基级  
NPN 三极管发射级 -> GND  
3.3v  VCC-> 10K电阻 -> NPN三极管发射级集电极 ->usbtty rxd  
5.0v  VCC->S.bus vcc  
GND ->S.bus GND  

## 电路图
![](http://www.playuav.com/uploads/article/20160310/56cf0f65bb1f7437c1618041a30dc308.png)
![](https://github.com/crossa/raspx4-sbus-rc-in/blob/master/sbusToSerial_small.jpg?raw=true)  

## 安装实例
![](https://raw.githubusercontent.com/crossa/raspx4-sbus-rc-in/master/example.png)  

## 启动
把 `linux_sbus start -d /dev/ttyUSB0 -c 8` 加入配置文件中，即可自动运行,并通过/dev/ttyUSBO监听8个通道
