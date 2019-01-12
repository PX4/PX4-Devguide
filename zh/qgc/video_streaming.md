# QGroundControl 视频流

此页演示如何设置相机 (logitech c920) 和机载计算机 (odroid c1), 以便通过 odroid c1 将视频流传输到网络计算机, 并显示在 QGC 中。

下图显示了整个硬件设置。 它由以下部分组成:

* Odroid C1
* Logitech 摄像头 C920
* WiFi 模块 TP-LINK TL-WN722N

![Setup](../../assets/videostreaming/setup_whole.jpg)

## 在 odroid c1 中安装 linux 环境

要安装 linux 环境 (ubuntu 14.04), 请按照 [Odroid c1 教程](https://pixhawk.org/peripherals/onboard_computers/odroid_c1) 中给出的说明进行操作。 在本教程中, 它还演示了如何使用 uart 电缆访问 odroid c1, 以及如何建立以太网连接。

## 设置备用电源连接

Odroid c1 可以通过 5v 直流插孔供电。 如果 Odroid 被安装在飞行器上，建议将两个跳线通过插片式的[方法](https://learn.sparkfun.com/tutorials/how-to-solder---through-hole-soldering)焊接在电路上 在例子中，Odroid C1 通过在上图所示的红色跳线连接 DC 电源 (5 V) 和通过上图所示的黑色跳线连接地线被通电。

![Power Pins](../../assets/videostreaming/power-pins.jpg)

## 为 Odroid C1 启用无线网络连接

在这篇教程中使用的是 WiFi 模块 TP-LINK TL-WN722N. 要为 odroid c1 启用 wifi 连接, 请按照 [Odroid c1 教程](https://pixhawk.org/peripherals/onboard_computers/odroid_c1) 中描述的步骤, 在 "用天线建立 wifi 连接" 一节中进行操作。

## 配置 WiFi 为接入点

本节演示如何设置 odroid c1, 使其成为接入点。 内容取自此[这篇教程](https://pixhawk.org/peripherals/onboard_computers/access_point), 并有一些小改动。 为了能够通过 odroid c1 将视频从相机流式传输到在计算机上运行的 QGroundControl, 并不一定需要遵循此部分。 但是, 这篇教程的意义是, 将 odroid c1 设置为接入点允许以独立的方式使用该系统。 在此使用的是 TP-LINK TL-WN722N。 在随后的步骤中, 假定 odroid c1 将 wlan0 的名称分配给您的 wifi 模块。 如果不同, 请将所有出现的 wlan0 更改为相应的接口 (例如 wlan1)。

### 配置机载电脑为接入点

有关更深入解释, 请查阅 [RPI-Wireless-Hotspot](http://elinux.org/RPI-Wireless-Hotspot)

安装必要的软件

```bash
sudo apt-get install hostapd udhcpd
```

配置 DHCP 编辑文件 `/etc/udhcpd.conf`

```bash
start 192.168.2.100 #这是热点将为客户端设备提供的IP范围。
end 192.168.2.200
interface wlan0 # The device uDHCP listens on.
remaining yes
opt dns 8.8.8.8 4.2.2.2 # The DNS servers client devices will use (if routing through the Ethernet link).
opt subnet 255.255.255.0
opt router 192.168.2.1 # The Onboard Computer's IP address on wlan0 which we will set up shortly.
opt lease 864000 # 10 day DHCP lease time in seconds
```

其他“opt”命令不应该被配置。如果您知道自己在做什么，则配置其他命令。

编辑如下文件 `/etc/default/udhcpd`，修改其中的一行：

```bash
DHCPD_ENABLED="no"
```

至

```bash
#DHCPD_ENABLED="no"
```

您需要为机载计算机配置静态 ip 地址。 Edit the file `/etc/network/interfaces` and replace the line `iface wlan0 inet dhcp` (or `iface wlan0 inet manual`) to:

```sh
auto wlan0
iface wlan0 inet static
address 192.168.2.1
netmask 255.255.255.0
network 192.168.2.0
broadcast 192.168.2.255
wireless-power off
```

Disable the original (WiFi Client) auto configuration. Change the lines (they probably will not be all next to each other or may not even be there at all):

```sh
allow-hotplug wlan0
wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf
iface default inet dhcp
```

to:

```sh
#allow-hotplug wlan0
#wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf
#iface default inet dhcp
```

If you have followed the [Odroid C1 tutorial](https://pixhawk.org/peripherals/onboard_computers/odroid_c1) to set up the WiFi connection, you might have created the file `/etc/network/intefaces.d/wlan0`. Please comment out all lines in that file such that those configurations have no effect anymore.

Configure HostAPD: To create a WPA-secured network, edit the file `/etc/hostapd/hostapd.conf` (create it if it does not exist) and add the following lines:

    auth_algs=1
    channel=6            # 要使用的通道
    hw_mode=g
    ieee80211n=1          # 802.11n 假设你的设备支持它
    ignore_broadcast_ssid=0
    interface=wlan0
    wpa=2
    wpa_key_mgmt=WPA-PSK
    wpa_pairwise=TKIP
    rsn_pairwise=CCMP
    # 更改至正确的驱动
    driver=nl80211
    # 如果需要的话，把下面两项改成别的名字和密码
    ssid=OdroidC1
    wpa_passphrase=QGroundControl
    
    

Change `ssid=`, `channel=`, and `wpa_passphrase=` to values of your choice. SSID is the hotspot's name which is broadcast to other devices, channel is what frequency the hotspot will run on, wpa_passphrase is the password for the wireless network. For many more options see the file `/usr/share/doc/hostapd/examples/hostapd.conf.gz`. Look for a channel that is not in use in the area. You can use tools such as *wavemon* for that.

Edit the file `/etc/default/hostapd` and change the line:

    #DAEMON_CONF=""
    

to:

    DAEMON_CONF="/etc/hostapd/hostapd.conf"
    

Your Onboard Computer should now be hosting a wireless hotspot. To get the hotspot to start on boot, run these additional commands:

    sudo update-rc.d hostapd enable
    sudo update-rc.d udhcpd enable
    

This is enough to have the Onboard Computer present itself as an Access Point and allow your ground station to connect. If you truly want to make it work as a real Access Point (routing the WiFi traffic to the Onboard Computer’s Ethernet connection), we need to configure the routing and network address translation (NAT). Enable IP forwarding in the kernel:

```sh
sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
```

To enable NAT in the kernel, run the following commands:

```sh
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
```

To make this permanent, run the following command:

```sh
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
```

Now edit the file /etc/network/interfaces and add the following line to the bottom of the file:

```sh
up iptables-restore < /etc/iptables.ipv4.nat
```

# Gstreamer 安装

To install gstreamer packages on the computer and on the Odroid C1 and start the stream, follow the instruction given in the [QGroundControl README](https://github.com/mavlink/qgroundcontrol/blob/master/src/VideoStreaming/README.md).

If you cannot start the stream on the Odroid with the uvch264s plugin, you can also try to start it with the v4l2src plugin:

```sh
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-h264,width=1920,height=1080,framerate=24/1 ! h264parse ! rtph264pay ! udpsink host=xxx.xxx.xxx.xxx port=5000
```

Where `xxx.xxx.xxx.xxx` is the IP address where QGC is running.

> **Tip** If you get the system error: `Permission denied`, you might need to prepend `sudo` to the command above. Alternatively add the current user to the `video` group as shown below (and then logout/login): 
> 
>     sh
>       sudo usermod -aG video $USER

If everything works, you should see the video stream on the bottom left corner in the flight-mode window of QGroundControl as shown in the screenshot below.

![](../../assets/videostreaming/qgc-screenshot.png)

If you click on the video stream, the satellite map is shown in the left bottom corner and the video is shown in the whole background.