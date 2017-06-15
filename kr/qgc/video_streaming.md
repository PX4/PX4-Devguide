# QGroundControl에서 비디오 스트리밍
이 페이지는 카메라(Logitech C920) 달린 컴패니온 컴퓨터(Odroid C1이나 C0)를 셋업하는 방법을 알려줍니다. 비디오 스트림은 Odroid C1을 통해 네트워크 컴퓨터로 전송되며 QGroundControl를 실행하는 컴퓨터에서 볼 수 있습니다. 이 셋업은 비연결(broadcast) 모드에서 WiFi를 사용합니다.

전체 하드웨어 셋업은 아래와 같습니다. 다음과 같은 부분들로 구성:
* Odroid C1
* Logitech camera C920
* WiFi module TP-LINK TL-WN722N

![](../../assets/videostreaming/setup-whole.png)

## Odroid C1에 리눅스 환경 설치

Linux 환경(Ubuntu 14.04)를 설치하기 위해서, [Odroid C1 튜토리얼](https://pixhawk.org/peripherals/onboard_computers/odroid_c1)의 지시를 따릅니다. 이 튜토리얼에서 UART 케이블이 있는 Odroid C1에 접근하는 방법과 이더넷 연결을 구성하는 방법을 소개하고 있습니다.

## 전원 연결 셋업

Odroid C1은 5V DC 잭으로 전원을 공급받습니다. 만약 Odroid가 드론에 장착되면, 아래 그림에서와 같은 [방법](https://learn.sparkfun.com/tutorials/how-to-solder---through-hole-soldering)으로 5V DC잭 옆에 2개 핀을 납땜하는 것을 추천합니다. 점퍼 케이블로 DC 전압 소스 (5V)에 연결해서 전원을 공급받으면
The Odroid C1 can be powered via the 5V DC jack. If the Odroid is mounted on a drone, it is recommended to solder two pins next to the 5V DC jack by applying the through-hole soldering [method](https://learn.sparkfun.com/tutorials/how-to-solder---through-hole-soldering) as shown in the figure below. The power is delivered by connecting the DC voltage source (5 V) via a jumper cable (red in the image above) with the Odroid C1 and connect the ground of the circuit with a jumper cable (black in the image above) with a ground pin of the Odroid C1 in the example setup.

![](../../assets/videostreaming/power-pins.png)

## Enable WiFi connection for Odroid C1
In this this tutorial the WiFi module TP-LINK TL-WN722N is used. To enable WiFi connection for the Odroid C1, follow the steps described in the [Odroid C1 tutorial](https://pixhawk.org/peripherals/onboard_computers/odroid_c1) in the section Establishing wifi connection with antenna.


## Configure as WiFi Access Point
This sections shows how to set up the Odroid C1 such that it is an access point. The content is taken from this [tutorial](https://pixhawk.org/peripherals/onboard_computers/access_point) with some small adaptions. To enable to stream the video from the camera via the Odroid C1 to the QGroundControl that runs on a computer it is not required to follow this section. However, it is shown here because setting up the Odroid C1 as an access point allows to use the system in a stand-alone fashion. The TP-LINK TL-WN722N is used as a WiFi module. In the ensuing steps it is assumed that the Odroid C1 assigns the name wlan0 to your WiFi module. Change all occurrences of wlan0 to the appropriate interface if different (e.g. wlan1).

### Onboard Computer as Access Point
For a more in depth explanation, you can look at [RPI-Wireless-Hotspot](http://elinux.org/RPI-Wireless-Hotspot)

Install the necessary software

<div class="host-code"></div>

```bash
sudo apt-get install hostapd udhcpd
```

Configure DHCP. Edit the file `/etc/udhcpd.conf`

<div class="host-code"></div>

```bash
start 192.168.2.100 # This is the range of IPs that the hostspot will give to client devices.
end 192.168.2.200
interface wlan0 # The device uDHCP listens on.
remaining yes
opt dns 8.8.8.8 4.2.2.2 # The DNS servers client devices will use (if routing through the ethernet link).
opt subnet 255.255.255.0
opt router 192.168.2.1 # The Onboard Computer's IP address on wlan0 which we will set up shortly.
opt lease 864000 # 10 day DHCP lease time in seconds
```
All other 'opt' entries should be disabled or configured properly if you know what you are doing.

Edit the file `/etc/default/udhcpd` and change the line:

<div class="host-code"></div>

```bash
DHCPD_ENABLED="no"
```

to

<div class="host-code"></div>

```bash
#DHCPD_ENABLED="no"
```

You will need to give the Onboard Computer a static IP address. Edit the file `/etc/network/interfaces` and replace the line `iface wlan0 inet dhcp` (or `iface wlan0 inet manual`) to:

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

<div class="host-code"></div>

```sh
allow-hotplug wlan0
wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf
iface default inet dhcp
```
to:

<div class="host-code"></div>

```sh
#allow-hotplug wlan0
#wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf
#iface default inet dhcp
```

If you have followed the [Odroid C1 tutorial](https://pixhawk.org/peripherals/onboard_computers/odroid_c1) to set up the WiFi connection, you might have created the file `/etc/network/intefaces.d/wlan0`. Please comment out all lines in that file such that those configurations have no effect anymore.

Configure HostAPD: To create a WPA-secured network, edit the file `/etc/hostapd/hostapd.conf` (create it if it does not exist) and add the following lines:


```
auth_algs=1
channel=6            # Channel to use
hw_mode=g
ieee80211n=1          # 802.11n assuming your device supports it
ignore_broadcast_ssid=0
interface=wlan0
wpa=2
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
# Change the to the proper driver
driver=nl80211
# Change these to something else if you want
ssid=OdroidC1
wpa_passphrase=QGroundControl

```

Change `ssid=`, `channel=`, and `wpa_passphrase=` to values of your choice. SSID is the hotspot's name which is broadcast to other devices, channel is what frequency the hotspot will run on, wpa_passphrase is the password for the wireless network. For many more options see the file `/usr/share/doc/hostapd/examples/hostapd.conf.gz`.
Look for a channel that is not in use in the area. You can use tools such as wavemon for that.

Edit the file `/etc/default/hostapd` and change the line:

<div class="host-code"></div>

```
#DAEMON_CONF=""
```
to:
```
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```
Your Onboard Computer should now be hosting a wireless hotspot. To get the hotspot to start on boot, run these additional commands:

<div class="host-code"></div>

```
sudo update-rc.d hostapd enable
sudo update-rc.d udhcpd enable
```

This is enough to have the Onboard Computer present itself as an Access Point and allow your ground station to connect. If you truly want to make it work as a real Access Point (routing the WiFi traffic to the Onboard Computer’s ethernet connection), we need to configure the routing and network address translation (NAT).
Enable IP forwarding in the kernel:

<div class="host-code"></div>

```sh
sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
```
To enable NAT in the kernel, run the following commands:

<div class="host-code"></div>

```sh
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
```

To make this permanent, run the following command:

<div class="host-code"></div>

```sh
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
```

Now edit the file /etc/network/interfaces and add the following line to the bottom of the file:

<div class="host-code"></div>

```sh
up iptables-restore < /etc/iptables.ipv4.nat
```

# gstreamer Installation

To install gstreamer packages on the computer and on the Odroid C1 and start the stream, follow the instruction  given in the [QGroundControl README](https://github.com/mavlink/qgroundcontrol/blob/master/src/VideoStreaming/README.md).

If you cannnot start the stream on the Odroid with the uvch264s plugin, you can also try to start it with the v4l2src plugin:

<div class="host-code"></div>

```sh
 gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-h264,width=1920,height=1080,framerate=24/1 ! h264parse ! rtph264pay ! udpsink host=xxx.xxx.xxx.xxx port=5000
```
Where `xxx.xxx.xxx.xxx` is the IP address where QGC is running. If you get the system error: `Permission denied`, you might need to prepend `sudo` to the  command above.

If everything works, you should see the video stream on the bottom left corner in the flight-mode window of QGroundControl as shown in the screeenshot below.

![](../../assets/videostreaming/qgc-screenshot.png)

If you click on the video stream, the satellite map is shown in the left bottom cornor and the video is shown in the whole background.
