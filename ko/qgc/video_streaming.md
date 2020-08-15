# Odroid C1에서 QGroundControl로의 동영상 스트리밍

> **TIp** 이 글 일부는 내용이 오래됐습니다. 커뮤니티 구성원들은 최근 우분투 버전에 맞추어 절차를 다시 테스트해주었으면 좋겠고, 위치에 오드로이드 설치 절차 내용도 가져다 두었으면 좋겠습니다.

이 주제에서는 보조 컴퓨터([Odroid C1](https://magazine.odroid.com/wp-content/uploads/odroid-c1-user-manual.pdf))에 붙은 카메라(로지텍 C920)로 동영상을 촬영한 후 (WiFi에 실어) 다른 컴퓨터로 실시간 전송하여 *QGroundControl*에 띄우는 방법을 보여드립니다. 

하드웨어 구성은 아래 그림과 같습니다. 다음 부분으로 구성합니다:

* 오드로이드 C1
* 로지텍 카메라 C920
* WiFi 모듈 TP-LINK TL-WN722N

![Setup](../../assets/videostreaming/setup_whole.jpg)

절차는 우분투 14.04에서 테스트했지만 최근 우분투 버전에서도 비슷한 방식으로 동작할지 모릅니다.

## 오드로이드 C1에 리눅스 환경 설치

리눅스 환경(우분투 14.04)를 설치하려면, [오드로이드 C1 자습서](http://web.archive.org/web/20180617111122/http://pixhawk.org/peripherals/onboard_computers/odroid_c1)(기록 보관 사이트)에 주어진 절차를 따르십시오. 자습서에는 오드로이드 C1에 UART 케이블로 연결하는 방법과 이더넷 케이블로 연결하는 방법을 알려줍니다.

## 대안 전원 연결 설정

오드로이드 C1에는 5V 직류 전원 커넥터로 전원을 공급합니다. 오드로이드를 드론에 연결할 경우 아래 그림과 같이 홀을 관통하는 [방식](https://learn.sparkfun.com/tutorials/how-to-solder---through-hole-soldering)으로 5V 직류 연결 커넥터 옆 핀 두개의 납땜을 권장합니다. 전원은 예제 설정과 같이 점퍼 케이블(그림 위쪽의 적색 케이블)을 직류 전압 공급원(5V)과 오드로이드 C1에 연결하고, 점퍼 케이블(그림 위쪽의 흑색)을 회로의 접지부와 오드로이드 C1의 접지 핀을 연결하여 끌어옵니다.

![Power Pins](../../assets/videostreaming/power-pins.jpg)

## 오드로이드 C1 WiFi 연결 활성화

In this this tutorial the WiFi module TP-LINK TL-WN722N is used. To enable WiFi connection for the Odroid C1, follow the steps described in the [Odroid C1 tutorial](http://web.archive.org/web/20180617111122/http://pixhawk.org/peripherals/onboard_computers/odroid_c1) in the section Establishing wifi connection with antenna.

## WiFi 액세스 포인트로 설정

This sections shows how to set up the Odroid C1 such that it is an access point. The content is taken from the pixhawk.org "access point" tutorial (no longer available) with some small adaptions. Odroid C1으로 촬영한 카메라 동영상을 컴퓨터에서 실행하는 QGroundControl로의 실시간 전송을 활성화하려 한다면 이 절의 내용을 따를 필요가 없습니다. However, it is shown here because setting up the Odroid C1 as an access point allows to use the system in a stand-alone fashion. The TP-LINK TL-WN722N is used as a WiFi module.

In the following steps it is assumed that the Odroid C1 assigns the name wlan0 to your WiFi module. Change all occurrences of wlan0 to the appropriate interface if different (e.g. wlan1).

### Onboard Computer as Access Point

For a more in depth explanation, you can look at [RPI-Wireless-Hotspot](http://elinux.org/RPI-Wireless-Hotspot)

Install the necessary software

```bash
sudo apt-get install hostapd udhcpd
```

DHCP를 설정합니다. `/etc/udhcpd.conf` 파일을 편집하십시오

```bash
start 192.168.2.100 # This is the range of IPs that the hotspot will give to client devices.
end 192.168.2.200
interface wlan0 # The device uDHCP listens on.
remaining yes
opt dns 8.8.8.8 4.2.2.2 # The DNS servers client devices will use (if routing through the Ethernet link).
opt subnet 255.255.255.0
opt router 192.168.2.1 # The Onboard Computer's IP address on wlan0 which we will set up shortly.
opt lease 864000 # 10 day DHCP lease time in seconds
```

기타 모든 'opt' 항목은 비활성화하든지, 설정 방법을 안다면 적절하게 설정해야합니다.

`/etc/default/udhcpd` 파일을 편집하여 다음 줄을:

```bash
DHCPD_ENABLED="no"
```

다음처럼 주석 처리하십시오.

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

If you have followed the *Odroid C1 tutorial* (originally pixhawk.org) to set up the WiFi connection, you might have created the file `/etc/network/intefaces.d/wlan0`. Please comment out all lines in that file such that those configurations have no effect anymore.

Configure HostAPD: To create a WPA-secured network, edit the file `/etc/hostapd/hostapd.conf` (create it if it does not exist) and add the following lines:

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
    

Change `ssid=`, `channel=`, and `wpa_passphrase=` to values of your choice. SSID is the hotspot's name which is broadcast to other devices, channel is what frequency the hotspot will run on, wpa_passphrase is the password for the wireless network. For many more options see the file `/usr/share/doc/hostapd/examples/hostapd.conf.gz`. Look for a channel that is not in use in the area. You can use tools such as *wavemon* for that.

`/etc/default/hostapd` 파일을 편집하여 다음 줄을:

    #DAEMON_CONF=""
    

다음처럼 주석을 해제하고 변수값을 입력하십시오.

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

# 지스트리머 설치

컴퓨터와 오드로이드 C1에 지스트리머 꾸러미를 설치하고 스트리밍을 시작하려면, [QGroundControl README](https://github.com/mavlink/qgroundcontrol/blob/master/src/VideoReceiver/README.md)에 설명하는 내용을 따르십시오.

uvch264 플러그인으로 오드로이드에서 스트리밍 전송을 시작할 수 없다면, v4l2src 플러그인도 함께 시작하게 할 수도 있습니다.

```sh
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-h264,width=1920,height=1080,framerate=24/1 ! h264parse ! rtph264pay ! udpsink host=xxx.xxx.xxx.xxx port=5000
```

여기사 `xxx.xxx.xxx.xxx` 부분은 QGC를 실행하는 컴퓨터의 IP 주소입니다.

> **Tip** `Permission denied` 오류가 뜬다면, 위 명령 앞에 `sudo` 를 붙여야합니다.

대신 아래와 같이 현재 사용자를 `video` 그룹에 추가할 수 있습니다(그리고 로그아웃한 다음 다시 로그인하십시오).

    sh
      sudo usermod -aG video $USER

If everything works, you should see the video stream on the bottom left corner in the flight-mode window of *QGroundControl* as shown in the screenshot below.

![QGC displaying video stream](../../assets/videostreaming/qgc-screenshot.png)

If you click on the video stream, the satellite map is shown in the left bottom corner and the video is shown in the whole background.