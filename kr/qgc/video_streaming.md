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

Odroid C1은 5V DC 잭으로 전원을 공급받습니다. 만약 Odroid가 드론에 장착되면, 아래 그림에서와 같은 [방법](https://learn.sparkfun.com/tutorials/how-to-solder---through-hole-soldering)으로 5V DC잭 옆에 2개 핀을 납땜하는 것을 추천합니다. 예제 셋업에서는 Odroid C1에 있는 점퍼 케이블(위 이미지에서 붉은색)과 그라운드 핀에 있는 점퍼 케이블(검은색)을 통해 DC 전압 소스 (5V)에 연결해서 전원을 공급받습니다.

![](../../assets/videostreaming/power-pins.png)

## Odroid C1에 대한 WiFi 연결 활성화
이 튜토리얼에서 WiFi 모듈 TP-LINK TL-WN722N가 사용됩니다. Odroid C1에 대해서 WiFi 연결을 활성화시키기 위해서 [Odroid C1 튜토리얼](https://pixhawk.org/peripherals/onboard_computers/odroid_c1)의 안테나가 있는 Wifi 연결 구성 섹션에서 설명한 단계를 따라 진행합니다.


## WiFi Access Point 설정
이 섹션은 Odroid C1을 access point로 동작할 때 셋업하는 방법을 보여줍니다. 내용은 [튜토리얼](https://pixhawk.org/peripherals/onboard_computers/access_point)에서 일부분을 가지고 왔습니다. Odroid C1 카메라에서 QGroundControl로 스트림을 활성화시키기 위해서는 이 섹션을 따라서할 필요는 없습니다. 하지만 여기서는 Odroid C1을 access point처럼 셋업해서 스탠드-얼론 방식으로 사용할 수 있는 방법을 보여줍니다. TP-LINK TL-WN722N는 WiFi 모듈로 사용합니다. Odroid C1은 wlan0을 WiFi 모듈 이름에 할당합니다. 만약 다른 경우(예로 wlan1)라면 wlan0의 모든 부분을 적절한 인터페이스로 변경합니다.

### Access Point로 온보드 컴퓨터
좀더 자세한 설명은 [RPI-Wireless-Hotspot](http://elinux.org/RPI-Wireless-Hotspot)을 참고하세요.

필요한 소프트웨어를 설치

<div class="host-code"></div>

```bash
sudo apt-get install hostapd udhcpd
```

DHCP를 설정. `/etc/udhcpd.conf` 파일을 수정

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

제대로 여러분이 이해하고 았다면 모든 다른 'opt' 엔트리들은 비활성화시키거나 적절하게 활성화시켜야 합니다.

`/etc/default/udhcpd` 파일을 수정하고 해당 라인을 변경:

<div class="host-code"></div>

```bash
DHCPD_ENABLED="no"
```

아래와 같이

<div class="host-code"></div>

```bash
#DHCPD_ENABLED="no"
```

온보드 컴퓨터에 고정 IP 주소를 줘야 합니다. `/etc/network/interfaces` 파일을 수정하고 해당 라인인 `iface wlan0 inet dhcp` (혹은 `iface wlan0 inet manual`)을 다음으로 대체 :

```sh
auto wlan0
iface wlan0 inet static
address 192.168.2.1
netmask 255.255.255.0
network 192.168.2.0
broadcast 192.168.2.255
wireless-power off
```

원본 (WiFi Clinet) 자동 설정을 비활성화시킵니다. 해당 라인을 변경합니다. (서로 붙어서 있을 않을 수도 있고 해당 위치에 없을 수도 있음) :

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

WiFi 연결을 셋업하기 위해서 [Odroid C1 tutorial](https://pixhawk.org/peripherals/onboard_computers/odroid_c1)을 따라했다면 `/etc/network/intefaces.d/wlan0` 파일을 생성했을 것입니다. 해당 파일에 있는 모든 라인을 커맨트처리해서 해당 설정이 영향을 미치지 않도록 합니다.

HostAPD 설정: WPA-secured 네트워크를 생성하기 위해서 `/etc/hostapd/hostapd.conf` 파일을 수정해 주세요. (만약 존재하지 않는 경우 생성하도록 합니다) 그리고 다음 라인을 추가합니다 :


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
`ssid=`, `channel=`와 `wpa_passphrase=`을 여러분이 선택한 값으로 변경합니다. SSID는 hotspot의 이름이고 다른 장치로 브로드캐스트됩니다. 채널은 hotspot이 실행될 주파수고 wpa_passphrase는 무선 네트워크 패스워드입니다. 더 상세한 옵션을 `/usr/share/doc/hostapd/examples/hostapd.conf.gz` 파일을 참고하세요. 해당 지역에서 사용하지 않는 채널을 찾아보세요. wavemon 같은 도구를 이용할 수 있습니다.

`/etc/default/hostapd` 파일을 수정하고 해당 라인을 변경:

<div class="host-code"></div>

```
#DAEMON_CONF=""
```
to:
```
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```
온보드 컴퓨터는 무선 hotspot을 호스팅하기 시작할 것입니다. 부팅시에 hotspot이 동작되도록 하기 위해서는 아래와 같이 추가 명령을 실행합니다:

<div class="host-code"></div>

```
sudo update-rc.d hostapd enable
sudo update-rc.d udhcpd enable
```

온보드 컴퓨터 자신이 Access Point가 될 수 있고 ground station이 연결되도록 할 수 있습니다. 만약 실제 Access Poin처럼 동작하기를 원한다면(WiFi 트래픽을 온보드 컴퓨터의 이더넷 연결로 라우팅) 라우팅과 네트워크 변환(NAT)를 설정해야 합니다.
커널에서 IP 포워딩 활성화 :

<div class="host-code"></div>

```sh
sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
```
커널에서 NAT를 활성화시키기 위해서 다음 명령을 실행합니다:

<div class="host-code"></div>

```sh
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
```

계속 동작하게 만들려면 다음 명령을 수행합니다 :

<div class="host-code"></div>

```sh
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
```

이제 /etc/network/interfaces 파일을 수정하고 파일의 맨밑에 다음 라인을 추가합니다:

<div class="host-code"></div>

```sh
up iptables-restore < /etc/iptables.ipv4.nat
```

# gstreamer 설치

컴퓨터와 Odroid C1에 gstreamer 패키지를 설치하고 스트림을 구동시키려면 [QGroundControl README](https://github.com/mavlink/qgroundcontrol/blob/master/src/VideoStreaming/README.md)에 있는 지시를 따르도록 합니다.

만약 Odroid에 uvch264s 플러그인으로 스트림을 구동시킬 수 없다면, v4l2src 플러그인으로 구동시켜보세요:

<div class="host-code"></div>

```sh
 gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-h264,width=1920,height=1080,framerate=24/1 ! h264parse ! rtph264pay ! udpsink host=xxx.xxx.xxx.xxx port=5000
```
`xxx.xxx.xxx.xxx`는 QGC가 실행하고 있는 IP주소입니다. `Permission denied`와 같은 시스템 에러가 발생하면, `sudo`를 붙여서 위에 명령을 실행하세요.

모든게 정상적이라면 아래 화면과 같이 비디오 스트림이 QGroundControl의 비행 모드 창의 왼쪽 바닥 모서리에 나타나게 됩니다.

![](../../assets/videostreaming/qgc-screenshot.png)

비디오 스트림을 클릭하면 위성지도가 왼쪽 바닥 모서리에 보여지고 비디오는 백그라운드로 나타납니다.
