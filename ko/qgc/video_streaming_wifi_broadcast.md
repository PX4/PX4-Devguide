# 장거리 실시간 동영상 전송 및 원시 WiFi 전파를 통한 텔레메트리 통신

이 페이지에서는 UAV에서 지상 통제국으로 실시간으로 동영상을 전송하여 *QGroundControl*에 나타내도록 카메라(Logitech C920 또는 라즈베리 파이 카메라)가 붙은 보조 컴퓨터의 설정 방법을 알려드리겠습니다. 매커니즘에서는 양방향 텔레메트리 연결(SiK 무선 통신)을 제공합니다. 이 설정 과정에서는 미연결 (브로드캐스팅) 모드로 [Wifibroadcast project](https://github.com/svpcom/wifibroadcast/wiki)의 프로그램을 활용합니다.

> **Note** *Wifibroadcast*를 활용하기 전 사용자 여러분의 국가에서 합법적으로 WiFi 무선 통신을 활용할 수 있는지 확인하십시오.

## Wifibroadcast Overview

The *Wifibroadcast project* provides video and telemetry transport that use low-level WiFi packets to avoid the distance and latency limitations of the ordinary IEEE 802.11 stack.

The high level benefits of *Wifibroadcast* include:

* 1:1 map RTP to IEEE 802.11 packets for minimum latency (doesn't serialize to byte steam).
* Smart FEC support (immediately yield packet to video decoder if FEC pipeline without gaps).
* [Bidirectional MAVLink telemetry](https://github.com/svpcom/wifibroadcast/wiki/Setup-HOWTO). You can use it for MAVLink up/down and video down link.
* Automatic TX diversity (select TX card based on RX RSSI).
* Stream encryption and authentication ([libsodium](https://download.libsodium.org/doc/)).
* Distributed operation. It can gather data from cards on different hosts. So you aren't limited to bandwidth of single USB bus.
* Aggregation of MAVLink packets. Doesn't send WiFi packet for every MAVLink packet.
* Enhanced [OSD](https://github.com/svpcom/wifibroadcast_osd) for Raspberry PI (consume 10% CPU on PI Zero).
* Compatible with any screen resolution. Supports aspect correction for PAL to HD scaling.

Additional information is provided in the [FAQ](#faq) below.

## 하드웨어 설정

The hardware setup consists of the following parts:

TX(무인 항공기) 측에서는:

* [나노파이 네오2](http://www.friendlyarm.com/index.php?route=product/product&product_id=180)(그리고 파이 카메라 활용시 라즈베리 파이)
* [로지텍 C920 카메라](https://www.logitech.com/en-us/product/hd-pro-webcam-c920?crid=34) 또는 [라즈베리 파이 카메라](https://www.raspberrypi.org/products/camera-module-v2/).
* [ALPHA AWUS036ACH](https://www.alfa.com.tw/products_detail/1.htm) WiFi 모듈.

RX(지상 통제국) 측에서는:

* 리눅스를 설치한 아무 컴퓨터(페도라 25 x86_64 시험 완료).
* [ALPHA AWUS036ACH](https://www.alfa.com.tw/products_detail/1.htm) 와이파이 모듈. 더 많은 지원 모듈 정보를 보려면 [wifibroadcast 위키 > WiFi 하드웨어](https://github.com/svpcom/wifibroadcast/wiki/WiFi-hardware) 를 참고하십시오.

고수준 신호세기 지원 카드가 필요하지 않으면 **rtl8812au** 칩셋이 달린 카드를 사용해도 됩니다.

## 하드웨어 개조

Alpha AWUS036ACH is a high power card that uses too much current while transmitting. If you power it from USB it will reset the port on most ARM boards. So it must be directly connected to 5V BEC in one of two ways:

1. Make a custom USB cable ([cut `+5V` wire from USB plug and connect it to BEC](https://electronics.stackexchange.com/questions/218500/usb-charge-and-data-separate-cables)
2. Cut a `+5V` wire on PCB near USB port and wire it to BEC (don't do this if doubt - use custom cable instead). Also I suggest to add 470uF low ESR capacitor (like ESC has) between power and ground to filter voltage spikes. Be aware of [ground loop](https://en.wikipedia.org/wiki/Ground_loop_%28electricity%29) when using several ground wires.

## 소프트웨어 설정

(리눅스) 개발 컴퓨터를 설정하려면:

1. **libpcap**과 **libsodium** 개발 라이브러리와 **python2.7** + **python-twisted** 패키지를 설치하십시오.
2. [wifibroadcast 소스 코드](https://github.com/svpcom/wifibroadcast)를 다운로드하십시오.
3. [설치 방법](https://github.com/svpcom/wifibroadcast/wiki/Setup-HOWTO)을 참고하여 데비안, rpm, tar.gz 패키지를 빌드하고 정하는 방법을 살펴보십시오.

### 무인 항공기 설정

1. 카메라에 RTP 실시간 전송 데이터 출력을 설정하십시오:
    
    a. 로지텍 C920 카메라: 
    
        gst-launch-1.0 uvch264src device=/dev/video0 initial-bitrate=4000000 average-bitrate=4000000 iframe-period=3000 name=src auto-start=true \
                   src.vidsrc ! queue ! video/x-h264,width=1280,height=720,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5602 b. 라즈베리 파이 카메라: ```raspivid --nopreview --awb auto -ih -t 0 -w 1280 -h 720 -fps 49 -b 4000000 -g 147 -pf high -o - | gst-launch-1.0 fdsrc ! h264parse !  rtph264pay !  udpsink host=127.0.0.1 port=5602```

2. [설정 방법](https://github.com/svpcom/wifibroadcast/wiki/Setup-HOWTO)에 따라 무인 항공기의 WFB를 설정하십시오

3. 오토파일럿(px4 스택)이 1500kbps 전송률로 텔레메트리 실시간 전송 데이터를 내보내도록 설정하십시오(기타 UART 속도는 네오2 주파수 분할 장치와 잘 맞지 않음). WFB간에 MAVLink 패킷을 주고받을 수 있도록 [mavlink-router](https://github.com/intel/mavlink-router)를 설정하십시오: 
        [UdpEndpoint wifibroadcast]
        Mode = Normal
        Address = 127.0.0.1
        Port = 14550

### 지상 통제국 설정

1. 동영상을 디코딩하려면 *QGroundStation*을 실행하거나 다음 명령을 실행하십시오: 
        gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
                 ! rtph264depay ! avdec_h264 ! clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false

2. [설정 방법](https://github.com/svpcom/wifibroadcast/wiki/Setup-HOWTO)에 따라 지상 통제국의 WFB를 설정하십시오.

## 미세 전파 조정

With default settings WFB use radio channel 165 (5825 MHz), width 20MHz, MCS #1 (QPSK 1/2) with long GI. This provides ~7 mbit/s of **effective** speed (i.e. usable speed after FEC and packet encoding) for **both directions** in sum, because WiFi is half-duplex. So it is suitable for video down stream 720p@49fps (4 mbit/s) + two full-speed telemetry streams (uplink and downlink). If you need a higher bandwidth you can use other MCS index (for example 2 or greater) and/or 40MHz channel.

## 안테나와 다양성

For simple cases you can use omnidirectional antennas with linear (that bundled with wifi cards) or circular leaf ([circularly polarized Coverleaf Antenna](http://www.antenna-theory.com/antennas/cloverleaf.php)) polarization. If you want to setup long distance link you can use multiple wifi adapters with directional and omnidirectional antennas. TX/RX diversity for multiple adapters supported out of box (just add multiple NICs to `/etc/default/wifibroadcast`). If your WiFi adapter has two antennas (like Alfa AWU036ACH) TX diversity is implemented via [STBC](https://en.wikipedia.org/wiki/Space%E2%80%93time_block_code). Cards with 4 ports (like Alfa AWUS1900) are currently not supported for TX diversity (only RX is supported).

## 자주 묻는 질문

**문:** *원래 wifibroadcast와의 차이는 뭔가요?*

**답:** wifibroadcast 원래 버전은 바이트스트림을 입력으로 받고 지정 길이(기본 1024 바이트) 패킷으로 쪼갭니다. 이 방식대로라면 무선 통신 패킷을 잃으면(그리고 FEC로 문제를 해결하지 못하면) 실시간 전송 데이터에 임의의 예기치 못한 구멍이 여기저기 납니다. 특히 데이터 프로토콜이 임의 소거 현상에 대비하지 못하면 좋지 못한 결과를 가져옵니다.

새 버전은 UDP를 데이터 전송 수단으로, 데이터를 UDP 패킷에 실어 무선 통신 패킷으로 보내도록 재작성했습니다. 이제 무선 통신 패킷은 페이로드 길이에 따라 크기가 바뀝니다. 이렇게 하여 동영상 처리 지연을 줄입니다.

**문:** *어떤 데이터 형식을 wifibroadcast로 보낼 수 있나요?*

**답:** 일정 크기의 UDP 패킷입니다. <= 1466. For example x264 inside RTP or MAVLink.

**문:** *전송을 보장하는 기술은 무엇인가요?*

**답:** wifibroadcast는 전송 오류 수정(FEC) 기법을 통해 기본 설정에 따라 12패킷 단위로 들어가는 한 블록에서 손실 패킷 4개 정도 복구할 수 있습니다. 필요에 따라 (TX, RX에서 동시에!) 설정 값을 조율할 수 있습니다.

> **Caution** RC TX 동작시 해당 대역을 사용하지 마십시오! 만일 그대로 사용하려면 모델 손실을 막기 위해 RTL 속성을 설정하십시오.

**문:** *라즈베리 파이에서만 지원하나요?*

**답:** wifibroadcast 기술은 어떤 GPU에 한정하지 않습니다. UDP 패킷에 대해 동작합니다. 그런데 RTP 실시간 전송 데이터를 받으려면 (카메라 원시 데이터에서 x264 실시간 전송 데이터로 변환하는) 동영상 인코더가 필요합니다. 라즈베리 파이의 경우 동영상 인코딩 목적으로만 활용(라즈베리 파이 제로에서 다른 작업을 동시에 하기엔 너무 느리므로)하고 기타 다른 작업(wifibroadcast 동작 포함)은 다른 보드(나노파이 네오2)에서 수행합니다.

## 이론

wifibroadcast는 WiFi 카드를 감시자 모드로 둡니다. 이 모드를 통해 ACK 패킷을 기다리거나 (3-way handshake 등을 통한) 연결을 진행하지 않고도 임의의 패킷을 주고받을 수 있습니다. [802.11 타이밍](https://github.com/ewa/802.11-data) [감시자 모드에서의 IEEE 802.11 하드웨어 데이터 강제 전송 기능 및 미디어 접근 분석](https://github.com/svpcom/wifibroadcast/blob/master/patches/Analysis%20of%20Injection%20Capabilities%20and%20Media%20Access%20of%20IEEE%20802.11%20Hardware%20in%20Monitor%20Mode.pdf) 

#### 무인 항공기에 추천할 ARM 보드는 무엇입니까?

| 보드                                                                                    | 장점                                                                                                                          | 단점                                                                                                                                                              |
| ------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [라즈베리 파이 제로](https://www.raspberrypi.org/products/raspberry-pi-zero/)                 | - 대형 커뮤니티  
- 카메라 지원  
- OMX API를 통한 하드웨어 기반  
동영상 인코더/디코더 지원                                                               | - 북미 외 지역에서의 구매시 어려움( 운송비 >> 구매비 )  
- 느린 CPU  
- USB 버스만 있음  
- 512MB SDRAM                                                                                    |
| [오드로이드 C0](https://www.hardkernel.com/shop/odroid-c0/)                                | - 빠른 CPU  
- EMMC  
- 1GB SDRAM                                                                                             | - 무선 통신 혼선에 매우 민감  
- 메인 라인 커널을 지원하지 않음  
- 높은 가격대  
- 하드웨어 동영상 인코더 동작 안함  
- 인쇄기판 품질 불량(너무 얇고, 접지 핀에 [내열](https://en.wikipedia.org/wiki/Thermal_relief)기능이 없음) |
| [나노파이 네오2](http://www.friendlyarm.com/index.php?route=product/product&product_id=180) | - ARM 64비트 CPU  
- 매우 쌈  
- 메인라인 커널 지원  
- 독립 USB 버스 3개 장착  
- 1Gbps 이더넷 포트  
- UART 포트 3개  
- 기판이 매우 작음  
- 무선 통신 혼선에 잘 견딤 | - 커뮤니티 조직이 작음  
- 512MB SDRAM  
- 카메라 인터페이스 없음                                                                                                                  |

위 내용을 통해 라즈베리 파이 제로를 카메라 보드(동영상 인코딩 용)로, 네오2를 무인항공기 메인보드(wifibroadcast, MAVLink 텔레메트리 통신 등)로 선택했습니다.

## 할 일

1. 사전 빌드 이미지를 만들어야합니다. pull 요청은 언제든 환영합니다.
2. 여러가지 카드/안테나로 비행 시험을 진행해야합니다.