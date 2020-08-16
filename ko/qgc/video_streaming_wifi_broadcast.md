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

## Hardware Setup

The hardware setup consists of the following parts:

On TX (UAV) side:

* [NanoPI NEO2](http://www.friendlyarm.com/index.php?route=product/product&product_id=180) (and/or Raspberry Pi if using Pi camera).
* [Logitech camera C920](https://www.logitech.com/en-us/product/hd-pro-webcam-c920?crid=34) or [Raspberry Pi camera](https://www.raspberrypi.org/products/camera-module-v2/).
* WiFi module [ALPHA AWUS036ACH](https://www.alfa.com.tw/products_detail/1.htm).

On RX (ground station side):

* Any computer with Linux (tested on Fedora 25 x86-64).
* WiFi module [ALPHA AWUS036ACH](https://www.alfa.com.tw/products_detail/1.htm). See [wifibroadcast wiki > WiFi hardware](https://github.com/svpcom/wifibroadcast/wiki/WiFi-hardware) for more information on supported modules.

If you don't need high-power cards, you can use any card with **rtl8812au** chipset.

## Hardware Modification

Alpha AWUS036ACH is a high power card that uses too much current while transmitting. If you power it from USB it will reset the port on most ARM boards. So it must be directly connected to 5V BEC in one of two ways:

1. Make a custom USB cable ([cut `+5V` wire from USB plug and connect it to BEC](https://electronics.stackexchange.com/questions/218500/usb-charge-and-data-separate-cables)
2. Cut a `+5V` wire on PCB near USB port and wire it to BEC (don't do this if doubt - use custom cable instead). Also I suggest to add 470uF low ESR capacitor (like ESC has) between power and ground to filter voltage spikes. Be aware of [ground loop](https://en.wikipedia.org/wiki/Ground_loop_%28electricity%29) when using several ground wires.

## Software Setup

To setup the (Linux) development computer:

1. Install **libpcap** and **libsodium** development libs and install **python2.7** + **python-twisted** packages.
2. Download [wifibroadcast sources](https://github.com/svpcom/wifibroadcast).
3. See [Setup HOWTO](https://github.com/svpcom/wifibroadcast/wiki/Setup-HOWTO) how to build debian, rpm or tar.gz package and configure it.

### UAV Setup

1. Setup camera to output RTP stream:
    
    a. Logitech camera C920 camera: 
    
        gst-launch-1.0 uvch264src device=/dev/video0 initial-bitrate=4000000 average-bitrate=4000000 iframe-period=3000 name=src auto-start=true \
                   src.vidsrc ! queue ! video/x-h264,width=1280,height=720,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5602 b. RaspberryPi camera: ```raspivid --nopreview --awb auto -ih -t 0 -w 1280 -h 720 -fps 49 -b 4000000 -g 147 -pf high -o - | gst-launch-1.0 fdsrc ! h264parse !  rtph264pay !  udpsink host=127.0.0.1 port=5602```

2. Configure WFB for drone as described in [Setup HOWTO](https://github.com/svpcom/wifibroadcast/wiki/Setup-HOWTO)

3. Configure autopilot (px4 stack) to output telemetry stream at 1500kbps (other UART speeds doesn't match well to NEO2 frequency dividers). Setup [mavlink-router](https://github.com/intel/mavlink-router) to route MAVLink packets to/from WFB: 
        [UdpEndpoint wifibroadcast]
        Mode = Normal
        Address = 127.0.0.1
        Port = 14550

### Ground Station Setup

1. 동영상을 디코딩하려면 *QGroundStation*을 실행하거나 다음 명령을 실행하십시오: 
        gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
                 ! rtph264depay ! avdec_h264 ! clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false

2. Configure WFB for ground station as described in [Setup HOWTO](https://github.com/svpcom/wifibroadcast/wiki/Setup-HOWTO).

## 미세 전파 조정

With default settings WFB use radio channel 165 (5825 MHz), width 20MHz, MCS #1 (QPSK 1/2) with long GI. This provides ~7 mbit/s of **effective** speed (i.e. usable speed after FEC and packet encoding) for **both directions** in sum, because WiFi is half-duplex. So it is suitable for video down stream 720p@49fps (4 mbit/s) + two full-speed telemetry streams (uplink and downlink). If you need a higher bandwidth you can use other MCS index (for example 2 or greater) and/or 40MHz channel.

## 안테나와 다양성

For simple cases you can use omnidirectional antennas with linear (that bundled with wifi cards) or circular leaf ([circularly polarized Coverleaf Antenna](http://www.antenna-theory.com/antennas/cloverleaf.php)) polarization. If you want to setup long distance link you can use multiple wifi adapters with directional and omnidirectional antennas. TX/RX diversity for multiple adapters supported out of box (just add multiple NICs to `/etc/default/wifibroadcast`). If your WiFi adapter has two antennas (like Alfa AWU036ACH) TX diversity is implemented via [STBC](https://en.wikipedia.org/wiki/Space%E2%80%93time_block_code). Cards with 4 ports (like Alfa AWUS1900) are currently not supported for TX diversity (only RX is supported).

## 자주 묻는 질문

**Q:** *What is a difference from original wifibroadcast?*

**A:** The original version of wifibroadcast used a byte-stream as input and split it to packets of fixed size (1024 by default). With this scheme if radio packets were lost (and this was not corrected by FEC) the result was random/unexpected holes in the stream. This is especially bad if the data protocol is not resistant to such random erasures.

The new version was rewritten to use UDP as data source and pack one source UDP packet into one radio packet. Radio packets now have variable size that depends on payload size. This significantly reduces a video latency.

**Q:** *What type of data can be transmitted using wifibroadcast?*

**A:** Any UDP with packet size <= 1466. For example x264 inside RTP or MAVLink.

**Q:** *What are transmission guarantees?*

**A:** Wifibrodcast use FEC (forward error correction) which can recover 4 lost packets from 12 packets block with default settings. You can tune it (both TX and RX simultaneously!) to fit your needs.

> **Caution** RC TX 동작시 해당 대역을 사용하지 마십시오! 만일 그대로 사용하려면 모델 손실을 막기 위해 RTL 속성을 설정하십시오.

**Q:** *Is only Raspberry PI supported?*

**A:** Wifibroadcast is not tied to any GPU - it operates with UDP packets. But to get RTP stream you need a video encoder (with encode raw data from camera to x264 stream). In my case RPI is only used for video encoding (because RPI Zero is too slow to do anything else) and all other tasks (including wifibroadcast) are done by other board (NanoPI NEO2).

## Theory

Wifibroadcast puts the WiFi cards into monitor mode. This mode allows to send and receive arbitrary packets without association and waiting for ACK packets. [Analysis of Injection Capabilities and Media Access of IEEE 802.11 Hardware in Monitor Mode](https://github.com/svpcom/wifibroadcast/blob/master/patches/Analysis%20of%20Injection%20Capabilities%20and%20Media%20Access%20of%20IEEE%20802.11%20Hardware%20in%20Monitor%20Mode.pdf) [802.11 timings](https://github.com/ewa/802.11-data)

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
- 무선 통신 혼선에 잘 견딤 | - Small community  
- 512MB SDRAM  
- No camera interface                                                                                                       |

This article chose to use Pi Zero as camera board (encode video) and NEO2 as main UAV board (wifibroadcast, MAVLink telemetry, etc.)

## TODO

1. Make prebuilt images. Pull requests are welcome.
2. Do a flight test with different cards/antennas.