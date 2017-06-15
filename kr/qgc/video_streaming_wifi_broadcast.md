# QGroundControl에서 장거리 비디오 스트리밍

이 페이지는 카메라(Logitech C920) 달린 컴패니온 컴퓨터(Odroid C1이나 C0)를 셋업하는 방법을 알려줍니다. 비디오 스트림은 Odroid C1을 통해 네트워크 컴퓨터로 전송되며 QGroundControl를 실행하는 컴퓨터에서 볼 수 있습니다. 이 셋업은 비연결(broadcast) 모드에서 WiFi를 사용합니다.  

전체 하드웨어 셋업은 다음 부품들로 구성 :

TX (콥터) 쪽:
* Odroid C1
* Logitech camera C920
* WiFi module  ALPHA AWUS051NH v2.

RX(ground station) 쪽:
* Linux 컴퓨터
* WiFi module  ALPHA AWUS051NH v2.


## 왜 일반 wifi는 장거리 비디오 전송에 적합하지 않을까?
 - 연결 : 비디오 전송기와 수신기는 관련이 높습니다. 만약 한쪽 장치가 연결을 끊어지면(신호가 약해서) 비디오 전송은 바로 끊어지게 됩니다.
 - 에러에 자유로운 전송 : Wifi는 올바른 데이터를 전송하거나 아니면 데이터가 없거나이다. FPV 시나리오에서 만약 약간의 에러가 있는 데이터를 수신하는 경우 사용할 수 없다는 뜻입니다. 유용한 데이터를 수신했지만 결국에는 비디오를 볼수 없다는 뜻입니다.
 - 양방향 통신 : 소스에서만 데이터를 보낸다할지라도 wifi를 사용하는 경우 양방향 데이터 flow가 필요합니다. 이유는 wifi 수신기는 받은 패킷에 대해서 ack를 해야합니다. 만약 전송기가 어떤 ack도 받지 못한다면 연결을 끊게 됩니다. 따라서 비행체와 지상 모두 똑같이 강한 세기의 전송기와 안테나가 필요합니다. 공중에서는 단방향 안테나와 강한 전송기로 셋업하고 지상은 high-gain 안테나를 상뇽해서 설정하는 방식은 일반 wifi에서 불가능합니다.g an omnidirectional antenna and a weak device on the ground using a high-gain antenna is not possible with normal wifi.
 - rate 제어 : 일반 wifi 연결에서 신호가 너무 약하면 자동으로 낮은 전송 rate로 전환됩니다. 이런 이유로 (자동으로) rate를 선택하는게 비디오 데이터를 전송하기에 너무 낮을 수 있습니다. 이런 방식으로 데이터가 큐에 쌓이고 예상치 못한 지연을 생기면 수초동안 지속될 수 있습니다.
 - 1대1 전송 : 프레임을 브로드캐스트하지 않거나 일반 wifi 데이터 flow가 1대1 연결과 유사한 기술. bystander가  
 One to one transfers: Unless you use broadcast frames or similar techniques a normal wifi data flow is a one to one connection. A scenario where a bystander just locks onto your “channel” as in analog video transmission to watch your stream is not easy to accomplish using traditional wifi.
 - Limited diversity: Normal wifi limits you to the number of diversity streams that your wifi card offers.

## What wifibroadcast makes different
Wifibroadcast puts the wifi cards into monitor mode. This mode allows to send and receive arbitrary packets without association. Additionally, it is also possible to receive erroneous frames (where the checksum does not match). This way a true unidirectional connection is established which mimics the advantageous properties of an analog link. Those are:

 - The transmitter sends its data regardless of any associated receivers. Thus there is no risk of sudden video stall due to the loss of association
 - The receiver receives video as long as it is in range of the transmitter. If it gets slowly out of range the video quality degrades but does not stall. Even if frames are erroneous they will be displayed instead of being rejected.
 - The traditional scheme “single broadcaster – multiple receivers” works out of the box. If bystanders want to watch the video stream with their devices they just have to “switch to the right channel”
 - Wifibroadcast allows you to use several low cost receivers in parallel and combine their data to increase probability of correct data reception. This so-called software diversity allows you to use identical receivers to improve relieability as well as complementary receivers (think of one receiver with an omnidirectional antenna covering 360° and several directional antennas for high distance all working in parallel)
 - Wifibroadcast uses Forward Error Correction to archive a high reliability at low bandwidth requirements. It is able to repair lost or corrupted packets at the receiver.


## 하드웨어 수정
Alpha WUS051NH는 높은 전원이 필요한 카드고 TX 동안 전류소모가 많습니다. 만약 USB로부터 전원을 준다면 Odroid C1/C0에 포트를 리셋할 것입니다. 5V BEC로 직접 연결할 필요가 있습니다. 2가지 방식으로 이를 할 수 있습니다 :

 1. 커스텀 usb 케이블 만들기
 2. usb 포트 주변 odroid PCB에 5V 전선을 끊고 BEC에 결선합니다.
    전원과 그라운드 사이에 전압 스파크를 필터링하기 위해서 470uF low ESR 캐패시터 추가를 권장합니다.

## 소프트웨어 셋업
wifibroadcast [소스](https://github.com/svpcom/wifibroadcast) 다운로드 받기

커널 패치의 필요성 :

 1. TX rate lock을 활성화 시키기. ``mac80211-radiotap-bitrate_mcs_rtscts.linux-4.4.patch``를 사용하세요. 대신에 삽입된 radiotap 패킷에 대해서 데이터 rate를 지정하는 방법은 없습니다.
 2. TX power lock을 활성화 시키기. ``ez-wifibroadcast-1.4-kernel-4.4-patches.diff``를 사용하세요. 카드로 최대로 지원할 수 있도록 tx power를 lock합니다.
 3. 잘못된 FSC(checksum)을 가진 프레임의 RX를 활성화 시키기. ``ez-wifibroadcast-1.4-kernel-4.4-patches.diff``를 사용하세요. 이는 선택적이며 현재 코드에서는 사용하지 않습니다.

TX쪽에만 커널 패치를 할 수 있습니다.

### 필요한 TX 쪽:

1. RTP 스트림 출력을 위한 카메라 셋업:
```
gst-launch-1.0 uvch264src device=/dev/video0 initial-bitrate=6000000 average-bitrate=6000000 iframe-period=1000 name=src auto-start=true \
               src.vidsrc ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5600
```
 2. TX 모드에서 wifibroadcast 셋업:

```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
ifconfig wlan1 down    #assume that TX card is wlan1
iw reg set BO          #set CRDA region with max allowed TX power
iw dev wlan1 set monitor otherbss fcsfail
ifconfig wlan1 up
iwconfig wlan1 channel 149
./tx -r 24 wlan1
```
149 wifi channel(in 5GHz band)에서 24Mbit/s 데이터 rate를 사용해서 wifibroadcast를 셋업합니다. 들어오는 데이커에 대해서 UDP 포트 5600을 listen합니다.

### 필요한 RX 쪽:

 1. RX 모드에서 wifibroadcast를 셋업:
```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
ifconfig wlan1 down    #assume that RX card is wlan1
iw reg set BO          #set the same region as TX to ensure that you can listen TX channel
iw dev wlan1 set monitor otherbss fcsfail
ifconfig wlan1 up
iwconfig wlan1 channel 149
./rx wlan1
```
 2. qgroundcontrol를 실행하거나
```
gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
             ! rtph264depay ! avdec_h264 ! clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false
```
비디오를 디코드하기 위해서

## FAQ
Q: 원래 wifibroadcast와 차이점은 무엇인가?

A: wifibroadcast의 원래 버전은 byte-stream을 입력으로 사용하고 고정 길이(1024가 디폴트)의 패킷으로 나눕니다. 만약 라디오 패킷을 잃어버리고 FEC로 보정이 되지 않으면 스트림의 임의의(예상못한) 위치에 구멍이 생기게 됩니다. 데이터 프로토콜이 임의로 지워지는 것을 회복하지 못하면 아주 좋지 않은 상황이 됩니다. 데이터 소스로 UDP를 사용하기 위해서 다시 쓸수 있고 소스 UDP 패킷을 라디오 패킷으로 패킹합니다. 이제 라디오 패킷은 payload 사이즈에 따라서 가변 길이를 가집니다. 이렇게 하면 비디오 지연을 크게 줄여줍니다.

Q: 어떤 데이터 타입이 wifibroadcast를 사용해서 전송될 수 있을까?

A: 패킷 사이즈 <= 1466인 어떤 UDP라도 가능. 예를 들면 RTP내부에 x264 혹은 Mavlink.  

Q: What are transmission guarancies?

A: Wifibrodcast use FEC (forward error correction) which can recover 4 lost packets from 12 packets block with default settings.
   You can tune it (both TX and RX simultaniuosly!) to fit your needs.

Q: I have a lot of frame drops and messages ``XX packets lost``. What is this?

A: This is can be due to:
   1. Signal power is too low. Use high-power card or annennas with more gain. Use directed antenna on RX side. Use additional RX card for diversity (add wlan2, wlan3, ... to rx program)
   2. Signal power is too high. Especially if you use 30dBm TX indoors. Try to reduce TX power (for example hack CRDA database inside kernel and make
      several regions with power limit 10dBm and 20dBm).
   3. Interference with other WiFi. Try to change WIFI channel and/or WIFI band. CAUTION: Don't use band where you RC TX operates on! Or setup RTL properly to avoid model loss.
      You can increase FEC block size (by default it is 8/12 (8 data blocks and 4 fec blocks), but it will also increase latency. Use additional RX card for diversity (add wlan2, wlan3, ... to rx program)

## TODO
1. Do a flight test with different cards/antennas.
2. Investigate how to set TX power without CRDA hacks.
3. Tune FEC for optimal latency/redundancy.

## Wifi Cards:

The following Atheros chipsets should work:

 -  Atheros AR9271, Atheros AR9280, Atheros AR9287

The following Ralink chipsets should work:

 -  RT2070, RT2770, RT2870, RT3070, RT3071, RT3072, RT3370, RT3572, RT5370, RT5372, RT5572

However, there might be whatever small issues that prevent some cards from working, so if you want to play it safe, choose one of the cards that have been tested by different people and definitely work:

 -  CSL 300Mbit Stick (2.4/5Ghz, Diversity, RT5572 chipset)
 -  Alfa AWUS036NHA (2.3/2.4Ghz, high power, Atheros AR9271 chipset)
 -  TPLink TL-WN722N (2.3/2.4Ghz, Atheros AR9271 chipset)
 -  ALFA AWUS051NH v2 (2.4Ghz/5Ghz, high power, Ralink RT3572 chipset)
 -  ALFA AWUS052NH v2 (2.4Ghz/5Ghz, Diversity, high power, Ralink chipset)
 -  TP-Link-TL-WDN3200 (2.4/5Ghz, Diversity, RT5572 chipset)
 -  Ralink RT5572 (2.4/5Ghz, Diversity???, RT5572 chipset)

On the other hand, if everybody gets the same cards, we'll never find out which other ones work. There are also very small and lightweight RT5370 cards available in china shops for under 4$. Aliexpress for example has a lot of cheap wifi cards in general. It would be nice if you report back your findings in case you tried a wifi card that is not listed here.

* AWUS036NHA This adapter will provide around 280mW output power. Ranges of several kilometers have been reported (with directional antennas though).

* TL-WN722N This adapter will provide around 60mW output power. Range should be roughly around 800-1000m with 2.1dbi stock antennas. IMPORTANT: Under certain circumstances, the second antenna on the PCB causes bad reception. Please disconnect the antenna by removing the white PCB component on the back of the PCB like shown below (in the picture, the component was soldered to the upper pad to be able to reverse the mod if needed)

* CSL 300Mbit stick This adapter provides around 30mw output power. Range on 5Ghz is not very high, around 200-300m. Stock antennas are not usable on 5Ghz, as they are simple 2.4Ghz 2.1dbi sleeved-dipole antennas.

When used as an Rx dongle, bad blocks can occur when the received signal strength is higher than -20dbm. This can be worked-around by using more than one adapter and pointing antennas in different directions / polarizations.

* AWUS051NH This adapter will provide around 330mw output power. Range on 5Ghz is around 800-1000m. Stock antenna is not recommended because they have 5dbi gain, which will give a too-flat radiation pattern.

* AWUS052NH This adapter will provide around 330mw output power. This is the same adapter as the 051NH, but with two TX chains. Stock antennas are not recommended because they have 5dbi gain, which will give a too-flat radiation pattern.

## Links:
 -  [Original version](https://befinitiv.wordpress.com/wifibroadcast-analog-like-transmission-of-live-video-data/) of wifibroadcast
