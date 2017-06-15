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
 - 양방향 통신 : 소스에서만 데이터를 보낸다할지라도 wifi를 사용하는 경우 양방향 데이터 flow가 필요합니다. 이유는 wifi 수신기는 받은 패킷에 대해서 ack를 해야합니다. 만약 전송기가 어떤 ack도 받지 못한다면 연결을 끊게 됩니다. 따라서 비행체와 지상 모두 똑같이 강한 세기의 전송기와 안테나가 필요합니다. 공중에서는 단방향 안테나와 강한 전송기로 셋업하고 지상은 high-gain 안테나를 사용해서 설정하는 방식은 일반 wifi에서 불가능합니다.
 - rate 제어 : 일반 wifi 연결에서 신호가 너무 약하면 자동으로 낮은 전송 rate로 전환됩니다. 이런 이유로 (자동으로) rate를 선택하는게 비디오 데이터를 전송하기에 너무 낮을 수 있습니다. 이런 방식으로 데이터가 큐에 쌓이고 예상치 못한 지연이 생기면 수초동안 지속될 수 있습니다.
 - 1대1 전송 : 프레임을 브로드캐스트하지 않거나 일반 wifi 데이터 flow가 1대1 연결과 유사한 기술인 경우. 주변 사람이 아날로그 비디오 전송에 사용하는 채널을 맞추면 비디오를 볼수 있는 시나리오가 전통 wifi를 사용하는 경우에는 어렵습니다.
 - 제한된 다이버시티(diversity) : 일반 wifi에서 wifi 카드가 제공하는 많은 다이버시티 스트림 수로 제한됩니다.

## wifibroadcast의 차이점은
wifibroadcast는 wifi 카드를 모니터 모드로 설정합니다. 이 모드에서 임의의 패킷을 연결없이도 주고받을 수 있습니다. 추가로 에러가 있을 수 있는 프레임을 수신하는 것도 가능합니다.(checksum이 매치되지 않는 경우) 이런 방식은 실제로는 단방향 연결을 설정하여 아날로그 링크의 장점을 흉내냅니다.
이런 것들은 :

 - 트랜스미터는 연결된 수신기에 상관없이 데이터를 전송합니다. 따라서 연결을 끊어진다고 갑자기 비디오가 멈추는 위험은 없습니다.
 - 수신기는 발신기의 범위내에 있는한 비디오를 수신합니다. 서서히 비디오 품질이 떨어지더라도 멈추지는 않도록합니다. 심지어 프레임에 에러가 있더라도 프레임을 버리는 대신에 표시하게 됩니다.
 - 고적적인 개념 "단일 브로드캐스터 - 멀티 수신기"는 바로 동작합니다. 만약 주변 사람이 그들의 장치로 비디오 스트림을 보고자 한다면 "채널만 제대로 맞추면" 가능합니다.
 - wifibroadcast는 병렬로 저렴한 수신기를 사용할 수 있게하며 데이터를 결합하여 수신한 데이터의 보정 가능성을 높이도록 합니다. 이런 소위 소프트웨어 다이버시티는 보완 리시버뿐만 아니라 신뢰성을 높이기 위해 동일한 수신기를 사용할수 있도록 합니다.(360도를 커버하는 단반향 안테나를 가지는 수신기와 병렬로 동작하는 높은 거리에 있는 여러 개의 방향 안테나를 생각해보세요)
 - wifibroadcast는 낮은 대역폭에서 높은 신뢰성을 달성하기 위해서 FEC(Forward Error Correction)을 사용합니다. 수신부에서 손실되었거나 변형된 패킷을 복구할 수도 있습니다.


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

Q: 전송 보장이란 무엇인가요?

A: Wifibrodcast는 FEC (forward error correction)을 사용해서 기본 셋팅으로 12개 패킷에서 손실된 4개 패킷을 복구할 수 있습니다. 요구에 맞추기 위해서 TX와 RX 동시에 튜닝할 수 있습니다.

Q: 프레임 드롭이 많이 일어나고 ``XX packets lost``를 나옵니다. 이것은 무슨 뜻인가요?

A: 이는 다음과 같은 이유 때문일 수 있습니다:
   1. 단일 전원은 매우 납습니다. 높은 전원 카드를 사용하거나 게인이 높은 안테나를 이용하세요. RX쪽에 직접 안테나를 사용합니다. 다이버시티를 위해 추가 RX 카드를 사용합니다. (wlan2, wlan3, ... 를 rx 프로그램에 추가)
   2. 신호 세기가 너무 높습니다. 인도어 30dBm TX를 사용하고 있다면 TX 전원을 낮춰보세요.(예로 커널 내부의 CRDA 데이터베이스를 해킹해서 전원 제약을 10dBm와 20dBm와 같이 여러 가지로 바꿔봅니다.)
   3. 다른 WiFi와 간섭. WIFI 채널이나 WIFI 밴드를 변경해 보세요. 주의: RC TX가 동작 중이 경우 밴드를 사용하지 마세요! 아니면 RTL을 적절하게 셋업해서 모델 손실을 회피해보세요. FEC block 사이즈를 증가해 보세요(기본적으로 8/12 (8 데이터 블록과 4 fec 블록)) 하지만 이렇게 되면 지연시간이 길어질 것입니다. 다이버시티를(wlan2, wlan3, ... 를 rx 프로그램에 추가) 위해서 추가 RX 카드를 사용하세요.

## TODO
1. 다른 카드/안테나로 비행 테스트를 해보세요.
2. CRDA 해킹하지 않고 TX 파워를 설정하는 방법을 찾아보세요.
3. FEC를 튜닝해서 지연/중복을 최적화시키세요.

## Wifi Cards:

다음 Atheros 칩셋은 동작합니다:

 -  Atheros AR9271, Atheros AR9280, Atheros AR9287

다음 Ralink 칩셋은 동작합니다:

 -  RT2070, RT2770, RT2870, RT3070, RT3071, RT3072, RT3370, RT3572, RT5370, RT5372, RT5572

하지만 카드가 동작하지 않는 자잘한 문제가 있을 수 있습니다. 따라서 안전하게 하려면 다른 사람들로부터 검증된 카드 중에 하나를 선택합니다:

 -  CSL 300Mbit Stick (2.4/5Ghz, Diversity, RT5572 chipset)
 -  Alfa AWUS036NHA (2.3/2.4Ghz, high power, Atheros AR9271 chipset)
 -  TPLink TL-WN722N (2.3/2.4Ghz, Atheros AR9271 chipset)
 -  ALFA AWUS051NH v2 (2.4Ghz/5Ghz, high power, Ralink RT3572 chipset)
 -  ALFA AWUS052NH v2 (2.4Ghz/5Ghz, Diversity, high power, Ralink chipset)
 -  TP-Link-TL-WDN3200 (2.4/5Ghz, Diversity, RT5572 chipset)
 -  Ralink RT5572 (2.4/5Ghz, Diversity???, RT5572 chipset)

반면에 만약에 모든 사람이 동일한 카드를 갖고 있다면 어느 것이 동작하는지 찾기가 어려울 것입니다. 중국에는 4$ 이하의 가격으로 매우 작고 가벼운 RT5370 카드도 있습니다. 예로 Aliexpress에는 저렴하고 다양한 Wifi 카드가 있습니다. 여기 목록에 없는 wifi 카드로 시도해 본 경험을 리포팅해주면 좋겠습니다.

* AWUS036NHA 이 아답터는 280mW 출력 파워를 제공합니다. 수 km 범위라고 알려져 있습니다.(지향성 안테나 사용)

* TL-WN722N 이 아답터는 60mw 출력 파워를 제공합니다. 범위는 대략 800-1000m 정도로 2.1dbi 안테나를 가지고 있습니다. 중요: 특정 환경에서 PCB에 있는 두번째 안테나의 성능이 나쁘게 나타납니다. 아래와 같이 PCB 뒷면에 PCB 흰색 컴포넌트르르 제거해서 안테나를 제거해주세요.(사진에서 컴포넌트는 필요에 따라서 mod를 뒤집을 수 있도록 상단 패드에 납떔이 되어있음)

* CSL 300Mbit stick 이 아답터는 대략 30mw 출력 파워를 제공합니다. 5Ghz 범위로 대략 200-300m로 매우 높지는 않습니다. stock 안테나는 5Ghz에서 사용할 수 없습니다. 단순히 2.4Ghz 2.1dbi sleeved-diploe 안테나입니다.

Rx 동글을 사용할 때, 수신 신호 세기가 -20dbm보다 높은 경우 배드 블록이 발생할 수 있습니다. 이는 1개 이상 아답터를 사용하거나 다른 방향이나 극성으로 안테나를 향하게 해서 회피할 수 있습니다.

* AWUS051NH 이 아답터는 330mw 출력 파워를 제공합니다. 5Ghz로 대략 800-1000m정도 입니다. stock 안테나를 추천하지 않는데 왜냐하면 5dbi 게인을 가지고 있기 때문입니다. 이는 too-flat radiation 패턴을 가지기 때문입니다.

* AWUS052NH 이 아답터는 330mw 출력 파워를 제공합니다. 051NH와 동일한 아답터지만 2개의 TX 체인을 가지고 있습니다. stock 안테나를 추천하지 않는데 왜냐하면 게인이 5dbi로 too-flat raidation 패턴을 가지고 있기 때문입니다.

## Links:
 - wifibroadcast의 [원본 버전](https://befinitiv.wordpress.com/wifibroadcast-analog-like-transmission-of-live-video-data/)
