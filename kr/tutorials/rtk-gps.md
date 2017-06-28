# UBlox M8P RTK GPS 설정

RTK(Real Time Kinematic)은 GPS 정확도를 센티미터 단위까지 높입니다. 시그널의 정보 내용보다는 시그널 캐리어 웨이브의 phase 측정을 사용하고 실시간 보정을 위해서 단일 레퍼런스 스테이션에 기반합니다.

PX4는 현재 RTK에 대해서 GNSS 리시버 기반으로 단일 주파수(L1) UBlox M8P만 지원합니다.

### 동작

PX4와 RKT 셋업을 위해서 2개 M8P GPS 모듈(아래 셋업 예제)과 데이터링크가 필요합니다. 지상에 있는 유닛(고정 위치)을 베이스(base)라고 부르고 공중에 있는 유닛을 로버(rover)라고 부릅니다. 베이스 유닛은 QGroundControl에 연결하고 비행체로 데이터링크를 사용해서 RTCM 보정 정보를 전달합니다.(MAVLink `GPS_RTCM_DATA` 메시지) autopilot에서 MAVLink 패킷을 풀어서 RTK 솔루션을 얻는 처리를 하는 airbone GNSS 유닛으로 전달합니다.

데이터링크는 일반적으로 업링크시 초당 300 바이트까지 처리가 가능합니다. 아래 업링크 datarate 섹션을 참고하세요.

### 자동 설정

QGroundControl과 autopilot 펌웨어는 동일한 [PX4 GPS driver stack](https://github.com/PX4/GpsDrivers)을 공유합니다. 실제로 새로운 프로토콜이나 메시지를 지원하기 위해서 한쪽에만 추가하면 된다는 뜻입니다.

PX4 GPS 스택은 자동으로 UBlox M8P 모듈을 셋업해서 UART나 USB를 통해서 보정 메시지를 주고 받습니다. U-Center를 사용한 설정이 필요하지 않습니다.

> **Note** UBlox은 2가지 M8P칩인 M8P-0와 M8P-2가 있습니다.
> M8P-0은 로버로만 사용이 가능합니다. 반면에 M8P-2은
> 로버와 베이스로도 사용이 가능합니다.

### RTCM 메시지

QGroundControl은 RTK 베이스 스테이션을 다음의 RTCM3.2 프레임을 출력하도록 설정합니다 :
- **1005** - Station coordinates XYZ for antenna reference point. (Base position.)
- **1077** - Full GPS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)
- **1087** - Full GLONASS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)
- **1127** -  Full BeiDou pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)
- **1097** - Full Galileo pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)
**Only supported with M8P Firmware Version 3.01 and above. See section below on firmware updates.**
- **1127** -  Full BeiDou pseudo-ranges, carrier phases, Doppler and signal strength (high resolution.)

**TODO : QGC Galileo(와 BeiDou?) 메시지 설정이 없음 -- 업데이트 요망**

### 업링크 Datarate

base로부터의 raw RTCM 메시지는 `GPS_RTCM_DATA` MAVLink 메시지로 패킹해서 데이터링크로 전송됩니다. 각 MAVLink 메시지의 길이는 182 바이트로 RTCM 메시지를 내부에 인캡슐레이션합니다. RTCM 메시지에 의존함으로 MAVLink 메시지가 완전히 차게되는 경우는 거의 없습니다.

Base Position 메시지 (1005)는 22 byte 길이이며 반면에 다른 것들은 모두 가변길이로 위성 여부나 위성으로부터 신호의 갯수(M8P와 같이 L1 유닛에 대해서 오직 1)에 따라 달라집니다. 단일 constellation에서 확인할 수 있는 위성의 _최대_ 갯수는 12개로 RTCM 메시지는 각각 대략 120 byte가 됩니다. 실제 조건에서 300 bps 업링크 rate는 이론상 충분합니다.

**MAVLink 1**이 사용되면 어떠한 패킷 끊어짐도 일어나지 않습니다. 따라서 전체 182-byte `GPS_RTCM_DATA` 메시지가 매번 RTCM 메시지를 위해 전송됩니다. 이말은 대략 업링크 요구가 초당 700+ 바이트까지 늘어나게 됩니다. 이렇게 되면 3DR 라디오와 같이 낮은 대역폭의 half-duplex 텔레메트리 모듈의 경우 링크 포화상태가 됩니다.

**MAVLink 2**가 사용되면(GCS와 텔레메트리 모듈이 이를 지원한다면 PX4는 자동으로 MAVLink 2 전환), 패킷의 빈 공간이 줄어들게되어 초당 ~300 바이트 업링크 요구로 가벼워집니다. MAVLink 2가 이 링크에서 좋은 RTK 성능을 내는 것이 **핵심**입니다. 따라서 텔레메트리 체인이 MAVLink 2 전체를 사용하는지 주의해야 신경써야 합니다. 시스템 콘솔에서 `mavlink status` 명령을 이용해서 프로토콜 버전을 검증할 수 있습니다. :

```
nsh> mavlink status
instance #0:
        GCS heartbeat:  593486 us ago
        mavlink chan: #0
        type:           3DR RADIO
        rssi:           219
        remote rssi:    219
        txbuf:          94
        noise:          61
        remote noise:   58
        rx errors:      0
        fixed:          0
        flow control:   ON
        rates:
        tx: 1.285 kB/s
        txerr: 0.000 kB/s
        rx: 0.021 kB/s
        rate mult: 0.366
        accepting commands: YES
        **MAVLink version: 2**
        transport protocol: serial (/dev/ttyS1 @57600)
```

### 펌웨어 업데이트

todo

## Drotek XL RTK 예제

온라인 구매 : [Drotek XL RTK](https://drotek.com/shop/en/home/792-xl-rtk-gps-neo-m8p-rover.html)

![](../../assets/drotek_rtk_base.jpg)

![](../../assets/drotek_rtk_rover.jpg)

## HEX/ProfiCNC Here+ 예제

온라인 구매 : [Here+ RTK GNSS](http://www.hex.aero/shop/all/here-rtk-gnss-set/)
