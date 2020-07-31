# RTK GPS (백그라운드)

[실시간 키네매틱](https://en.wikipedia.org/wiki/Real_Time_Kinematic) (RTK) 에서는 센티미터 단위의 GPS 정확도를 확보해줍니다. 이 페이지에서는 실시간 키네메틱 기능을 PX4에 결합하는 방법을 설명합니다.

> **참고** RTK GPS *사용* 방법은 [PX4 사용자 안내서](https://docs.px4.io/master/en/advanced_features/rtk-gps.html)에 있습니다.

## 개요

실시간 키네매틱(RTK)은 시그널 정보 내용 그 자체를 활용하지 않고, 시그널 캐리어 전파의 파장 측정 방식을 활용합니다. 단일 참조 스테이션에 의존하여 다중 모바일 스테이션을 대상으로 동작할 수 있는 실시간 연결을 제공합니다.

PX4에 실시간 키네매틱(RTK)을 설정하려면, RTK GPS 모듈 두 개와 데이터 링크가 필요합니다. 고정 위치 대지 기반 GPS 유닛을 *베이스* 라 하고, 공중에 띄우는 유닛을 *로버(Rover)*라 합니다. 베이스 유닛은 (USB로) *QGroundControl*에 연결하고, 비행체로의 RTCM 연결 데이터를 지속적으로 송수신하는 데이터 링크를 활용합니다(MAVLink [GPS_RTCM_DATA](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) 메시지 활용). autopilot에서는 MAVLink 패킷을 패키징 해제한 후 RTK 솔루션 획득을 처리할 수 있는 로버 유닛에 보냅니다.

데이터링크는 보통 초당 300바이트 전송을 처리할 수 있어야합니다(더 많은 정보는 [상위 링크 데이터 수신율](#uplink-datarate) 부분을 참고하십시오).

## 지원하는 RTK GPS 모듈

PX4는 단일 주파(L1) u-blox M8P 기반 GNSS 수신기만을 RTK 기능용으로 지원합니다.

많은 제조사에서 이 수신기로 제품을 만들고있습니다. 커뮤니티에서 시험한 장치 목록은 [사용자 안내서](https://docs.px4.io/master/en/gps_compass/rtk_gps.html#supported-rtk-devices)에서 찾아볼 수 있습니다. 

> **참고** u-blox는 두가지 M8P칩 모델, M8P-0과 M8P-2의 사용 여부에 따라 다릅니다. M8P-0칩을 장착한 모델은 베이스가 아닌 로버용으로만 사용할 수 있으나, M8P-2칩 장착 모델은 로버용, 베이스용 둘 다 활용 가능합니다.

## 자동 설정

PX4 GPS 스택은 u-blox M8P 모듈을 자동으로 설정하여 UART 또는 USB 둘 중 어떤 미디움을 통해 (*QGroundControl* 또는 autopilot에) 모듈을 연결했느냐에 따라 올바른 메시지를 주고 받을 수 있게 합니다.

autopilot에서 `GPS_RTCM_DATA` MAVLink 메시지를 받는 즉시, RTCM 데이터를 GPS 모듈에 자동으로 전달합니다.

> **참고** U-Center RTK 모듈 설정 도구는 필요하지도 않고 사용하지도 않습니다!

<span></span>

> **Note** *QGroundControl*과 autopilot 펌웨어는 동일한 [PX4 GPS 드라이버 스택](https://github.com/PX4/GpsDrivers)을 공유합니다. 실제로, 새 프로토콜 또는 메시지 지원시 한쪽에만 추가하면 됩니다.

### RTCM 메시지

QGroundControl 은 RTK 베이스 스테이션을 설정하여 다음 RTCM3.2 프레임을 1초에 한번씩 출력합니다:

- **1005** - 안테나 참조 지점 값인 스테이션 좌표 XYZ 값(베이스 위치).
- **1077** - 전체 GPS 가상 범위, 캐리어 위상, 도플러 신호 세기 (고해상).
- **1087** - 전체 GLONASS 가상 범위, 캐리어 위상, 도플러 신호 세기 (고해상).

## 상위 링크 데이터 전송율

베이스 포인트에서의 RAW RTCM 메시지는 MAVLink `GPS_RTCM_DATA` 메시지로 포장하여 데이터 링크를 통해 보냅니다. MAVLink 메시지 최대 길이는 182 바이트입니다. RTCM 메시지에 따라 MAVLink 메시지는 거의 대부분 완전히 채울 일이 없습니다.

RTCM 베이스 위치 메시지 (1005)는 22 바이트 길이를 가지며, 다른 메시지는 가시 범위의 위성 숫자와 위성 신호 수(그 중 하나는 M8P와 같은 유닛의 L1 신호입니다)에 따라 다양한 길이를 가집니다. Since at a given time, the *maximum* number of satellites visible from any single constellation is 12, under real-world conditions, theoretically an uplink rate of 300 B/s is sufficient.

If *MAVLink 1* is used, a 182-byte `GPS_RTCM_DATA` message is sent for every RTCM message, irrespective of its length. As a result the approximate uplink requirement is around 700+ bytes per second. This can lead to link saturation on low-bandwidth half-duplex telemetry modules (e.g. 3DR Telemetry Radios).

If *MAVLink 2* is used then any empty space in the `GPS_RTCM_DATA message` is removed. The resulting uplink requirement is about the same as the theoretical value (~300 bytes per second).

> **Tip** PX4 automatically switches to MAVLink 2 if the GCS and telemetry modules support it.

MAVLink 2 must be used on low-bandwidth links for good RTK performance. Care must be taken to make sure that the telemetry chain uses MAVLink 2 throughout. You can verify the protocol version by using the `mavlink status` command on the system console:

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
            MAVLink version: 2
            transport protocol: serial (/dev/ttyS1 @57600)