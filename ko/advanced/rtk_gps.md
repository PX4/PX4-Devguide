# RTK GPS (백그라운드)

[실시간 키네매틱](https://en.wikipedia.org/wiki/Real_Time_Kinematic) (RTK) 에서는 센티미터 단위의 GPS 정확도를 확보해줍니다. 이 페이지에서는 실시간 키네메틱 기능을 PX4에 결합하는 방법을 설명합니다.

> **참고** RTK GPS *사용* 방법은 [PX4 사용자 안내서](https://docs.px4.io/master/en/advanced_features/rtk-gps.html)에 있습니다.

## 개요

실시간 키네매틱(RTK)은 시그널 정보 내용 그 자체를 활용하지 않고, 시그널 캐리어 전파의 파장 측정 방식을 활용합니다. 단일 참조 스테이션에 의존하여 다중 모바일 스테이션을 대상으로 동작할 수 있는 실시간 연결을 제공합니다.

PX4에 실시간 키네매틱(RTK)을 설정하려면, RTK GPS 모듈 두 개와 데이터 링크가 필요합니다. 고정 위치 대지 기반 GPS 유닛을 *베이스* 라 하고, 공중에 띄우는 유닛을 *로버(Rover)*라 합니다. 베이스 유닛은 (USB로) *QGroundControl*에 연결하고, 비행체로의 RTCM 연결 데이터를 지속적으로 송수신하는 데이터 링크를 활용합니다(MAVLink [GPS_RTCM_DATA](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) 메시지 활용). 자동 운항 과정에서, MAVLink 패킷을 패키징 해제한 후, RTK 솔루션 획득을 처리할 수 있는 로버 유닛에 보냅니다.

The datalink should typically be able to handle an uplink rate of 300 bytes per second (see the [Uplink Datarate](#uplink-datarate) section below for more information).

## Supported RTK GPS modules

PX4 currently only supports the single-frequency (L1) u-blox M8P based GNSS receivers for RTK.

A number of manufacturers have created products using this receiver. The list of devices that we have tested can be found [in the user guide](https://docs.px4.io/master/en/gps_compass/rtk_gps.html#supported-rtk-devices).

> **Note** u-blox has two variants of the M8P chip, the M8P-0 and the M8P-2. The M8P-0 can only be used as Rover, not as Base, whereas the M8P-2 can be used both as Rover or as Base.

## Automatic Configuration

The PX4 GPS stack automatically sets up the u-blox M8P modules to send and receive the correct messages over the UART or USB, depending on where the module is connected (to *QGroundControl* or the autopilot).

As soon as the autopilot receives `GPS_RTCM_DATA` MAVLink messages, it automatically forwards the RTCM data to the attached GPS module.

> **Note** The U-Center RTK module configuration tool is not needed/used!

<span></span>

> **Note** Both *QGroundControl* and the autopilot firmware share the same [PX4 GPS driver stack](https://github.com/PX4/GpsDrivers). In practice, this means that support for new protocols and/or messages only need to be added to one place.

### RTCM messages

QGroundControl configures the RTK base station to output the following RTCM3.2 frames, each with 1 Hz:

- **1005** - Station coordinates XYZ for antenna reference point (Base position).
- **1077** - Full GPS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).
- **1087** - Full GLONASS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).

## Uplink datarate

The raw RTCM messages from the base are packed into a MAVLink `GPS_RTCM_DATA` message and sent over the datalink. The maximum length of each MAVLink message is 182 bytes. Depending on the RTCM message, the MAVLink message is almost never completely filled.

The RTCM Base Position message (1005) is of length 22 bytes, while the others are all of variable length depending on the number of visible satellites and the number of signals from the satellite (only 1 for L1 units like M8P). Since at a given time, the *maximum* number of satellites visible from any single constellation is 12, under real-world conditions, theoretically an uplink rate of 300 B/s is sufficient.

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