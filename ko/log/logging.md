# 로깅

로거 프로그램은 어떤 ORB 주제에 대해서든 모든 내용을 넣어 기록할 수 있습니다. 결국 우리가 필요한 것은 `.msg` 파일이기에 토픽 이름만 정하면 됩니다. 추가 주기 매개변수에는 각 토픽별 최대 기록 주기를 지정합니다. 모든 토픽의 실존 인스턴스를 기록합니다.

출력 로그 형식은 [ULog](../log/ulog_file_format.md) 입니다.

## 사용법

기본적으로, 로깅은 이륙 준비를 마쳤을 때 시작하며, 이륙 준비를 해제할 때 멈춥니다. 매회 이륙 준비 세션당 새 로그 파일은 SD 카드에 만듭니다. 현재 상태를 표시하려면 콘솔에서 `logger status`명령을 활용하십시오. 로깅을 바로 시작하고 싶다면 `logger on` 명령을 내리십시오. 이 명령으로 이륙 준비 상태를 마쳤을 때, 그 이전 상태에 관계없이 재개합니다. `logger off` 명령은 그 반대입니다.

다음 명령을 활용

    logger help
    

하여 모든 지원 로거 명령과 매개변수 목록을 확인하십시오.

## 설정

The list of logged topics can be customized with a file on the SD card. Create a file `etc/logging/logger_topics.txt` on the card with a list of topics (For SITL, it's `build/px4_sitl_default/tmp/rootfs/fs/microsd/etc/logging/logger_topics.txt`):

    <topic_name> <interval> <instance>
    

The `<interval>` is optional, and if specified, defines the minimum interval in ms between two logged messages of this topic. If not specified, the topic is logged at full rate.

The `<instance>` is optional, and if specified, defines the instance to log. If not specified, all instances of the topic are logged. To specify `<instance>`, `<interval>` must be specified. It can be set to 0 to log at full rate

The topics in this file replace all of the default logged topics.

예시 :

    sensor_accel 0 0
    sensor_accel 100 1
    sensor_gyro 200
    sensor_mag 200 1
    

This configuration will log sensor_accel 0 at full rate, sensor_accel 1 at 10Hz, all sensor_gyro instances at 5Hz and sensor_mag 1 at 5Hz.

## Scripts

There are several scripts to analyze and convert logging files in the [pyulog](https://github.com/PX4/pyulog) repository.

## Dropouts

Logging dropouts are undesired and there are a few factors that influence the amount of dropouts:

- Most SD cards we tested exhibit multiple pauses per minute. This shows itself as a several 100 ms delay during a write command. It causes a dropout if the write buffer fills up during this time. This effect depends on the SD card (see below).
- Formatting an SD card can help to prevent dropouts.
- Increasing the log buffer helps.
- Decrease the logging rate of selected topics or remove unneeded topics from being logged (`info.py <file>` is useful for this).

## SD 카드

The following provides performance results for different SD cards. Tests were done on a Pixracer; the results are applicable to Pixhawk as well.

> **Tip** The maximum supported SD card size for NuttX is 32GB (SD Memory Card Specifications Version 2.0).

| SD 카드                                                         | 평균 시퀀스 기록 속도 [KB/s] | 최대 기록 시간 / 블록 (평균) [ms] |
| ------------------------------------------------------------- | ------------------- | ----------------------- |
| SanDisk Extreme U3 32GB                                       | 461                 | **15**                  |
| Sandisk Ultra Class 10 8GB                                    | 348                 | 40                      |
| Sandisk Class 4 8GB                                           | 212                 | 60                      |
| SanDisk Class 10 32 GB (High Endurance Video Monitoring Card) | 331                 | 220                     |
| Lexar U1 (Class 10), 16GB High-Performance                    | 209                 | 150                     |
| Sandisk Ultra PLUS Class 10 16GB                              | 196                 | 500                     |
| Sandisk Pixtor Class 10 16GB                                  | 334                 | 250                     |
| Sandisk Extreme PLUS Class 10 32GB                            | 332                 | 150                     |

More important than the mean write speed is the maximum write time per block (of 4 KB). This defines the minimum buffer size: the larger this maximum, the larger the log buffer needs to be to avoid dropouts. Logging bandwidth with the default topics is around 50 KB/s, which all of the SD cards satisfy.

By far the best card we know so far is the **SanDisk Extreme U3 32GB**. This card is recommended, because it does not exhibit write time spikes (and thus virtually no dropouts). Different card sizes might work equally well, but the performance is usually different.

You can test your own SD card with `sd_bench -r 50`, and report the results to https://github.com/PX4/Firmware/issues/4634.

## 로그 스트리밍

태로초부터 지금까지 제대로 지원하는 로그 스트리밍 방법은 FMU에서 SD 카드를 활용하는 방법입니다. 그러나, 이 대안책으로, MAVLink로 동일한 로깅 데이터를 전송하는 로깅 스트리밍 방식이 있습니다. 이 방식은 FMU에 SD 카드 슬롯이 없을 경우(예: Intel® Aero Ready to Fly 드론)에 활용하거나, 단순히 SD 카드의 취급을 피하려 하고자 할 경우 진행할 수 있습니다. 두가지 방식은 동시에 제각각 활용할 수 있습니다.

필요 요소는 WiFi 연결 처럼 초당 50KB를 제공할 수 있는 통신 수단입니다. 단일 클라이언트만 동시에 로그 스트리밍을 요청할 수 있습니다. 프로토콜에서 손실 패킷을 관리하기에 연결은 굳이 안정적이지 않아도 좋습니다.

ulog 스트리밍을 지원하는 클라이언트는 여러가지가 있습니다.:

- Firmware/Tools의 `mavlink_ulog_streaming.py`
- QGroundControl:![QGC 로그 스트리밍](../../assets/gcs/qgc-log-streaming.png)
- [MAVGCL](https://github.com/ecmnet/MAVGCL)

### 진단

- 로그 스트리밍을 시작하지 않았다면, `logger`를 실행 중인지(위 참고) 확인하고, 시작하는 동안 콘솔 출력을 살펴보십시오.
- 여전히 동작하지 않는다면 MAVLink 2를 사용하고 있는지 확인하십시오. `MAV_PROTO_VER` 매개변수 값을 2로 강제 설정하십시오.
- 로그 스트리밍은 MAVLink 전송율 설정(`-r` 매개변수) 최대값의 70% 전송율을 보입니다. 더 큰 전송율이 필요하다면, 메세지가 사라집니다. 현재 MAVLink 패킷에서 로그가 차지하는 백분율은 `mavlink status` 명령으로 확인할 수 있습니다(이 예제에서는 1.8%). 
        instance #0:
              GCS heartbeat:  160955 us ago
              mavlink chan: #0
              type:           GENERIC LINK OR RADIO
              flow control:   OFF
              rates:
              tx: 95.781 kB/s
              txerr: 0.000 kB/s
              rx: 0.021 kB/s
              rate mult: 1.000
              ULog rate: 1.8% of max 70.0%
              accepting commands: YES
              MAVLink version: 2
              transport protocol: UDP (14556) 또한 
    
    `txerr` 값이 0에 머물러있는지 확인하십시오. 만약 이 값이 올라간다면, NuttX 전송 버퍼가 너무 작거나, 물리 링크 대역폭이 포화 상태이거나, 하드웨어가 데이터를 처리하기에 너무 느린 상황임을 의미합니다.