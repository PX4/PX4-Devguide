# Logging

여기서는 새로운 logger에 대해서 알아봅니다. `SYS_LOGGER`를 1로 설정합니다.

logger는 포함하는 모든 필드와 함께 ORB topic을 기록할 수 있습니다. 모든 것은 `.msg` 파일에서 생성되며 topic 이름은 지정해줘야 합니다. 선택적으로 interval 파라미터는 특정 topic에 대해서 최대 logging rate를 지정합니다. 모든 기존 topic 인스턴스는 로그를 남기게 됩니다.

출력 log 포맷은 [ULog](../log/ulog_file_format.md)입니다.

## 사용
기본적으로 logging은 arming이 되면 자동으로 시작되고 disarming이 되면 멈추게 됩니다. 매번 arming 세션마다 새로운 log 파일이 SD카드에 생성됩니다. 현재 상태 보기 위해서는 콘솔에서 `logger status`를 사용하세요. 즉시 logging을 시작시키고자 한다면 `logger on`를 사용합니다. 이렇게하면 시스템이 이미 arm 상태가 된 것처럼 arming 상태를 무시하게 됩니다. `logger off`를 하면 원상태로 돌아갑니다.

Use
```
logger help
```
지원하는 모든 logger 명령과 파라미터의 목록을 보여줍니다.


## 설정

로그로 기록되는 topic의 목록은 SD카드에 파일로 커스터마이즈될 수 있습니다. topic의 목록과 함께 카드에 `etc/logging/logger_topics.txt` 파일을 생성합니다:
```
<topic_name>, <interval>
```
`<interval>`은 선택사항이며 만약 지정하면 해당 topic이 2개 로그 메시지를 기록하는 사이의 시간 구간을 ms 단위로 정의합니다. 만약 지정하지 않으면 topic은 최대 rate로 로그를 기록합니다.

이 파일에 있는 해당 topic들은 모두 디폴트 로그의 topic으로 대체됩니다.

## Scripts
[pyulog](https://github.com/PX4/pyulog) 저장소에서 로깅 파일을 분석하고 변환하는 여러 스크립트가 있습니다.

## 손실 (Dropouts)
로깅 손실이 발생하는 것은 바람직하지 않으면 손실되는 양에 영향을 미치는 몇 가지 요소들이 있습니다. :
- 테스트한 대부분 SD카드는 분당 여러번 멈추는 현상이 있었습니다. write 명령 동안 여러 번 100ms delay가 있습니다. writer buffer가 이 시간동안 채워지면 드롭아웃이 발생합니다. 이런 현상은 SD카드에 따라 발생합니다.(아래 참조)
- SD 카드를 포맷하면 드롭아웃을 예방하는데 도움이 될 수도 있습니다.
- 로그 버퍼를 증가도 도움이 됩니다.
- 선택한 topic의 로깅 rate를 줄이거나 로깅된 불필요한 topic을 삭제합니다. (이런 경우 `info.py <file>`가 유용)

## SD Cards
SD 카드별로 성능 결과를 알아봅니다.
테스트는 Pixracer에서 진행했고 이 결과는 Pixhawk에서도 동일하게 적용됩니다.

| SD Card | 평균 Seq. 쓰기 속도 [KB/s] | 최대 쓰기 시간 / 블록 (평균) [ms] |
| -- | -- | -- |
| SanDisk Extreme U3 32GB | 461 | **15** |
| Sandisk Ultra Class 10 8GB | 348 | 40 |
| Sandisk Class 4 8GB | 212 | 60 |
| SanDisk Class 10 32 GB (높은 지속성의 비디오 모니터링 카드) | 331 | 220 |
| Lexar U1 (Class 10), 16GB High-Performance | 209 | 150 |
| Sandisk Ultra PLUS Class 10 16GB | 196 | 500 |
| Sandisk Pixtor Class 10 16GB | 334 | 250 |
| Sandisk Extreme PLUS Class 10 32GB | 332 | 150 |

평균 쓰기 속도보다 더 중요한 것은 바로 블록당(4 KB) 최대 쓰기 시간입니다. 이것으로 최소 버퍼 크기를 정의합니다. : 최대 쓰기 시간이 클수록, log buffer가 커질수록 드롭아웃 회피가 필요하게 됩니다. 기본 topic의 logging bandwidth는 대략 50 KB/s로 SD 카드들 모두 이를 만족시킵니다.

 **SanDisk Extreme U3 32GB** 가 가장 좋은 카드로 보입니다. 이 카드를 추천하는 이유는 쓰기 시간 편차가 없어서 이론상 드롭아웃이 발생하지 않습니다. 다른 카드들도 잘 동작하지만 각자 성능에 편차가 있습니다.

 여러분이 가지고 있는 SD 카드를 `sd_bench -r 50`로 테스트해서 결과를 https://github.com/PX4/Firmware/issues/4634 로 보내주세요.

## Log Streaming
로깅을 하는데 있어 지금까지 전통적으로 지원하는 방식은 FMU에 SD 카드를 사용하는 방법입니다. 하지만 log streaming이라는 다른 대안이 있습니다. log streaming은 MAVLink를 통해 동일한 logging data를 보내는 것입니다. 이 방법은 FMU에 SD 카드 슬롯이 없는 경우(Intel Aero)나 SD 카드로 처리하는 것을 피하고 싶을때 사용할 수 있습니다. 두가지 방법 동시에 독립적으로 사용이 가능합니다.

요구사항은 연결시 속도가 최소한 50KB/s 이상이 되어야 합니다. WiFi 연결가 좋은 예제입니다. 그리고 client 하나만 log streaming을 요청할 수 있습니다. 연결이 안정적이지 않다면, 해당 프로토콜이 드롭을 처리할 수 있도록 설계해야 합니다.

ulog streaming을 지원하는 여러 client들 :
- `mavlink_ulog_streaming.py` script in Firmware/Tools.
- QGroundControl:
![](../../assets/gcs/qgc-log-streaming.png)
- [MAVGCL](https://github.com/ecmnet/MAVGCL)

### Diagnostics
- 만약 log streaming 실행이 되지 않는다면, `logger`가 실행 중인지 확인하고 시작하는 동안 콘솔 출력값을 조사합니다.
- 여전히 동작하지 않는다면 Mavlink 2를 사용하고 있는지 확인합니다. `MAV_PROTO_VER`를 2로 강제로 설정합니다.  
- log streaming은 mavlink rate의 최대 70%을 사용합니다.(`-r` 인자) 더 필요하게 되면 메시지는 드롭됩니다. 현재 사용중인 퍼센트를 `mavlink status` 를 이용해서 감시할 수 있습니다. (이 예제에서는 1.8%로 사용) :
```
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
        transport protocol: UDP (14556)
```
  `txerr`가 0으로 되어 있는지 확인합니다. 만약 이 값이 올라가면 Nuttx의 sending buffer가 너무 작거나 물리적인 연결이 포화상태거나 하드웨어가 데이터를 처리하기에 성능이 느린 것일 수 있습니다.
