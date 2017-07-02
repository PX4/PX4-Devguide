# DJI 프레임휠 450 (거리 센서와 RTK GPS 장착)
-------------------------

{% youtube %}https://www.youtube.com/watch?v=JovSwzoTepU{% endyoutube %}

![](../../assets/airframes/multicopter/Flamewheel_450/f450_setup_full.JPG)

![](../../assets/airframes/multicopter/Flamewheel_450/f450_setup_back.JPG)

## 부품 목록
* Autopilot:        [Pixhawk 3 Pro](..//flight_controller/pixhawk3_pro.md)
* 프레임:            DJI Flamewheel 450
* 모터:           3DR Iris Plus 950kv rebranded T-Motors MN2213
* ESCs:             Hobbywing XRotor 35A Micro 3-6S BLHeli
* 블레이드:           Graupner 10"x5" (You need to print [these adapters](https://drive.google.com/open?id=0B2piootk_fIKMWhIVzVPWEFGLU0) to be M6 compatible) <!--TODO-->
* 거리센서:  Lidar-Lite V3
* GPS:              Here+ RTK GPS
* 텔레메트리:        3DR Telemetry
* 배터리:          Roxxy LiPo - 4S, 4000mAh

추가로 FrSky X4R-SB 3/16ch 2.4Ghz 리시버와 FrSky Taranis 컨트롤러를 사용할 수 있습니다. 케이블타이와 양면 테이프, 납땜기, 3D 프린터가 필요합니다. GPS 기둥은 Intel Aero에서 재사용할 수 있습니다.

![](../../assets/airframes/multicopter/Flamewheel_450/f450_setup_open.JPG)

Pixhawk 3 Pro는 IMU쪽이 이미 댐핑처리가 되어 있어서 양면 테잎을 이용해서 붙일 수 있습니다.

여기서 셋업에는 Pixhawk 3 Pro를 180도 돌려서 놨는데 이렇게 하면 SD카드로 쉽게 접근할 수 있습니다. 만약에 밑판을 180도 돌리는 경우 flight controller를 정면을 향하게 마운트시킬 수 있습니다. 어떤 방법이든 상관없습니다. 결국 QGC에서 보드의 회전을 제대로 설정해 주기만 하면 됩니다.


## 결선과 연결

Pixhawk 3용으로 일반적인 핀아웃은 [여기](https://pixhawk.drotek.com/en/inputs-outputs.html)을 참고하세요.

### 3DR 텔레메트리

3DR 텔레메트리는 Pixhawk 3 Pro가 사용하는 JST GH 커넥터가 아닙니다. 핀아웃은 동일하며 플러그만 바꿔주면 됩니다. Pixhawk 3 Pro에 있는 Telem 1 포트를 사용합니다.

| pin | Pixhawk 3 Pro Telem 1 | 3DR 텔레메트리      |
| --- | --------------------- | ---------------- |
| 1   | VCC                   | VCC              |
| 2   | TX                    | RX               |
| 3   | RX                    | TX               |
| 4   | CTS                   | CTS              |
| 5   | RTS                   | RTS              |
| 6   | GND                   | GND              |

### Lidar-Lite V3

Lidar Lite V3용 핀아웃과 Pixhawk 3 Pro I2C 1 포트는 다음과 같습니다.

| pin | Pixhawk 3 Pro I2C 1 | Lidar Lite V3    |
| --- | ------------------- | ---------------- |
| 1   | VCC                 | VCC              |
| 2   | SCL                 | - (Power enable) |
| 3   | SDA                 | - (Mode control) |
| 4   | GND                 | SCL              |
| 5   | -                   | SDA              |
| 6   | -                   | GND              |

### Here+ RTK GPS

Here+ GPS는 Pixhawk 2에 맞는 8 핀 커넥터로 되어 있습니다. Pixhawk 3 Pro (혹은 Pixracer)에 사용하려면 6 핀 커넥터를 사용해야만 하며, 핀 6과 7은 필요하지 않습니다.(아래 참고) 추가 핀은 세이프티 버튼이고 필요에 따라서 추가할 수 있습니다.
핀아웃에 관련된 추가 정보는 [이 문서](http://www.hex.aero/wp-content/uploads/2016/07/DRS_Pixhawk-2-17th-march-2016.pdf)에 17페이지를 참고하세요.

![](../../assets/airframes/multicopter/Flamewheel_450/f450_setup_gps.JPG)

| pin | Here+ GPS     | pin | Pixhawk 3 Pro GPS |
| --- | ------------- | --- | ----------------- |
| 1   | VCC_5V        | 1   | VCC               |
| 2   | GPS_RX        | 2   | GPS_TX            |
| 3   | GPS_TX        | 3   | GPS_RX            |
| 4   | SCL           | 4   | SCL               |
| 5   | SDA           | 5   | SDA               |
| 6   | BUTTON        | -   | -                 |
| 7   | BUTTON_LED    | -   | -                 |
| 8   | GND           | 6   | GND               |

## Parameters

General documentation on how to setup your quad in QGC can be found [here](https://docs.qgroundcontrol.com/en/).

### 에어프레임

`QGC -> Airframe -> Quadrotor x`에서 `DJI Flame Wheel 450` 에어프레임을 선택합니다.

![](../../assets/airframes/multicopter/Flamewheel_450/f450_setup_airframe.png)

### Lidar-Lite

I2C로 연결로 Lidar-Lite V3를 사용하는 경우, 파리미터 `SENS_EN_LL40LS`는 `2`로 설정해야 하빈다. NuttX 쉘을 사용해서 `param set SENS_EN_LL40LS 2` 명령을 줄 수 있습니다. 쉘을 사용하는 방법은 [여기](../debug/sensor_uorb_topic_debugging.md)를 참고하세요. (이것은 현재 임시 방법입니다. 왜냐하면 QGC가 현재 값을 2로 직접 바꾸는 것을 제공하지 않기 때문입니다.)

### RTK GPS

RTK GPS는 플러그앤플레이로 동작합니다. 추가 정보는 [여기](https://docs.px4.io/en/advanced_features/rtk-gps.html)를 참고하세요.


### 기타
다음 파라미터들을 설정 :
- `EKF2_HGT_MODE=2`: 이렇게 하면 Lidar-Lite에서 얻은 높이 정보를 사용
- `MAV_PROTO_VER=2`: Mavlink protocol version 2 사용
- `CBRK_IO_SAFETY=22027`: 세이프티 버튼을 비활성화
- `EKF2_GPS_POS_X`, `EKF2_GPS_POS_Y`, `EKF2_GPS_POS_Z`: 해당 보드에(NED coordinates) 따라서 GPS 장치 offset 설정
