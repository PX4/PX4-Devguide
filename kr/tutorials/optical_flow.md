# Optical Flow
position estimation을 위해서 아래를 향하는 카메라와 거리 센서를 사용합니다. Optical Flow 기반 네비게이션은 모두 3개 estimator에서 지원합니다. : EKF2, LPE 그리고 INAV (아래 참고)

## 셋업
위에서 언급한 것처럼 Optical Flow 셋업에는 아래를 향하는 카메라와 거리 센서(LiDAR 선호)가 필요합니다. 각각은 [`OPTICAL_FLOW_RAD` topic](http://mavlink.org/messages/common#OPTICAL_FLOW_RAD) 메시지와 [`DISANCE_SENSOR` topic](http://mavlink.org/messages/common#DISTANCE_SENSOR) 메시지를 publish합니다.

출력은 다음과 같습니다

| MAV의 움직이는 방향 | Integrated flow |
| -- | -- |
| Forwards | + Y |
| Backwards | - Y |
| Right | - X |
| Left | + X |

그리고 순전히 회전을 위해서 integraded_xgyro와 integraded_x(각각 integraded_ygyro와 integraded_y)는 동일해야만 합니다.

예제 셋업은 PX4Flow와 LIDAR-Lite입니다.(사진 참고)

![](../../assets/hardware/optical_flow/flow_lidar_attached.jpg)

### 카메라

#### PX4Flow
optical flow를 계산하는 가장 쉬운 방법은 PX4Flow 보드를 사용하는 것입니다. PX4Flow 보드를 사용하기 위해서 I2C에 연결하기만 하면 됩니다. 추천 마운팅 방법은 Sonar 쪽이 전면을 향하게 하는 것입니다.(이미지 참고) 이 설정에서 `SENS_FLOW_ROT` 파라미터는 270 도입니다.(기본값) PX4Flow 보드가 흔들리지 않도록 합니다.

![](../../assets/hardware/optical_flow/px4flowalignwithpixhawk.jpg)

##### Custom I2C address
The default I2C address of the PX4Flow is 0x42, but it can be incremented using the three solder jumpers labeled "I2C BUS ADDR" on the picture above. This is useful if another device has the same address.
The address increment is equal to the 3-bit value encoded by the jumpers. For example if jumper 0 and 1 are soldered and jumper 2 is unsoldered, the address is incremented by 1\*1 + 1\*2 + 0\*4 = 3, which gives address 0x45.
If all jumpers are unsoldered, the camera will be automatically discovered by the autopilot firmware.
If you modify the I2C address of the PX4Flow, make sure to start the PX4 driver with the correct address:
```
px4flow start                  # address=0x42 (default)
px4flow stop
px4flow start -a 0x45          # address=0x45
```

##### 렌즈 초점 맞추기
양질의 optical flow 정보를 얻기 위해서, PX4Flow에 카메라가 비행의 원하는 높이에서 초점이 맞도록 해야합니다. 카메라 초점을 맞추기 위해서 글자가 있는 물체(책)을 놓고 PX4Flow에 USB를 꽂고 QGroundControl를 실행합니다. 셋팅메뉴에서 PX4Flow를 선택하고 카메라 이미지가 나옵니다. 나사를 돌려서 렌즈 초점을 맞춥니다.

**Note: 3m 이상을 날리는 경우 카메라는 무한대로 초점이 되며 더 높은 비행에 대해서 변경할 필요가 없습니다 **

![](../../assets/flow/flow_focus_book.png)

*Figure: 책을 이용해서 비행을 원하는 높이에서 flow 카메라 초점을 맞춥니다. 일반적으로 1-3 미터. 3미터가 넘으면 카메라는 무한대로 초점이 되므로 이상의 높이에서 동작합니다*

![](../../assets/flow/flow_focusing.png)

*Figure: QGroundControl에서 px4flow 인터페이스는 카메라 초점을 맞추는데 사용*

#### 다른 카메라
통합 카메라를 가진 보드/쿼드를 사용하는 것이 가능합니다.(Bebop2, Snapdragon Flight) [Optical Flow repo](https://github.com/PX4/OpticalFlow)에 대해서 사용 가능. ([snap_cam](https://github.com/PX4/snap_cam) 참고)

### Range Finder
일정하고 정확함을 위해 Sonar에서 LIDAR를 사용하는 것을 추천합니다. [LIDAR-Lite](https://pixhawk.org/peripherals/rangefinder)를 사용할 수 있습니다.

## Estimators

### Extended Kalman Filter (EKF2)
EKF2 estimator를 사용하기 위해서 `SYS_MC_EST_GROUP` 파라미터가 `2`로 설정하고 리부팅합니다. Optical Flow fusion을 위해서 `EKF2_AID_MASK` 파라미터도 설정해야만 합니다.

### Local Position Estimator (LPE)
TODO

<!-- ### INAV (더이상 개발하지 않음)
INAV는 보정을 위해 고정 게인 행렬을 가지며 일정한 상태 kalman filter로 볼수 있습니다. 모든 position estimators 중에 가장 계산량이 적습니다.

#### 실내 비행 비디오
{% youtube %}https://www.youtube.com/watch?v=MtmWYCEEmS8{% endyoutube %}

#### 실외 비행 비디오
{% youtube %}https://www.youtube.com/watch?v=4MEEeTQiWrQ{% endyoutube %}


#### 파라미터
* `INAV_LIDAR_EST`는 1로 설정해서 측정한 거리기반 altitude estimation을 활성화
* `INAV_FLOW_DIST_X` and `INAV_FLOW_DIST_Y`
	이 2개 값(미터)은 yaw 보상으로 사용됩니다.
  offset은 위 Figure 1에 따라서 측정해야만 합니다.
  위 예제에서 PX4Flow의 offset은(붉은 점선) 음수 X offset과 음수 Y offset을 가집니다.
* `INAV_LIDAR_OFF`
  lidar-lite에 대해서 칼리브레이션 offset을 미터 단위로 설정합니다. 해당 값은 측정한 거리에 추가됩니다.


#### 고급 파라미터

고급 사용/개발에 대해서 다음 파라미터도 변경할 수 있습니다. 내용을 알지 못한다면 변경하지 마세요!

* `INAV_FLOW_W`
	flow estimation/update에 대한 weight를 설정
* `INAV_LIDAR_ERR`
	altitude estimation/update 에대한 임계값을 미터단위로 설정. 만약 보정할 값이 이 값보다 크다면 업데이트로 사용할 수 없습니다. -->
