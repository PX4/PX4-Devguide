# FAKE GPS
이 페이지에서는 mocap 데이터를 가짜 gps로 사용하는 방법에 대해서 알아봅니다.

셋업은 다음과 같습니다. :
"VICON 컴퓨터"와 "여러분 컴퓨터"가 필요합니다. "VICON 컴퓨터"는 [ROS](http://www.ros.org/)와 [vicon_bridge](https://github.com/ethz-asl/vicon_bridge)가 설치되어 있어야 하며 네트워크를 이용해서 이 데이터를 "여러분 컴퓨터"로 보내게 됩니다. "여러분 컴퓨터"에는 ROS와 MAVROS가 설치되어 있어야 합니다. MAVROS에는 mocap 데이터를 이용해서 gps 데이터를 시뮬레이션할 수 있는 스크립트가 있습니다.
다음으로 "여러분 컴퓨터"는 3DR radiometry를 이용해서 Pixhawk로 데이터를 보냅니다.
*주의*: "VICON 컴퓨터"와 "여러분 컴퓨터"는 물론 동일한 것일 수도 있습니다.

## 전제 조건
* MOCAP 시스템 (여기 예제에서는 VICON을 사용)
* 컴퓨터 (ROS, MAVROS, Vicon_bridge가 설치)
* 3DR radiometry 셋트

## 절차
### Step 1
"여러분 컴퓨터"는 "VICON 컴퓨터"와 동일한 네트워크 상에 있어야 합니다.(무선 아답터 이용 가능) "VICON 컴퓨터"에 2개 파일을 생성합니다 : "launch_fake_gps.sh"와 "launch_fake_gps_distorted.sh"

"launch_fake_gps.sh" 파일에 다음과 같이 2줄을 추가하고 xxx.xxx.x.xxx 부분은 "여러분 컴퓨터"의 IP 주소로 대체합니다. (터미널에서 "ifconfig"를 입력해서 IP 주소를 얻을 수 있습니다.)
```sh
export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
roslaunch vicon_bridge vicon.launch $@
```

다음으로 "launch_fake_gps_distorted.sh" 파일에 다음과 같이 2줄을 추가하고 xxx.xxx.x.xxx 부분은 "여러분 컴퓨터"의 IP 주소로 대체합니다.
```sh
export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
rosrun vicon_bridge tf_distort $@
```

드론에 마커를 붙이고 MOCAP 시스템에서 model을 생성합니다. (아래 yourModelName에 해당)

### Step 2
실행
```sh
$ roscore
```
"여러분 컴퓨터"에서


### Step 3
실행
```sh
$ sh launch_fake_gps.sh
```
그리고
```sh
$ sh launch_fake_gps_distorted.sh
```
"VICON 컴퓨터"의 2개 파일을 생성한 디렉토리에 2개 서로 다른 터미널을 이용합니다.


### Step 4
"여러분 컴퓨터"에서 실행
```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```
새 윈도우는 "tf_distort"를 선택할 수 있는 곳에 열어야 합니다. 이 도구로 MOCAP 데이터를 조작하기 위해 파라미터를 편집할 수 있습니다.

GPS를 시뮬레이션하기 :
* publish rate = 5.0Hz
* tf_frame_in = vicon/yourModelName/yourModelName (e.g. vicon/DJI_450/DJI_450)
* delay = 200ms
* sigma_xy = 0.05m
* sigma_z = 0.05m


### Step 5
Pixhawk를 QGroundControl에 연결합니다. PARAMETERS -> System로 가서 SYS_COMPANION을 257600으로 변경합니다. (매직 모드를 활성화;)).

PARAMETERS -> MAVLink로 가서 MAV_USEHILGPS를 1로 변경 (HIL GPS 활성화)

PARAMETERS -> Attitude Q estimator로 가서 ATT_EXT_HDG_M을 2로 변경합니다.(motion capture에서 heading을 사용)

마지막으로 PARAMETERS -> Position Estimator INAV로 가서 INAV_DISAB_MOCAP를 1로 변경합니다. (mocap estimation을 비활성화 시킴)

*NOTE*: 위에서 언급한 파라미터를 찾지 못했다면, PARAMETERS -> default Group을 확인해 보세요.


### Step 6
다음으로 "mocap_fake_gps.cpp"을 열어봅시다. 아래 경로에 위치하고 있습니다.:
yourCatkinWS/src/mavros/mavros_extras/src/plugins/mocap_fake_gps.cpp

DJI_450/DJI_450를 여러분이 정한 모델 이름으로 대체합니다. (e.g. /vicon/yourModelName/yourModelname_drop)
```sh
mocap_tf_sub = mp_nh.subscribe("/vicon/DJI_450/DJI_450_drop", 1, &MocapFakeGPSPlugin::mocap_tf_cb, this);
```
"_drop"은 다음 단계에서 설명합니다.


### Step 7
step 5에서 motion capture의 heading을 활성화시켰습니다. 따라서 pixhawk는 원래의 North, East 방향을 사용하지 않고 motion capture 시스템의 것을 사용합니다. 3DR radiometry 장치가 빠르지 않기 때문에 MOCAP 데이터의 rate를 제한해야만 합니다. 따라서 다음을 실행합니다.
```sh
$ rosrun topic_tools drop /vicon/yourModelName/yourModelName 9 10
```
이 의미는 rostopic /vicon/yourModelName/yourModelName에서 10개 메시지 중에 9개가 drop되고 topic 이름 "/vicon/yourModelName/yourModelName_drop"로 publish됩니다.


### Step 8
3DR radiometry를 pixhawk TELEM2에 연결하고 카운터파트는 여러분 컴퓨터(USB)에 연결합니다.


### Step 9
catkinWS로 가서 실행
```sh
$ catkin build
```
그리고 나서
```sh
$ roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600
```
여기까지입니다! 여러분의 pixhawk는 이제 GPS 데이터를 수신하고 녹색 불이 들어올 것입니다.
