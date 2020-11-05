# 플라이트기어 모의시험

[플라이트기어](https://www.flightgear.org/)는 강력한 [FDM 엔진](http://wiki.flightgear.org/Flight_Dynamics_Model) 기반 비행 모의시험 환경입니다. 플라이트기어에서는 다양한 기후 환경에서 탐사기체 운행을 모의시험해볼 수 있습니다([ThunderFly s.r.o.](https://www.thunderfly.cz/)가 브릿지를 개발한 이유입니다).

이 페이지는 SITL에서 플라이트기어 단일 기체 활용법을 설명합니다. 다중 기체 정보는 [플라이트기어 다중 기체 모의 시험](../simulation/multi_vehicle_flightgear.md)을 참고하십시오.

**지원 기체:** 오토자이로, 항공기, 탐사선

{% youtube %}https://www.youtube.com/watch?v=iqdcN5Gj4wI{% endyoutube %}


[![머메이드 그래프 ](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEZsaWdodEdlYXIgLS0-IEZsaWdodEdlYXItQnJpZGdlO1xuICBGbGlnaHRHZWFyLUJyaWRnZSAtLT4gTUFWTGluaztcbiAgTUFWTGluayAtLT4gUFg0X1NJVEw7XG5cdCIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEZsaWdodEdlYXIgLS0-IEZsaWdodEdlYXItQnJpZGdlO1xuICBGbGlnaHRHZWFyLUJyaWRnZSAtLT4gTUFWTGluaztcbiAgTUFWTGluayAtLT4gUFg0X1NJVEw7XG5cdCIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)


<!-- Original mermaid graph
graph LR;
  FlightGear-- >FlightGear-Bridge;
  FlightGear-Bridge-- >MAVLink;
  MAVLink-- >PX4_SITL;
-->

> **Note** 모의 시험 프로그램, 모의 시험 환경, 모의 시험 설정(예: 지원 기체) 관련 일반 정보는 [모의 시험](/simulation/README.md)에 있습니다.

<a id="installation"></a>

## Installation (Ubuntu Linux)

> **Note** 우분투 18.04에서 절차를 시험해보았습니다

1. [우분투 LTS / 데비안 리눅스 개발 환경](../setup/dev_env_linux_ubuntu.md)을 설치하십시오.
1. 플라이트기어를 설치하십시오:
   ```sh
   sudo add-apt-repository ppa:saiarcot895/flightgear
   sudo apt update
   sudo apt install flightgear
   ```
   이 과정은 PAA 저장소에서 FGdata 패키지를 함께 끌어오는 최신 안정판 플라이트기어를 설치합니다.

   > **Tip** 일부 모델에서는 (예: 전동 엔진) 새 기능이 들어간 일일 빌드가 필요할 수 있습니다. [일일 빌드 PPA](https://launchpad.net/~saiarcot895/+archive/ubuntu/flightgear-edge)를 설치하십시오.

1. 플라이트기어 실행 가능 여부를 확인하십시오:
   ```
   fgfs --launcher
   ```
1. 플라이트기어 설치 디렉터리의 **Protocols** 폴더에 쓰기 권한을 설정하십시오:
   ```
   sudo chmod a+w /usr/share/games/flightgear/Protocols
   ```
   PX4-FlightGear-Bridge에서 통신 정의 파일을 이곳에 복사하기 때문에 권한 설정이 필요합니다.

추가 설치 과정은 [플라이트기어 위키](http://wiki.flightgear.org/Howto:Install_Flightgear_from_a_PPA)에 있습니다.   

<a id="running"></a>

## Running the Simulation

PX4 SITL을 시작하여 모의 시험환경을 실행하고, 원하는 에어프레임 설정 값을 부여하십시오.

The easiest way to do this is to open a terminal in the root directory of the PX4 *PX4-Autopilot* repository and call `make` for the desired target. 예를 들어 비행체 모의 시험을 시작하려면:
```sh
cd /path/to/PX4-Autopilot
make px4_sitl_nolockstep flightgear_rascal
```

지원 기체와 `make` 명령은 아래와 같습니다(기체 그림을 보려면 링크를 누르십시오).

| 기체                                                                | 명령                                           |
| ----------------------------------------------------------------- | -------------------------------------------- |
| [표준 비행체](../simulation/flightgear_vehicles.md#standard_plane)     | `make px4_sitl_nolockstep flightgear_rascal` |
| [Ackerman 기체 (UGV/탐사선)](../simulation/flightgear_vehicles.md#ugv) | `make px4_sitl_nolockstep flightgear_tf-r1`  |
| [Autogyro](../simulation/flightgear_vehicles.md#autogyro)         | `make px4_sitl_nolockstep flightgear_tf-g1`  |

위 명령은 전체 인터페이스에 단일 기체를 띄워 실행합니다. *QGroundControl*은 모의시험 환경에 모델로 띄운 기체에 자동으로 연결할 수 있어야합니다.

> **Note** FlightGear 빌드 대상 전체 목록 (강조) 을 보려면 다음 명령을 실행하십시오: 
> 
> ```
  make px4_sitl_nolockstep list_vmd_make_targets | grep flightgear_
```
  추가 정보는 [플라이트기어 기체](../simulation/flightgear_vehicles.md)를 살펴보십시오("지원하지 않는" 기체와 새 기체 추가 방법 정보도 있습니다).

<span></span>
> **Note** [파일 및 코드 설치](../setup/dev_env.md) 안내서는 빌드 과정에 오류가 나타날 경우 도움이 될 참고서입니다.


## 하늘로 띄우기

위에서 언급한 `make` 명령은 PX4를 우선 빌드하고 플라이트기어 모의 시험 환경을 띄웁니다.

PX4를 시작하면 아래와 같이 PX4 셸을 실행합니다. 명령 프롬프트를 띄우려면 진입해야합니다.

```
______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [px4] Calling startup script: /bin/sh etc/init.d-posix/rcS 0
INFO  [param] selected parameter default file eeprom/parameters_1034
I'm Mavlink to FlightGear Bridge
Targed Bridge Freq: 200, send data every step: 1
4
  5   -1
  7   -1
  2   1
  4   1
[param] Loaded: eeprom/parameters_1034
INFO  [dataman] Unknown restart, data manager file './dataman' size is 11798680 bytes
INFO  [simulator] Waiting for simulator to accept connection on TCP port 4560
INFO  [simulator] Simulator connected on TCP port 4560.
INFO  [commander] LED: open /dev/led0 failed (22)
INFO  [commander] Mission #3 loaded, 9 WPs, curr: 8
INFO  [init] Mixer: etc/mixers-sitl/plane_sitl.main.mix on /dev/pwm_output0
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [airspeed_selector] No airspeed sensor detected. Switch to non-airspeed mode.
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [logger] logger started (mode=all)
INFO  [logger] Start file log (type: full)
INFO  [logger] Opened full log file: ./log/2020-04-28/22_03_36.ulg
INFO  [mavlink] MAVLink only on localhost (set param MAV_BROADCAST = 1 to enable network)
INFO  [px4] Startup script returned successfully
pxh> StatsHandler::StatsHandler() Setting up GL2 compatible shaders
Now checking for plug-in osgPlugins-3.4.1/osgdb_nvtt.so
PX4 Communicator: PX4 Connected.

pxh>
```

콘솔에서는 PX4에서 airframe별 초기화를 진행하고 매개변수 파일을 읽은 다음, 모의 시험 환경을 기다리(고 연결하)는 상태를 출력합니다. INFO 출력에서는 [ecl/EKF]에서 `혼합 GPS 수신을 시작하고` 기체의 동력을 인가할 준비가 끝났음을 나타냅니다. 여기서 플라이트기어 창과 기체 모습이 나타나야합니다.


> **Note** **Ctrl+V**를 눌러 화면 모습을 바꿔볼 수 있습니다.

![플라이트기어 UI](../../assets/simulation/flightgear/flightgearUI.jpg)

이 기체를 다음 명령으로 띄울 수 있습니다:

```sh
pxh> commander takeoff
```

## 사용법/설정 옵션

플라이트기어 설치/설정 상태를 다음 환경 변수로 세밀하게 조정할 수 있습니다:

- `FG\_BINARY` - 플라이트기어 바이너리 실행 절대 경로. (앱 이미지)
- `FG\_MODELS\_DIR` - 모의시험 환경에서 사용하려 직접 다운로드한 비행체 모델을 넣는 폴더의 절대 경로.
- `FG\_ARGS\_EX` - 추가 FG 매개변수.

<a id="frame_rate"></a>

### Display the frame rate

플라이트기어에서는 **View > View Options > Show frame rate** 에서 프레임 재생율을 표시할 수 있습니다.

<a id="custom_takeoff_location"></a>

### Set Custom Takeoff Location

SITL 플라이트 기어의 이륙 위치는 추가 변수를 활용하여 설정할 수 있습니다. 변수 설정을 통해 기본 이륙 위치를 재지정합니다.

설정할 수 있는 변수는 `--airport`, `--runway`, `--offset-distance`가 있습니다. 다른 옵션은 [플라이트기어 위키](http://wiki.flightgear.org/Command_line_options#Initial_Position_and_Orientation)에 있습니다

예를 들어:
```
FG_ARGS_EX="--airport=PHNL"  make px4_sitl_nolockstep flightgear_rascal
```

위 예제는 [호놀룰루 국제 공항](http://wiki.flightgear.org/Suggested_airports)에서의 모의 시험을 시작합니다.

<a id="joystick"></a>

### Using a Joystick

조종기와 엄지 조종기는 *QGroundControl* 에서 지원합니다([설정 방법은 여기에 있음](../simulation/README.md#joystickgamepad-integration)).

플라이트기어의 조종기 입력을 끄지 않으면 FG 조종기 입력과 PX4 명령간 "입력 처리 경쟁 현상"이 일어날 수 있습니다.


## 확장 및 개별 설정

모의 시험 인터페이스를 확장하거나 개별 설정하려면,   **Tools/flightgear_bridge** 폴더의 파일을 편집하십시오. 코드는 깃허브의 [PX4-FlightGear-Bridge 저장소](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)에 있습니다.


## 추가 정보

* [PX4-FlightGear-Bridge readme](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
