# 플라이트기어 모의시험

[플라이트기어](https://www.flightgear.org/)는 강력한 [FDM 엔진](http://wiki.flightgear.org/Flight_Dynamics_Model) 기반 비행 모의시험 환경입니다. 플라이트기어에서는 다양한 기후 환경에서 탐사기체 운행을 모의시험해볼 수 있습니다([ThunderFly s.r.o.](https://www.thunderfly.cz/)가 브릿지를 개발한 이유입니다).

이 페이지는 SITL에서 플라이트기어 단일 기체 활용법을 설명합니다. 다중 기체 정보는 [플라이트기어 다중 기체 모의 시험](../simulation/multi_vehicle_flightgear.md)을 참고하십시오.

**지원 기체:** 오토자이로, 항공기, 탐사선

{% youtube %}https://www.youtube.com/watch?v=iqdcN5Gj4wI{% endyoutube %}


[![Mermaid Graph ](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEZsaWdodEdlYXIgLS0-IEZsaWdodEdlYXItQnJpZGdlO1xuICBGbGlnaHRHZWFyLUJyaWRnZSAtLT4gTUFWTGluaztcbiAgTUFWTGluayAtLT4gUFg0X1NJVEw7XG5cdCIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEZsaWdodEdlYXIgLS0-IEZsaWdodEdlYXItQnJpZGdlO1xuICBGbGlnaHRHZWFyLUJyaWRnZSAtLT4gTUFWTGluaztcbiAgTUFWTGluayAtLT4gUFg0X1NJVEw7XG5cdCIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)


<!-- Original mermaid graph
graph LR;
  FlightGear-- >FlightGear-Bridge;
  FlightGear-Bridge-- >MAVLink;
  MAVLink-- >PX4_SITL;
-->

> **Note** See [Simulation](/simulation/README.md) for general information about simulators, the simulation environment, and simulation configuration (e.g. supported vehicles).


## 설치(우분투 리눅스) {#installation}

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


## 모의 시험 환경 실행 {#running}

PX4 SITL을 시작하여 모의 시험환경을 실행하고, 원하는 에어프레임 설정 값을 부여하십시오.

가장 쉬운 방법은 PX4 *Firmware* 저장소 루트 디렉터리에서 터미널을 열고 원하는 대상에 대해 `make` 명령을 호출하는 방식입니다. 예를 들어 비행체 모의 시험을 시작하려면:
```sh
cd /path/to/Firmware
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


## Taking it to the Sky

The `make` commands mentioned above first build PX4 and then run it along with the FlightGear simulator.

Once the PX4 has started it will launch the PX4 shell as shown below. You must select enter to get the command prompt.

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

The console will print out status as PX4 loads the airframe-specific initialization and parameter files, wait for (and connect to) the simulator. Once there is an INFO print that [ecl/EKF] is `commencing GPS fusion` the vehicle is ready to arm. At this point, you should see a FlightGear window with some view of aircraft.


> **Note** You can change the view by pressing **Ctrl+V**.

![FlightGear UI](../../assets/simulation/flightgear/flightgearUI.jpg)

You can bring it into the air by typing:

```sh
pxh> commander takeoff
```

## Usage/Configuration Options

You can tune your FG installation/settings by the following environment variables:

- `FG\_BINARY` - absolute path to FG binary to run. (It can be an AppImage)
- `FG\_MODELS\_DIR` - absolute path to the folder containing the manually-downloaded aircraft models which should be used for simulation.
- `FG\_ARGS\_EX` - any additional FG parameters.

### Display the frame rate {#frame_rate}

In FlightGear you can display the frame rate by enabling it in: **View > View Options > Show frame rate**.

### Set Custom Takeoff Location {#custom_takeoff_location}

Takeoff location in SITL FlightGear can be set using additional variables. Setting the variable will override the default takeoff location.

The variables which can be set are as follows: `--airport`, `--runway`, and `--offset-distance`. Other options can be found on [FlightGear wiki](http://wiki.flightgear.org/Command_line_options#Initial_Position_and_Orientation)

For example:
```
FG_ARGS_EX="--airport=PHNL"  make px4_sitl_nolockstep flightgear_rascal
```

The example above starts the simulation on the [Honolulu international airport](http://wiki.flightgear.org/Suggested_airports)


### Using a Joystick {#joystick}

조이스틱과 엄지 조이스틱은 *QGroundControl* 에서 지원합니다([설정 방법은 여기에 있음](../simulation/README.md#joystickgamepad-integration)).

The joystick input in FlightGear should be disabled in otherwise there will be a "race condition" between the FG joystick input and PX4 commands.


## Extending and Customizing

To extend or customize the simulation interface, edit the files in the **Tools/flightgear_bridge* folder. The code is available in the [PX4-FlightGear-Bridge repository](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge) on Github.


## Further Information

* [PX4-FlightGear-Bridge readme](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
