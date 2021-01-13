# Gazebo 시뮬레이션

[Gazebo](http://gazebosim.org)는 자동 로못을 위한 3D 시뮬레이션 환경입니다. ROS없이 독립적으로 사용하거나 SITL + ROS를 지원합니다.

{% youtube %}https://www.youtube.com/watch?v=qfFF9-0k4KA&vq=hd720{% endyoutube %}


[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEdhemViby0tPlBsdWdpbjtcbiAgUGx1Z2luLS0-TUFWTGluaztcbiAgTUFWTGluay0tPlNJVEw7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggTFI7XG4gIEdhemViby0tPlBsdWdpbjtcbiAgUGx1Z2luLS0-TUFWTGluaztcbiAgTUFWTGluay0tPlNJVEw7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)

## 설치

Gazebo와 시뮬레이션 플러그인 설치가 필요합니다.

> ** Note ** Gazebo 버전 7을 추천합니다.(최소 Gazebo 6 버전) Linux을 실행하고 Jade보다 이전 ROS 버전이 설치되어 있다면 예전 버전인 번들 Gazebo를 언인스톨해야 합니다.(sudo apt-get remove ros-indigo-gazebo)

설치 관련 정보는 [Linux](../setup/dev_env_linux.md)와 [Mac](../setup/dev_env_mac.md)를 참고하세요.

#### ROS 사용자

PX4 SITL은 Gazebo 시뮬레이터를 사용하며 ROS에 의존하지 않습니다. 시뮬레이션은 [ROS로 인터페이스](../simulation/ros_interface.md)될 수 있고 동일한 방식으로 일반 flight 코드에도 적용될 수 있습니다.

PX4를 ROS와 사용할려고 한다면, 다음 ROS를 위한 [Gazebo 버전 7 가이드](http://gazebosim.org/tutorials?tut=ros_wrapper_versions#Gazebo7.xseries)를 참고하세요.

#### 일반 설치

Gazebo 7용 [Linux 설치 방법](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)을 참고하세요.

모두 설치되었는지 확인 : `gazebo7` and `libgazebo7-dev`.

## 시뮬레이션 실행

PX4 펌웨어의 소스 디렉토리내에서 airframe 중에 하나로(Quads, planes, VTOL을 지원하며 optical flow 포함) PX4 SITL을 실행합니다 :

> **Note** Gazebo 실행을 유지하면서 PX4를 재실행하려면 아래와 같이 합니다.

### Quadrotor

```sh
cd ~/src/Firmware
make px4_sitl gazebo
```

### Quadrotor with Optical Flow

```sh
make px4_sitl gazebo_iris_opt_flow
```

### 3DR Solo

```sh
make px4_sitl gazebo_solo
```

![](../../assets/gazebo/solo.png)

### 표준 Plane

```sh
make px4_sitl gazebo_plane
```

![](../../assets/gazebo/plane.png)

### 표준 VTOL

```sh
make px4_sitl gazebo_standard_vtol
```

![](../../assets/gazebo/standard_vtol.png)

### Tailsitter VTOL

```sh
make px4_sitl gazebo_tailsitter
```

![](../../assets/gazebo/tailsitter.png)

### Ackerman vehicle (UGV/Rover) {#ugv}

```sh
make px4_sitl gazebo_rover
```

![](../../assets/gazebo/rover.png)


## World 바꾸기

현재 기본 world는 iris.wold로 [worlds](https://github.com/PX4/sitl_gazebo/tree/367ab1bf55772c9e51f029f34c74d318833eac5b/worlds) 디렉토리에 있습니다. iris.world에 기본 환경에서 ground로 heightmap을 사용합니다. 이 ground로는 거리 센서 사용이 어렵습니다. heightmap으로 예상치 못한 결과가 나오면 iris.model에 있는 uneven_ground를 asphalt_plane으로 model을 변경하는 것을 추천합니다.

## 하늘로 날리기

> ** Note ** 에러가 발생하면 [파일과 코드 설치](../setup/dev_env_mac.md) 가이드를 참고하세요.

PX4 쉘이 나타납니다:

```sh
[init] shell id: 140735313310464
[init] task name: px4

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.


pxh>
```

> ** Note ** 쿼드로터 모델을 오른쪽 클릭하면 콘텍스트 메뉴에서 팔로우 모드를 활성화시킬 수 있습니다.
Right-clicking the quadrotor model allows to enable follow mode from the context menu, which is handy to keep it in view.

![Gazebo UI](../../assets/simulation/gazebo.png)

시스템은 일단 초기화를 마치면 home position을 출력합니다. (`telem> home: 55.7533950, 37.6254270, -0.00`) 다음을 입력해서 공중으로 띄웁니다 :

```sh
pxh> commander takeoff
```

> ** Note ** 조이스틱 지원은 QGroundControl(QGC)를 통해서 가능합니다. 수동 입력을 사용하기 위해서는 메뉴얼 비행 모드로(POSCTL, position control) 시스템을 둡니다. QGC preferences 메뉴에서 조이스틱을 활성화시킬 수 있습니다.

## 커스텀 takeoff 위치 설정

SITL에서 디폴트 takeoff 위치는 환경변수로 바꿀수 있습니다.

설정할 변수는 : `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`

예로:
```
export PX4_HOME_LAT=28.452386
export PX4_HOME_LON=-13.867138
export PX4_HOME_ALT=28.5
make px4_sitl gazebo
```

## Gazebo와 PX4를 별도로 구동시키기

확장 개발 세션에 대해서 Gazebo와 PX4를 별도로 혹은 IDE 내부에서 구동시키는 것이 편할 수 있습니다.

px4가 `sitl_run.sh` 를 실행시키는 기존 cmake 타겟에 추가하는데 여기에 px4가 올바른 모델을 로드하도록 파라미터를 사용합니다. 원래 sitl px4 app의 wrapper 역할을 하는 `px_<mode>`라는 이름의 launcher 타겟을 생성합니다. 이 wrapper는 단순히 현재 워킹 디렉토리와 모델 파일에 대한 패스와 같이 app 인자를 포함하고 있습니다.

### 사용 방법

  * 터미널을 통해서 gazebo(다른 시뮬레이터) 서버와 클라이언트 뷰어 실행:
```
make px4_sitl_default gazebo_none_ide
```
  * IDE에서 디버깅하기를 원하는 `px4_<mode>` 타겟을 선택 (예로 `px4_iris`)
  * IDE에서 바로 디버깅 세션을 시작
이 방법은 시뮬레이터가 항상 백그라운드로 동작하기 때문에 사이클 타임을 상당히 줄여주는데 이유는 시뮬레이터는(gazebo) 항상 백그라운드로 실행되고 px4 프로스세가 매우 가벼워서 재실행만 하면 됩니다.

## 확장과 커스터마이징

시뮬레이션 인터페이스를 확장하고 커스터마이징하기 위해서 `Tools/sitl_gazebo` 폴더에 있는 파일을 수정합니다. 코드는 Github에 [sitl_gazebo repository](https://github.com/px4/sitl_gazebo)를 참고하세요.

> ** Note ** 빌드 시스템은 올바른 GIT 서브모듈과 시뮬레이터가 있어야합니다. 디렉토리에 파일들에 변경 내용을 덮어쓰기는 하지 않습니다.

## ROS 인터페이스

시뮬레이션은 실제 비행체와 동일한 방식으로 [ROS 인터페이스](../simulation/ros_interface.md)를 사용합니다.
