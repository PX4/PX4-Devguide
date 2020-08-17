# MAVROS

[mavros](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.sys_status) ROS 패키지는 ROS를 구동하는 컴퓨터와, MAVLink를 활성화한 오토파일럿, MAVLink 기능을 지닌 GCS간의 MAVLink 확장 통신이 가능하게 합니다.

> **Note** *MAVROS*는 ROS와 MAVLink 프로토콜간 "공식" 지원 브릿지입니다. 현재로서는 [고속 RTPS 메세징](../middleware/micrortps.md), PX4 [uORB 메세지](../middleware/uorb.md)의 일반 ROS 문맥 변환 계층을 사용할 수 있도록 확장했습니다.

MAVROS 가 MAVLink를 활성화한 어떤 오토파일럿과도 통신할 수 있지만, 이 문서에서는 PX4 플라이트 스택과 ROS 활성 보조 컴퓨터의 통신 활성화 상황만 다룹니다.

## 설치

MAVROS 는 소스코드 또는 바이너리 그 어떤 수단으로든 설치할 수 있습니다. ROS롤 다루는 개발자는 소스 코드 설치 방식을 추천드립니다.

> **Tip** 이 절차는 [공식 설치 안내서](https://github.com/mavlink/mavros/tree/master/mavros#installation)에서 단순화했습니다. 이 문서에서는 *ROS Melodic* 릴리스를 다룹니다.

### 바이너리 설치 (데비안 / 우분투)

ROS 저장소에는 x86, amd64(x86\_64), armhf(ARMv7)용 우분투 바이너리 패키지가 들어있습니다. 키네틱에서는 데비안 Jessie amd64와 arm64(ARMv8)를 지원합니다.

설치시 `apt-get` 명령을 사용하십시오:

    sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
    

`install_geographiclib_datasets.sh` 스크립트를 실행하여 [GeographicLib](https://geographiclib.sourceforge.io/) 데이터셋을 설치하십시오:

    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    ./install_geographiclib_datasets.sh
    

### 소스 코드 설치

이 설치 과정에서는 catkin 작업 영역 `~/catkin_ws` 디렉터리를 만들었다고 가정합니다. 만약 이 디렉터리가 없다면 다음 명령을 실행하십시오:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```

이번 설치 과정에서는 ROS 파이썬 도구 *wstool* (소스 코드 다운로드), *rosinstall*, and *catkin_tools* (빌드) 를 활용하도록 하겠습니다. While they may have been installed during your installation of ROS you can also install them with:

```sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```

> **Tip** **catkin_make** 명령으로 패키지를 빌드할 수 있지만, **catkin_tools** 명령을 활용하는 방식이 선호하는 방식인 이유는 기능이 다양하고 사용이 "편한" 빌드 도구이기 때문입니다.

wstool을 처음 사용한다면 소스 코드 영역을 다음 명령으로 초기화해야합니다:

```sh
$ wstool init ~/catkin_ws/src
```

이제 빌드할 준비가 끝났습니다

1. MAVLink를 설치하십시오: 
        # We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
        rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

2. MAVROS를 출시 버전 또는 최신 버전의 소스 코드로 설치하십시오:
    
    - 출시/안정 ```rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall```
    - 최신 소스 코드 
            sh
            rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
        
            sh
            # For fetching all the dependencies into your catkin_ws, 
            # just add '--deps' to the above scripts, E.g.:
            #   rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall

3. 작업 영역과 의존 요소 트리를 만드십시오
    
        wstool merge -t src /tmp/mavros.rosinstall
        wstool update -t src -j4
        rosdep install --from-paths src --ignore-src -y
        

4. [GeographicLib](https://geographiclib.sourceforge.io/) 데이터세트를 설치하십시오:
    
        ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
        

5. 소스 코드를 빌드하십시오
    
        catkin build
        

6. 작업 영역에서 setup.bash 또는 setup.zsh 스크립트를 사용했는지 확인하십시오.
    
        #Needed or rosrun can't find nodes from this workspace.
        source devel/setup.bash
        

오류가 발생했을 경우, [mavros 저장소](https://github.com/mavlink/mavros/tree/master/mavros#installation)의 추가 설치 및 문제 해결 참고 부분을 확인하십시오.