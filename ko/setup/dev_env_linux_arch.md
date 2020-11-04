# 아치 리눅스 개발 환경

The PX4-Autopilot repository provides a convenient script to set your Arch installation up for PX4 development: [Tools/setup/arch.sh](https://github.com/PX4/PX4-Autopilot/blob/{{ book.px4_version }}/Tools/setup/arch.sh).

(기본적으로) 스크립트에서는 PX4 NuttX 대상 빌드에 필요한 모든 도구(RTPS 제외)를 설치하며 *jMAVSim*으로 모의시험 환경을 실행합니다. 추가로 명령행에 `--gazebo` 인자를 지정하여 *가제보(Gazebo)* 모의시험 환경을 설치할 수도 있습니다.

![아치 리눅스용 가제보  ](../../assets/simulation/gazebo/arch-gazebo.png)

> **Note** 아치 리눅스보다 설치가 더 쉬운 [만자로](https://manjaro.org/)(아치 기반 배포판)에서 설명서 절차를 시험해보았습니다.

스크립트를 받아서 실행하려면, 다음 둘 중 한가지 절차를 이행하십시오:

- [PX4 소스코드를 다운로드](../setup/building_px4.md) 하고, 해당 위치에서 스크립트를 실행하십시오: 
        git clone https://github.com/PX4/PX4-Autopilot.git
        bash PX4-Autopilot/Tools/setup/arch.sh

- 필요한 스크립트만 다운로드하고 바로 실행하십시오: 
        sh
        wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/master/Tools/setup/arch.sh
        wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/master/Tools/setup/requirements.txt
        bash arch.sh

이 스크립트는 다음 추가 매개변수를 받을 수 있습니다:

- `--gazebo`: 이 매개변수를 추가하여 [AUR](https://aur.archlinux.org/packages/gazebo/)에서 가제보(Gazebo)를 설치합니다. > **Note** 가제보는 소스트리 안에서 컴파일합니다. 설치에 시간이 좀 걸리며 (의존성 설치시) 여러번의 `sudo` 암호 입력이 필요합니다.
- `--no-nuttx`: NuttX/픽스호크 툴체인을 설치하지 않습니다 (모의시험 환경만 설치할 경우).
- `--no-sim-tools`: jMAVSim/가제보를 설치하지 않습니다(예: 픽스호크/NuttX 대상만 빌드할 경우)