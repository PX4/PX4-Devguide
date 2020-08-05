# 윈도우 가상 머신 제공 툴체인

> **Note** [윈도우 Cygwin 툴체인](../setup/dev_env_windows_cygwin.md)이 윈도우 개발 환경에서 (유일하게) 공식 지원하는 툴체인입니다.

윈도우 개발자는 PX4 툴체인을 게스트 운영체제로 설치한 리눅스 가상 머신(VM)에서에서 실행할 수 있습니다. 가상 머신 설치 후, 가상 머신 내의 PX4 설치 및 설정은 리눅스 자체를 설치한 컴퓨터와 동일합니다.

> **Tip** 가능하면 가상 머신에 CPU 코어와 메모리를 많이 할당하십시오.

VM을 활용하는 방법이 설치하고 펌웨어 빌드 환경을 시험하기에 가장 쉬운 방버이긴 하나, 사용자 여러분은 다음을 인지하고 있어야 합니다:

1. 펌웨어 빌드는 리눅스 자체에서 빌드할 때보다 느립니다.
2. jMAVSim 프레임 재생율은 리눅스 자체에서 재생할 때보다 느립니다. 어떤 경우에는 가상 머신 자원이 부족하여 기체가 멈출 수 있습니다.
3. 가제보와 ROS를 설치할 수 있습니다만, 도저히 못쓸 만큼 느립니다.

## 절차

시스템에 PX4 환경 실행 기능을 심은 VM 환경을 설치하는 방법에는 여러가지가 있습니다. 이 안네서에서는 VMWare 설치 방식으로 진행하도록 하겠습니다. VMWare performance is acceptable for basic usage (building Firmware) but not for running ROS or Gazebo.

1. Download [VMWare Player Freeware](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html)
2. Install it on your Windows system
3. Download the desired version of [Ubuntu Desktop ISO Image](https://www.ubuntu.com/download/desktop). (see [Linux Instructions Page](../setup/dev_env_linux.md) for recommended Ubuntu version).
4. Open *VMWare Player* and select the option to create a new virtual machine
5. In the VM creation wizard choose the downloaded Ubuntu ISO image as your installation medium and will automatically detect the operating system you want to use
6. Also in the wizard, select the resources you want to allocate to your virtual machine while it is running. Allocate as much memory and as many CPU cores as you can without rendering your host Windows system unusable.
7. Run your new VM at the end of the wizard and let it install Ubuntu following the setup instructions. Remember all settings are only for within your host operating system usage and hence you can disable any screen saver and local workstation security features which do not increase risk of a network attack.
8. Once the new VM is booted up make sure you install *VMWare tools drivers and tools extension* inside your guest system. This will enhance performance and usability of your VM usage: 
    - Significantly enhanced graphics performance
    - Proper support for hardware device usage like USB port allocation (important for target upload), proper mouse wheel scrolling, sound suppport
    - Guest display resolution adaption to the window size
    - Clipboard sharing to host system
    - File sharing to host system
9. Continue with [PX4 environment setup for Linux](../setup/dev_env_linux.md)