!REDIRECT "https://docs.px4.io/master/ko/debug/mavlink_shell.html"

# MAVLink 셸

MAVLink 셸은 직렬 포트 연결(USB/텔레메트리), 무선랜(UDP/TCP) 연결을 거쳐 MAVLink로 접근할 수 있는 *NSH 콘솔*입니다(특히, 픽스호크, 픽스레이서 등의 NuttX 기반 시스템으로).

셸은 명령, 모듈을 실행하고 해당 결과를 출력하는 식으로 활용할 수 있습니다. 셸은 시작하지 않은 모듈의 출력을 *직접* 할 수는 없지만, `dmsg` 명령을 통해 간접적으로 살펴볼 수 있습니다(`dmesg -f &` 명령으로 작업 큐에서 실행하는 다른 모듈이나 작업의 출력을 화면에 출력할 수 있음).

> **Tip** [QGroundControl MAVLink 콘솔](#qgroundcontrol)은 콘솔에 접근하는 가장 간단한 수단입니다. 시스템을 제대로 시작하지 않았다면 [시스템 콘솔](../debug/system_console.md)을 대신 사용해야합니다.

## 셸 열기

<a id="qgroundcontrol"></a>

### QGroundControl MAVLink Console

셸에 접근하는 가장 간단한 수단은  [QGroundControl MAVLink 콘솔](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_console.html)입니다(**Analyze View > Mavlink Console** 참고).

### mavlink_shell.py

**mavlink_shell.py** 스크립트로 터미널에서 셸에 접근할 수 있습니다:
1. *QGroundControl*을 끄십시오.
1. 의존 요소를 설치하십시오:
   ```sh
   sudo pip3 install pymavlink pyserial
   ```
1. Open terminal (in PX4-Autopilot directory) and start the shell:
   ```sh
   # For serial port
   ./Tools/mavlink_shell.py /dev/ttyACM0
   ```
    ```sh
   # For Wifi connection
   ./Tools/mavlink_shell.py 0.0.0.0:14550
   ```

모든 가용 인자 설명을 보려면 `mavlink_shell.py -h` 명령을 사용하십시오.

## MAVLink 셸 사용법

자세한 정보는 [PX4 콘솔/셸 > 콘솔/셸 사용법](../debug/consoles.md#using_the_console)을 살펴보십시오.
