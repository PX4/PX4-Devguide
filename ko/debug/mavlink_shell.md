# MAVLink 셸

The MAVLink Shell is an *NSH console* that can be accessed via MAVLink over serial (USB/Telemetry) or WiFi (UDP/TCP) links (in particular, on NuttX-based systems like: Pixhawk, Pixracer, etc.).

The shell can be used for running commands and modules, and displaying their output. While the shell cannot *directly* display the output of modules that it does not start, it can do so indirectly using the `dmesg` command (`dmesg -f &` can be used to display the output of other modules and tasks running on the work queue).

> **Tip** The [QGroundControl MAVLink Console](#qgroundcontrol) is the easiest way to access the console. If the system does not start properly you should instead use the [System Console](../debug/system_console.md).

## Opening the Shell

### QGroundControl MAVLink Console {#qgroundcontrol}

The easiest way to access shell is to use the [QGroundControl MAVLink Console](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_console.html) (see **Analyze View > Mavlink Console**).

### mavlink_shell.py

You can also access the shell in a terminal using the **mavlink_shell.py** script:
1. *QGroundControl*을 끄십시오.
1. 의존 요소를 설치하십시오:
   ```sh
   sudo pip3 install pymavlink pyserial
   ```
1. 터미널을 (Firmware 디렉터리에서) 열고 셸을 시작하십시오:
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
