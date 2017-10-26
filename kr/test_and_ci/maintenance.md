# 유지보수 노트

코드베이스의 상태를 분석하고 유지보수에 도움이 되는 도구에 대해서 설명합니다.

## Analyze churn

churn의 양, 파일에 변경이 일어난 수는 어떤 파일과 부분에 리팩토링이 필요한지를 보여주는 지표가 될 수 있습니다.

churn 메트릭을 찾는 도구로 [Churn](https://github.com/danmayer/churn)를 사용할 수 있습니다.:

```
gem install churn
```

`v1.6.0-rc2`의 출력 예제로 :

```
cd src/Firmware
churn --start_date "6 months ago"
**********************************************************************
* Revision Changes
**********************************************************************
Files
+------------------------------------------+
| file                                     |
+------------------------------------------+
| src/modules/navigator/mission.cpp        |
| src/modules/navigator/navigator_main.cpp |
| src/modules/navigator/rtl.cpp            |
+------------------------------------------+



**********************************************************************
* Project Churn
**********************************************************************

Files
+---------------------------------------------------------------------------+---------------+
| file_path                                                                 | times_changed |
+---------------------------------------------------------------------------+---------------+
| src/modules/mc_pos_control/mc_pos_control_main.cpp                        | 107           |
| src/modules/commander/commander.cpp                                       | 67            |
| ROMFS/px4fmu_common/init.d/rcS                                            | 52            |
| Makefile                                                                  | 49            |
| src/drivers/px4fmu/fmu.cpp                                                | 47            |
| ROMFS/px4fmu_common/init.d/rc.sensors                                     | 40            |
| src/drivers/boards/aerofc-v1/board_config.h                               | 31            |
| src/modules/logger/logger.cpp                                             | 29            |
| src/modules/navigator/navigator_main.cpp                                  | 28            |
```
