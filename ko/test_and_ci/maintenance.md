# 유지관리 참고

여기서는 코드 베이스 상태를 분석하고 유지 관리를 지원하는 몇가지 도구를 설명합니다.

## Analyze churn

The amount of churn, so the number of changes done to a file can be an indicator which files/parts might need refactoring.

To find churn metrics a tool such as [Churn](https://github.com/danmayer/churn) can be used:

    gem install churn
    

`v1.6.0-rc2` 출력 예제는 다음과 같습니다:

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