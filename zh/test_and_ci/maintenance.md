# 维护说明

这选择并描述了一些工具来帮助分析代码库的状态并支持其维护。

## 分析 churn

改动的数量，因此对文件所做的更改次数可以指示哪些文件/部件可能需要重构。

要查找流失指标，可以使用 [Churn](https://github.com/danmayer/churn) 等工具：

    gem install churn
    

从 `v1.6.0-rc2` 开始的示例输出将是：

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