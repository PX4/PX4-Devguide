# 硬件在环仿真（HITL）

硬件在环仿真模式 (HITL 或 HIL) 下 PX4 固件代码运行在真实的飞行控制器硬件平台上。 这种方法的优点是可以在实际硬件上测试大多数的实际飞行代码。

PX4 supports HITL for multicopters (using jMAVSim or Gazebo) and VTOL (using Gazebo).

## HITL兼容机架 {#compatible_airframe}

目前兼容的机架构型和模拟器的情况如下：

| 机架                                                                                                     | `SYS_AUTOSTART` | Gazebo | jMAVSim |
| ------------------------------------------------------------------------------------------------------ | --------------- | ------ | ------- |
| <a href="../airframes/airframe_reference.md#copter_simulation_(copter)_hil_quadcopter_x">HIL Quadcopter X</a>                                                                              | 1001            | Y      | Y       |
| <a href="../airframes/airframe_reference.md#vtol_standard_vtol_hil_standard_vtol_quadplane">HIL Standard VTOL QuadPlane</a>                                                                              | 1002            | Y      |         |
| [Generic Quadrotor x](../airframes/airframe_reference.md#copter_quadrotor_x_generic_quadcopter) copter | 4001            | Y      | Y       |
| [DJI Flame Wheel f450](../airframes/airframe_reference.md#copter_quadrotor_x_dji_flame_wheel_f450)     | 4011            | Y      | Y       |

## HITL 仿真环境 {#simulation_environment}

硬件在环仿真（HITL）模式下标准的 PX4 固件在真实的硬件上运行。 JMAVSim 或 Gazebo (运行在开发计算机上) 通过 USB或者串口 完成与飞行控制器硬件平台连接。 模拟器充当在 PX4 和 *QGroundControl* 之间共享 MAVLink 数据的网关。

> **Note** 如果飞行控制器支持网络连接且使用的是稳定、低延迟的连接（如有线以太网，WIFI 通常不太稳定），那么模拟器也可以使用 UDP 完成通讯连接。 例如，该配置已经使用一台运行 PX4 且通过以太网连接到开发计算机的 Raspberry Pi 进行了验证测试 (包括 jMAVSim 运行命令的启动配置在 [这里](https://github.com/PX4/Firmware/blob/master/posix-configs/rpi/px4_hil.config))。

下面展示了仿真模拟的环境：

* 飞控板 HITL 模式被激活 (通过 *QGroundControl*) ，该模式下不会启动飞控板上任何传感器。
* *jMAVSim* 或者 *Gazebo* 通过 USB 连接到飞控板。
* 模拟器通过 UDP 连接到 *QGroundControl* 并将 MAVLink 数据传输至 PX4 。
* *Gazebo*和*jMAVSim*也可以连接到 offboard API和桥接MAVLink消息到 PX4。
* (可选) 通过串口可将操纵杆/游戏手柄通过 *QGroundControl* 连接至仿真回路中。

![HITL Setup - jMAVSim and Gazebo](../../assets/simulation/px4_hitl_overview_jmavsim_gazebo.png)

## HITL 相比于 SITL

SITL 开发计算机中的模拟环境中运行, 并使用专门为该环境生成的固件。 除了通过模拟器提供模拟环境数据的模拟驱动数据以外，系统正常运作的其他驱动数据。

与此相对照，HITL 在正常硬件上运行正常的 PX4 固件。 仿真数据进入整个仿真系统的时间点与 SITL 有所不同。 commander 和传感器等核心模块在启动时有HITL 模式，这种模式绕过了某些正常功能。

总而言之， HITL 在真实硬件上运行标准 PX4 固件，而 SITL 实际上要比标准 PX4 系统执行更多的代码。

## 配置 HITL

### PX4配置

1. 通过 USB 将自动驾驶仪直接连接到 *QGroundControl*。
2. 激活 HITL 模式
    
    1. 打开 **Setup > Safety** 选项卡。
    2. 在 *HITL Enabled* 下拉框中选择 **Enabled** 完成 HITL 模式的激活。
        
        ![QGroundControl HITL 配置](../../assets/gcs/qgc_hitl_config.png)

3. 选择机架
    
    1. 打开 **Setup > Airframes** 选项卡。
    2. 选择一个你想要进行测试的 [兼容的机架](#compatible_airframe) 。 Then click **Apply and Restart** on top-right of the *Airframe Setup* page.
        
        ![选择机架](../../assets/gcs/qgc_hil_config.png)

4. 如有必要, 校准您的 RC 遥控器 或操纵杆。

5. 设置 UDP
    
    1. 在设置菜单的 "*General*" 选项卡下, 取消选中 *AutoConnect* 一栏中除 **UDP** 外的所有复选框。
        
        ![GITL 模式 QGC 自动连接设置](../../assets/gcs/qgc_hitl_autoconnect.png)

6. (可选) 配置操纵杆和故障保护。 Set the following [parameters](https://docs.px4.io/master/en/advanced_config/parameters.html) in order to use a joystick instead of an RC remote control transmitter:
    
    * [COM_RC_IN_MODE](../advanced/parameter_reference.md#COM_RC_IN_MODE) 更改为 "Joystick/No RC Checks". 这允许操纵杆输入并禁用 RC 输入检查。
    * [NAV_RCL_ACT](../advanced/parameter_reference.md#NAV_RCL_ACT) to "Disabled". 这可确保在没有无线遥控的情况下运行 HITL 时 RC 失控保护不会介入。
    
    > **Tip** *QGroundControl User Guide* 中也有如何配置 [操纵杆](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html) 和 [虚拟操纵杆](https://docs.qgroundcontrol.com/en/SettingsView/VirtualJoystick.html) 的说明。

完成所有的配置设定后 **关闭** *QGroundControl* 并断开飞控板与计算机的连接。

### 模拟器特定设置

在以下章节中按照特定模拟器的适当设置步骤。

#### Gazebo

> **Note** 确保 *QGroundControl* 没有运行！

1. 使用 Gazebo 构建PX4 (用于构建Gazebo 插件)。 
        cd <Firmware_clone>
        make px4_sitl_default gazebo

2. 打开飞行器模型的 sdf 文件（例如 **Tools/sitl_gazebo/models/iris/iris.sdf**）。
3. 找到文件的 `mavlink_interface plugin` 分区，将 `serialEnabled` 和 `hil_mode` 参数更改为 `true` 。
    
    ![HIL 参数](../../assets/simulation/gazebo_sdf_model_hil_params.png)
    
    > **Note** iris.sdf文件时自动生成的。 因此，您需要保存您修改过的文件的副本，或者为每个版本重新编辑。

4. 如有必要的话替换掉 `serialDevice` 参数 (`/dev/ttyACM0`) 。
    
    > **Note** 串口设备参数取决于载具与计算机使用哪个端口完成连接 (通常情况下都是 `/dev/ttyACM0`)。 在 Ubuntu 上最简单的一个检测办法就是将自驾仪插入电脑，然后打开终端窗口输入 `dmesg | grep "tty"` 命令。 命令执行结果中最后一个显示的设备就是我们关心的。

5. 设置环境变量
    
    ```sh
    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    ```
    
    在 HITL 模式下运行 Gazebo ：:
    
    ```sh
    gazebo Tools/sitl_gazebo/worlds/iris.world
    ```

6. 启动*QGroundControl*。 它应该会自动连接 PX4 和 Gazebo 。

#### jMAVSim (Quadrotor only) {#jmavsim_hitl_configuration}

> **Note** 确保 *QGroundControl* 没有运行！

1. 将飞行控制器连接到计算机, 并等待其启动。
2. 在 HIL 模式下运行 jMAVSim 
        sh
        ./Tools/jmavsim_run.sh -q -d /dev/ttyACM0 -b 921600 -r 250
    
    **Note** 酌情替换序列端口名称`/dev/ttyACM0`。 在 macOS 上，这个端口将是 `/dev/tty.bankmodem1`。 在 Windows 上(包括Cygwin) 它将是 COM1 或另一个端口 - 请检查 Windows 设备管理器中的连接。
3. 开启 *QGroundControl*。 它应该会自动连接 PX4 和 Gazebo 。

## 在 HITL 仿真中执行自主飞行任务

你可以使用 *QGroundControl* 实现对飞机的控制并令其 [执行飞行任务](../qgc/README.md#planning-missions) 。