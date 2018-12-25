# 硬件在环仿真（HITL）

硬件在环仿真模式 (HITL 或 HIL) 下 PX4 固件代码运行在真实的飞行控制器硬件平台上。 这种方法的优点是可以在实际硬件上测试大多数的实际飞行代码。

HITL 模式下 PX4 支持多旋翼 (使用 jMAVSim 或者 Gazebo) 和固定翼 (使用 Gazebo 或者 X-Plane demo/full version) 无人机的仿真。

## HITL兼容机架 {#compatible_airframe}

目前兼容的机架构型和模拟器的情况如下：

| 机架                                                                                                      | `SYS_AUTOSTART` | X-Plane | Gazebo | jMAVSim |
| ------------------------------------------------------------------------------------------------------- | --------------- | ------- | ------ | ------- |
| <a href="../airframes/airframe_reference.md#plane_simulation_(plane)_hilstar_(xplane)">HILStar (X-Plane)</a>                                                                               | 1000            | Y       |        |         |
| <a href="../airframes/airframe_reference.md#copter_simulation_(copter)_hil_quadcopter_x">HIL Quadcopter X</a>                                                                               | 1001            |         | Y      | Y       |
| <a href="../airframes/airframe_reference.md#vtol_standard_vtol_hil_standard_vtol_quadplane">HIL Standard VTOL QuadPlane</a>                                                                               | 1002            |         | Y      |         |
| [Standard planes](../airframes/airframe_reference.md#plane_standard_plane_standard_plane)               | 2100            | Y       |        |         |
| [Generic Quadrotor x](../airframes/airframe_reference.md#copter_quadrotor_x_generic_quadrotor_x) copter | 4001            |         | Y      | Y       |
| [DJI Flame Wheel f450](../airframes/airframe_reference.md#copter_quadrotor_x_dji_flame_wheel_f450)      | 4011            |         | Y      | Y       |

## HITL 仿真环境 {#simulation_environment}

硬件在环仿真（HITL）模式下标准的 PX4 固件在真实的硬件上运行。 使用Gazebo, jMAVSim 和 X-Plane 仿真平台进行 HITL 仿真的配置设定稍有不同。

> **Tip** 更多有关信息请参阅： [仿真模拟](../simulation/README.md) 。

### JMAVSim/Gazebo HITL 仿真环境

JMAVSim 或 Gazebo (运行在开发计算机上) 通过 USB/UART 完成与飞行控制器硬件平台的连接。 模拟器充当在 PX4 和 *QGroundControl* 之间共享 MAVLink 数据的网关。

> **Note** 如果飞行控制器支持网络连接且使用的是稳定、低延迟的连接（如有线以太网，WIFI 通常不太稳定），那么模拟器也可以使用 UDP 完成通讯连接。 例如，该配置已经使用一台运行 PX4 且通过以太网连接到开发计算机的 Raspberry Pi 进行了验证测试 (包括 jMAVSim 运行命令的启动配置在 [这里](https://github.com/PX4/Firmware/blob/master/posix-configs/rpi/px4_hil.config))。

<span></span>

> **Tip** Gazebo 还支持与offboard API 共享 MAVLink 数据！

下图展示了仿真模拟的环境：

* 飞控板 HITL 模式被激活 (通过 *QGroundControl*) ，该模式下不会启动飞控板上任何传感器。
* *jMAVSim* 或者 *Gazebo* 通过 USB 连接到飞控板。
* 模拟器通过 UDP 连接到 *QGroundControl* 并将 MAVLink 数据传输至 PX4 。
* (可选) 通过串口可将操纵杆/游戏手柄通过 *QGroundControl* 连接至仿真回路中。
* (可选 - 仅适用于Gazebo) Gazebo 还可以连接到一个 offboard API ，并将 MAVLink 数据桥接到 PX4 。

![HITL Setup - jMAVSim and Gazebo](../../assets/simulation/px4_hitl_overview_jmavsim_gazebo.png)

### X-Plane HITL 仿真环境

*QGroundControl* 通过 USB 连接至飞控板硬件平台，并作为一个网关为在开发计算机上运行着的X-Plane 模拟器、 PX4 和任意 offboard API 三个平台进行数据中转。 下图展示了仿真模拟的环境：

* 飞控板 HITL 模式被激活 (通过 *QGroundControl*) ，该模式下不会启动飞控板上任何传感器。
* *QGroundControl* 通过 USB 连接到飞行控制器。
* *QGroundControl* 通过 UDP 连接到模拟器和offboard API。
* 通过串口将操纵杆/游戏手柄通过 *QGroundControl* 连接至仿真回路中。

![HITL Setup - X-Plane](../../assets/simulation/px4_hitl_overview_xplane.png)

## HITL vs SITL

SITL 开发计算机中的模拟环境中运行, 并使用专门为该环境生成的固件。 除了仿真程序从模拟器中获取虚假的环境数据外，系统的行为也很正常。

相比之下， HITL 在正常飞控硬件平台上运行正常的处于 ”HITL 模式“ 的 PX4 固件。 仿真数据进入整个仿真系统的时间点与 SITL 有所不同。 指令器和传感器等有 HIL 模式的核心模块在启动时被绕过了一些正常的功能。

总而言之， HITL 在真实硬件上运行标准 PX4 固件，而 SITL 实际上要比标准 PX4 系统执行更多的代码。

## 配置 HITL

### PX4 配置

1. 通过 USB 将自动驾驶仪直接连接到 *QGroundControl*。
2. 激活 HITL 模式
    
    1. 打开 **Setup > Safety** 选项卡。
    2. 在 *HITL Enabled* 下拉框中选择 **Enabled** 完成 HITL 模式的激活。
        
        ![QGroundControl HITL configuration](../../assets/gcs/qgc_hitl_config.png)

3. 选择机架
    
    1. 打开 **Setup > Airframes** 选项卡。
    2. 选择一个你想要进行测试的 [兼容的机架](#compatible_airframe) 。 通常情况下选择 *HILStar* 作为固定翼平台/X-Plane 模拟器，选择 *HIL QuadCopter* 作为旋翼平台 ( jMAVSim 或 Gazebo) 。 然后单击 "机身设置" 页面右上角的 "Apply and Restart"。
        
        ![Select Airframe](../../assets/gcs/qgc_hil_config.png)

4. 如有必要, 校准您的 RC 遥控器 或操纵杆。

5. 设置 UDP
    
    1. 在设置菜单的 "*General*" 选项卡下, 取消选中 *AutoConnect* 一栏中除 **UDP** 外的所有复选框。
        
        ![QGC Auto-connect settings for HITL](../../assets/gcs/qgc_hitl_autoconnect.png)

6. (可选) 配置操纵杆和故障保护。 设置以下 [parameters](https://docs.px4.io/en/advanced_config/parameters.html#finding-a-parameter) 以便使用操纵杆而不是 RC 遥控器：
    
    * [COM_RC_IN_MODE](../advanced/parameter_reference.md#COM_RC_IN_MODE) 更改为 "Joystick/No RC Checks". 这允许操纵杆输入并禁用 RC 输入检查。
    * [NAV_DLL_ACT](../advanced/parameter_reference.md#NAV_DLL_ACT) 更改为 "Disabled"。 这可确保在没有无线遥控的情况下运行 HITL 时 RC 失控保护不会介入。
    
    > **Tip** *QGroundControl User Guide* 中也有如何配置 [操纵杆](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html) 和 [虚拟操纵杆](https://docs.qgroundcontrol.com/en/SettingsView/VirtualJoystick.html) 的说明。

完成所有的配置设定后 **关闭** *QGroundControl* 并断开飞控板与计算机的连接。

### 模拟器配置

按照下面的小节对你的模拟器进行合理的设置。

#### Gazebo

> **Note** 确保 *QGroundControl* 没有运行！

1. 更新环境变量：
    
    ```sh
    cd <Firmware_clone>
    make px4_sitl_default gazebo
    ```
    
    在新的终端窗口运行：
    
    ```sh
    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    ```

2. 打开载具模型的 sdf 文件（例如 **Tools/sitl_gazebo/models/iris/iris.sdf**）。

3. Under the `mavlink_interface plugin` section, change the `serialEnabled` and `hil_mode` parameters to `true`.
    
    ![HIL Parameters](../../assets/simulation/gazebo_sdf_model_hil_params.png)

4. Replace the `serialDevice` parameter (`/dev/ttyACM0`) if necessary.
    
    > **Note** The serial device depends on what port is used to connect the vehicle to the computer (this is usually `/dev/ttyACM0`). An easy way to check on Ubuntu is to plug in the autopilot, open up a terminal, and type `dmesg | grep "tty"`. The correct device will be the last one shown.

5. Close Gazebo, connect the flight controller to the computer and wait for it to boot.

6. Run Gazebo in HITL mode 
        sh
        gazebo Tools/sitl_gazebo/worlds/iris.world

7. Start *QGroundControl*. It should autoconnect to PX4 and Gazebo.

#### jMAVSim (仅适用于四旋翼无人机)

> **Note** Make sure *QGroundControl* is not running!

1. Connect the flight controller to the computer and wait for it to boot.
2. Run jMAVSim in HITL mode (replace the serial port name `/dev/ttyACM0` if necessary - e.g. on Mac OS this would be `/dev/tty.usbmodem1`): 
        sh
        ./Tools/jmavsim_run.sh -q -d /dev/ttyACM0 -b 921600 -r 250

3. Start *QGroundControl*. It should autoconnect to PX4 and jMAVSim.

#### 使用 X-Plane (仅适用于固定翼无人机)

> **Note** X-Plane is currently not recommended. Among other issues, the frame update rate is too slow to run the system realistically.

To set up X-Plane:

1. Open X-Plane
2. In **Settings > Data Input and Output**, set these checkboxes:
    
    ![X-Plane data configuration](../../assets/gcs/xplane_data_config.png)

3. In **Settings > Net Connections**, in the *Data* tab, set localhost and port 49005 as IP address, as shown below:
    
    ![X-Plane 网络配置](../../assets/gcs/xplane_net_config.png)

4. Enable X-Plane HITL in *QGroundControl*:
    
    1. Open *QGroundControl*
    2. Open **Widgets > HIL Config**. Select X-Plane 10 in the drop-down and hit connect. Once the system is connected, battery status, GPS status and aircraft position should all become valid:
        
        ![](../../assets/gcs/qgc_sim_run.png)

## 在 HITL 仿真中执行自主飞行任务

You should be able to use *QGroundControl* to [run missions](../qgc/README.md#planning-missions) and otherwise control the vehicle.