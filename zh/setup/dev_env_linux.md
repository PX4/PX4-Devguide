# Linux环境下的开发

Linux允许您构建[所有PX4目标](../setup/dev_env.md#supported-targets)(基于NuttX的硬件、高通骁龙飞行硬件、基于Linux的硬件、仿真、ROS)。

> **Tip** 我们已经使用Debian/[Ubuntu](https://wiki.ubuntu.com/LTS) (16.04) 作为标准linux开发系统， 也为[CentOS](../setup/dev_env_linux_centos.md)和[Arch Linux](../setup/dev_env_linux_arch.md)提供了说明。

下文说明了如何使用方便的bash脚本在Ubuntu LTS上设置开发环境。 有关*手动安装*和其他目标的说明, 可参见[Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md)。

## 开发工具链

下问说明了如何使用[bash脚本](../setup/dev_env_linux_ubuntu.md#convenience-bash-scripts)在Ubuntu上设置开发工具链。 以下脚本作用分别是安装*Qt Creator IDE*、[ Ninja构建系统](https://ninja-build.org/)、[通用依赖项](../setup/dev_env_linux_ubuntu.md#common-dependencies)、[FastRTPS](../setup/dev_env_linux_ubuntu.md#fastrtps-installation), 以及将PX4源下载到您的目录(**~/src/Firmware**)。

> **Tip** 该脚本已经在全新Ubuntu 16.04安装测试过 如果安装在除上述提到的系统或其他Ubuntu版本上, 则它们*可能*无法正常工作。 如果您遇到任何问题, 请参照[手动安装说明](../setup/dev_env_linux_ubuntu.md)操作。

首先将用户加入组"dialout"

1. 在命令提示符下输入: 
        sh
        sudo usermod -a -G dialout $USER

2. 注销并重新登录(更改后重新登录生效)。

请对应以下各部分中的开发目标说明进行操作。

### Pixhawk/NuttX（和jMAVSim）

安装开发工具链:

1. 下载 <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx</a>。
2. 在bash shell中运行脚本: 
        bash
        source ubuntu_sim_nuttx.sh 随着脚本的运行，可能需要确认一些提示。

3. 完成后重新启动计算机。

### 高通骁龙飞控

在*PX4用户指南*中提供了高通骁龙飞控的安装说明:

* [开发环境](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [软件安装](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [配置](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

### 树莓派

安装开发工具链:

1. 下载<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps</a>(包含jMAVSim模拟器和常见工具链依赖)。
2. 在 bash shell 中运行脚本: 
        bash
        source ubuntu_sim_common_deps.sh 随着脚本的运行，可能需要确认一些提示。

3. 按照[树莓Pi](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware)在[Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md)中的安装说明进行。

### Parrot Bepop

请按照此处的(手动)说明操作: [ Ubuntu/Debian Linux >Parrot Bebop](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware)。

### jMAVSim/Gazebo 模拟

安装Gazebo和jMAVSim模拟器:

1. 下载<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a>。
2. 在bash shell中运行脚本: 
        bash 
        source ubuntu_sim.sh 随着脚本的运行，可能需要确认一些提示。

> **Tip** 如果您只需要jMAVSim，请下载并运行<a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps</a>。

<span><span></p> 

<blockquote>
  <p>
    <strong>Note</strong> PX4兼容Gazebo7、8和9。 该脚本安装Gazebo 9.
  </p>
</blockquote>

<h3>
  Gazebo与 ROS
</h3>

<p>
  安装开发工具链:
</p>

<ol start="1">
  <li>
    下载 <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh" target="_blank" download> ubuntu_sim_ros_gazebo</a>
  </li>
  
  <li>
    在bash shell中运行脚本: <pre><code>bash
source ubuntu_sim_ros_gazebo. sh</code></pre> 你可能需要承认一些随着脚本的进展而提示.
  </li>
</ol>

<p>
  Note:
</p>

<ul>
  <li>
    ROS默认安装Gazebo7 (我们选择使用默认值而不是Gazebo8或Gazebo9, 以简化ROS的开发).
  </li>
  <li>
    Catkin(ROS 生成系统)工作区创建于<strong>～/catkin_ws/</strong>.
  </li>
</ul>

<h2>
  其他工具
</h2>

<p>
  设置生成/模拟工具链后, 请参阅<a href="../setup/generic_dev_tools.md">其他工具</a>以了解有关其他有用的工具.
</p>

<h2>
  下一步
</h2>

<p>
  在完成环境设置后, 请继续执行<a href="../setup/building_px4.md">生成说明</a>.
</p>