# Linux环境下的开发

Linux允许您构建[所有PX4目标](../setup/dev_env.md#supported-targets)(基于NuttX的硬件、高通骁龙飞行硬件、基于Linux的硬件、仿真、ROS)。

> **Tip** [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 16.04 is the tested/supported Linux distribution for most development. Ubuntu 18.04 LTS with ROS Melodic is used for [ROS development](#ros). Instructions are also provided for [CentOS](../setup/dev_env_linux_centos.md) and [Arch Linux](../setup/dev_env_linux_arch.md).

下文说明了如何使用方便的bash脚本在Ubuntu LTS上设置开发环境。 有关*手动安装*和其他目标的说明, 可参见[Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md)。

## 开发工具链

下问说明了如何使用[bash脚本](../setup/dev_env_linux_ubuntu.md#convenience-bash-scripts)在Ubuntu上设置开发工具链。 以下脚本作用分别是安装*Qt Creator IDE*、[ Ninja构建系统](https://ninja-build.org/)、[通用依赖项](../setup/dev_env_linux_ubuntu.md#common-dependencies)、[FastRTPS](../setup/dev_env_linux_ubuntu.md#fastrtps-installation), 以及将PX4源下载到您的目录(**~/src/Firmware**)。

> **Tip** The scripts have been tested on clean Ubuntu LTS 16.04 and Ubuntu LTS 18.04 installations. 如果安装在除上述提到的系统或其他Ubuntu版本上, 则它们*可能*无法正常工作。 如果您遇到任何问题, 请参照[手动安装说明](../setup/dev_env_linux_ubuntu.md)操作。

First make the user a member of the group "dialout":

1. 在命令提示符下输入: 
        sh
        sudo usermod -a -G dialout $USER

2. 注销并重新登录(更改后重新登录生效)。

请对应以下各部分中的开发目标说明进行操作。

### Pixhawk/NuttX（和jMAVSim）

安装开发工具链:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a>.
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

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a> (this contains the jMAVSim simulator and common toolchain dependencies).
2. 在 bash shell 中运行脚本: 
        bash
        source ubuntu_sim_common_deps.sh 随着脚本的运行，可能需要确认一些提示。

3. 按照[树莓Pi](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware)在[Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md)中的安装说明进行。

### Parrot Bepop

请按照此处的(手动)说明操作: [ Ubuntu/Debian Linux >Parrot Bebop](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware)。

### jMAVSim/Gazebo 模拟

To install the Gazebo9 and jMAVSim simulators:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a>.
2. 在bash shell中运行脚本: 
        bash 
        source ubuntu_sim.sh 随着脚本的运行，可能需要确认一些提示。

> **Tip** If you just need jMAVSim, instead download and run <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a>.

<span><span></p> 

<blockquote>
  <p>
    <strong>Note</strong> PX4兼容Gazebo7、8和9。 The script installs Gazebo 9.
  </p>
</blockquote>

<h3 id="ros">
  Gazebo with ROS Melodic
</h3>

<blockquote>
  <p>
    <strong>Note</strong> PX4 is tested with ROS Melodic on Ubuntu 18.04 LTS. ROS Melodic does not work on Ubuntu 16.04.
  </p>
</blockquote>

<p>
  To install the development toolchain:
</p>

<ol start="1">
  <li>
    Download <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_ros_melodic.sh" target="_blank" download>ubuntu_sim_ros_melodic.sh</a>.
  </li>
  
  <li>
    Run the script in a bash shell: <pre><code>bash
source ubuntu_sim_ros_gazebo.sh</code></pre> You may need to acknowledge some prompts as the script progresses.
  </li>
</ol>

<p>
  Note:
</p>

<ul>
  <li>
    ROS Melodic is installed with Gazebo9 by default.
  </li>
  <li>
    Your catkin (ROS build system) workspace is created at <strong>~/catkin_ws/</strong>.
  </li>
</ul>

<h2>
  Additional Tools
</h2>

<p>
  After setting up the build/simulation toolchain, see <a href="../setup/generic_dev_tools.md">Additional Tools</a> for information about other useful tools.
</p>

<h2>
  Next Steps
</h2>

<p>
  Once you have finished setting up the environment, continue to the <a href="../setup/building_px4.md">build instructions</a>.
</p>