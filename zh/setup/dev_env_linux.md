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

2. Logout and login again (the change is only made after a new login).

Then follow the instructions for your development target in the sections below.

### Pixhawk/NuttX (and jMAVSim)

To install the development toolchain:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh" target="_blank" download>ubuntu_sim_nuttx.sh</a>.
2. Run the script in a bash shell: 
        bash
        source ubuntu_sim_nuttx.sh You may need to acknowledge some prompts as the script progresses.

3. Restart the computer on completion.

### Snapdragon Flight

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:

* [Development Environment](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [Software Installation](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [Configuration](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

### Raspberry Pi

To install the development toolchain:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a> (this contains the jMAVSim simulator and common toolchain dependencies).
2. Run the script in a bash shell: 
        bash
        source ubuntu_sim_common_deps.sh You may need to acknowledge some prompts as the script progresses.

3. Follow setup instructions in [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md) for [Raspberry Pi](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).

### Parrot Bepop

Follow the (manual) instructions here: [Ubuntu/Debian Linux > Parrot Bebop](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).

### jMAVSim/Gazebo Simulation

To install the Gazebo and jMAVSim simulators:

1. Download <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh" target="_blank" download>ubuntu_sim.sh</a>.
2. Run the script in a bash shell: 
        bash
        source ubuntu_sim.sh You may need to acknowledge some prompts as the script progresses.

> **Tip** If you just need jMAVSim, instead download and run <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a>.

<span><span></p> 

<blockquote>
  <p>
    <strong>Note</strong> PX4 works with Gazebo 7, 8, and 9. The script installs Gazebo 9.
  </p>
</blockquote>

<h3>
  Gazebo with ROS
</h3>

<p>
  To install the development toolchain:
</p>

<ol start="1">
  <li>
    Download <a href="https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh" target="_blank" download>ubuntu_sim_ros_gazebo.sh</a>.
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
    ROS is installed with Gazebo7 by default (we have chosen to use the default rather than Gazebo8 or Gazebo9 to simplify ROS development).
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