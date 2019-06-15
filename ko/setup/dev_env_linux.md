# 리눅스 개발환경

리눅스에서는 [모든 PX4 타겟](../setup/dev_env.md#supported-targets)에 대한 빌드가 가능합니다.(NuttX 기반 하드웨어, Qualcomm Snapdracon 비행제어보드, 리눅스 기반 기기, 시뮬레이션, ROS).

> **Tip** Ubuntu 버젼 16.04가 현재 지원/검증된 리눅스 디스트리뷰션입니다. Ubuntu 18.04 LTS with ROS Melodic is used for [ROS development](#ros). Instructions are also provided for [CentOS](../setup/dev_env_linux_centos.md) and [Arch Linux](../setup/dev_env_linux_arch.md).

The following instructions explain how to set up a development environment on Ubuntu LTS using convenience bash scripts. Instructions for *manually installing* these and additional targets can be found in [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md).

## 개발 툴체인

The instructions below show how you can use our [convenience bash scripts](../setup/dev_env_linux_ubuntu.md#convenience-bash-scripts) to setup the developer toolchain on Ubuntu LTS. All the scripts install the *Qt Creator IDE*, [Ninja Build System](https://ninja-build.org/), [Common Dependencies](../setup/dev_env_linux_ubuntu.md#common-dependencies), [FastRTPS](../setup/dev_env_linux_ubuntu.md#fastrtps-installation), and also download the PX4 source to your computer (**~/src/Firmware**).

> **Tip** The scripts have been tested on clean Ubuntu LTS 16.04 and Ubuntu LTS 18.04 installations. They *may* not work as expected if installed on top of an existing system or on another Ubuntu release. If you have any problems then follow the [manual installation instructions](../setup/dev_env_linux_ubuntu.md).

First make the user a member of the group "dialout":

1. On the command prompt enter: 
        sh
        sudo usermod -a -G dialout $USER

2. Logout and login again (the change is only made after a new login).

Then follow the instructions for your development target in the sections below.

### Pixhawk/NuttX (and jMAVSim)

To install the development toolchain:

1. Download the script in a bash shell: 
        bash
        wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh

2. 스크립트 실행: 
        bash
        source ubuntu_sim_nuttx.sh You may need to acknowledge some prompts as the script progresses.

3. Restart the computer on completion.

### Snapdragon Flight

Setup instructions for Snapdragon Flight are provided in the *PX4 User Guide*:

* [Development Environment](https://docs.px4.io/en/flight_controller/snapdragon_flight_dev_environment_installation.html)
* [소프트웨어 설치](https://docs.px4.io/en/flight_controller/snapdragon_flight_software_installation.html)
* [설정](https://docs.px4.io/en/flight_controller/snapdragon_flight_configuration.html)

### 라즈베리파이

To install the development toolchain:

1. Download the script in a bash shell (this contains the jMAVSim simulator and common toolchain dependencies): 
        bash
        wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh

2. Run the script: 
        bash
        source ubuntu_sim_common_deps.sh You may need to acknowledge some prompts as the script progresses.

3. Follow setup instructions in [Ubuntu/Debian Linux](../setup/dev_env_linux_ubuntu.md) for [Raspberry Pi](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).

### Parrot Bepop

Follow the (manual) instructions here: [Ubuntu/Debian Linux > Parrot Bebop](../setup/dev_env_linux_ubuntu.md#raspberry-pi-hardware).

### jMAVSim/Gazebo Simulation

To install the Gazebo9 and jMAVSim simulators:

1. Download the script in a bash shell: 
        bash
        wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim.sh

2. Run the script: 
        bash
        source ubuntu_sim.sh You may need to acknowledge some prompts as the script progresses.

> **Tip** If you just need jMAVSim, instead download and run <a href="https://raw.githubusercontent.com/PX4/Devguide/{{ book.px4_version }}/build_scripts/ubuntu_sim_common_deps.sh" target="_blank" download>ubuntu_sim_common_deps.sh</a>.

<span><span></p> 

<blockquote>
  <p>
    <strong>Note</strong> PX4 works with Gazebo 7, 8, and 9. The script installs Gazebo 9.
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
    Download the script in a bash shell: <pre><code>bash
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh</code></pre>
  </li>
  
  <li>
    Run the script: <pre><code>bash
source ubuntu_sim_ros_melodic.sh</code></pre> You may need to acknowledge some prompts as the script progresses.
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