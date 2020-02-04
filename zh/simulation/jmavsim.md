# jMAVSim 进行 SITL 仿真

jMAVSim是一个简单的多旋翼/四旋翼仿真软件，它可以允许你在仿真环境中飞行运行着 PX4 的 *旋翼* 无人机。 它很容易设置，可以用来测试您的工具是否可以起飞、飞行、降落、并对各种故障条件 (例如 gps 故障) 做出适当的反应。

<strong>支持机型：</strong>

* 四旋翼

本问主要演示如何设置 jMAVSim 以连接到 SITL 版本的 PX4 。

> **Tip**jMAVSim 也可以用 HITL 仿真([看这里](../simulation/hitl.md#jmavsimgazebo-hitl-environment)

## 仿真环境

软件在环仿真在主机上运行仿真系统的全部组件，使用软件来模拟真实飞控， 并通过当地网络实现与仿真软件的连接。 整套仿真方案设置如下：

{% mermaid %} graph LR 仿真器-->MAVLink MAVLink-->SITL {% endmermaid %}

## 运行 SITL

在确保 [仿真环境](../setup/dev_env.md) 已经搭建在电脑上了之后直接运行命令：便捷的 make target 命令会完成 POSIX 平台的交叉编译并启动仿真。

```sh
make px4_sitl_default jmavsim
```

该命令最终将得到如下 PX4 控制台显示界面：

```sh
[init] shell id: 140735313310464
[init] task name: px4

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

Ready to fly.


pxh>
```

此外，它还会打开一个显示 [jMAVSim](https://github.com/PX4/jMAVSim) 仿真器的3D视图的界面。

![jMAVSim 3d 视图](../../assets/simulation/jmavsim.jpg)

## 简单飞上天

系统将开始输出状态信息。 飞机完成位置锁定之后（控制台立刻提示： *EKF commencing GPS fusion* 信息之后不久 ）就可以开始飞行了。

在控制台输入以下命令进行起飞：

```sh
pxh> commander takeoff
```

你可以使用 *QGroundControl* 规划一个飞行任务，或者连接一个 [操纵杆](#joystick) 。

## 可选参数配置

### 指定起飞位置

手动设定环境变量可以覆盖默认的起飞点坐标： `PX4_HOME_LAT`, `PX4_HOME_LON`, 以及 `PX4_HOME_ALT` 。

例如，要设置飞机的维度、经度和海拔高度：

    export PX4_HOME_LAT=28.452386
    export PX4_HOME_LON=-13.867138
    export PX4_HOME_ALT=28.5
    make px4_sitl_default jmavsim
    

### 更改仿真的时间流速

可以使用环境变量 `PX4_SIM_SPEED_FACTOR` 增长或者缩短仿真环境的时间流速相对于实际时间流速的大小。

    export PX4_SIM_SPEED_FACTOR=2
    make px4_sitl_default jmavsim
    

更多相关信息请参考：[Simulation > Run Simulation Faster than Realtime](../simulation/README.md#simulation_speed).

### 使用游戏手柄 {#joystick}

通过 *QGroundControl* 可引入游戏手柄或者拇指操纵杆（[如何进行设置看这里](../simulation/README.md#joystickgamepad-integration)）。

### 模拟一个 Wifi 无人机

有一个特殊的平台可以模拟通过本地 Wifi 网络进行连接无人机。

```sh
make broadcast jmavsim
```

模拟器会跟真实的该类无人机一样在当地网络中广播自己的位置信息等。

### 分别启动 JMAVSim 和 PX4

你可以单独启动 JMAVSim 和 PX4:

    ./Tools/jmavsim_run.sh -l
    make px4_sitl none
    

此举可以缩短测试循环时间（重启 jMAVSim 需要耗费非常多的时间）。

### 无航向模式

要使用没有jMAVSim图形界面的仿真，请设置如下环境变量 `HEADLESS=1`

```bash
HEADLESS=1 make px4_sitl jmavsim
```

## 多机仿真

JMAVSim也常用做多机仿真，更多请参考：[JMAVSim多机当真](../simulation/multi_vehicle_jmavsim.md)

## 扩展和自定义

如果要扩展或自定义仿真接口，可以编辑 **Tools/jMAVSim** 里面文件。 源码可以从 Github 上的 [jMAVSim 库](https://github.com/px4/jMAVSim) 获取。

> **Info** 编译系统会强制检查所有依赖项的子模块正确无误，其中就包括了模拟器。 但是，它不会直接覆盖你对目录中文件所做的更改， 当提交这些更改时你需要在 固件 库中重新为子模块注册新的哈希值。 为此,，使用 `git add Tools/jMAVSim` 并提交你的更改。 这就会更新仿真器中 GIT 哈希值。

## ROS 交互

仿真器中跟真实飞机一样方式实现 [与 ROS 交互](../simulation/ros_interface.md) 。

## 值得一看的脚本文件

* The startup scripts are discussed in [System Startup](../concept/system_startup.md).
* 已仿真的根文件系统 ("`/`" 目录) 是在编译文件里面生成的： `build/px4_sitl_default/tmp/rootfs`

## 常见问题处理

### java.long.NoClassDefFoundError

如果你遇到如下的错误，原因可能是你的 Java 版本高于8：

    Exception in thread "main" java.lang.NoClassDefFoundError: javax/vecmath/Tuple3d
    at java.base/java.lang.Class.forName0(Native Method)
    at java.base/java.lang.Class.forName(Class.java:374)
    at org.eclipse.jdt.internal.jarinjarloader.JarRsrcLoader.main(JarRsrcLoader.java:56)
    Caused by: java.lang.ClassNotFoundException: javax.vecmath.Tuple3d
    at java.base/java.net.URLClassLoader.findClass(URLClassLoader.java:466)
    at java.base/java.lang.ClassLoader.loadClass(ClassLoader.java:566)
    at java.base/java.lang.ClassLoader.loadClass(ClassLoader.java:499)
    ... 3 more
    Exception in thread "main" java.lang.NoClassDefFoundError: javax/vecmath/Tuple3d
    at java.base/java.lang.Class.forName0(Native Method)
    at java.base/java.lang.Class.forName(Class.java:374)
    at org.eclipse.jdt.internal.jarinjarloader.JarRsrcLoader.main(JarRsrcLoader.java:56)
    Caused by: java.lang.ClassNotFoundException: javax.vecmath.Tuple3d
    at java.base/java.net.URLClassLoader.findClass(URLClassLoader.java:466)
    at java.base/java.lang.ClassLoader.loadClass(ClassLoader.java:566)
    at java.base/java.lang.ClassLoader.loadClass(ClassLoader.java:499)
    

更多详情，请查阅 [GitHub issue](https://github.com/PX4/Firmware/issues/9557) 。

解决方法是安装 Java 8 ，如下各小节所示。

#### Ubuntu：

    sudo apt install openjdk-8-jdk
    sudo update-alternatives --config java # 选择Java 8 的指令
    rm -rf Tools/jMAVSim/out
    

#### Mac系统：

We recommend to install OpenJDK 8 from [AdoptOpenJDK](https://adoptopenjdk.net/) using brew:

    brew tap adoptopenjdk/openjdk
    brew cask install adoptopenjdk8
    brew install ant
    export JAVA_HOME=$(/usr/libexec/java_home -v 1.8)
    rm -rf Tools/jMAVSim/out
    

Alternatively you could [download Oracle Java 8](https://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html) and install it manually.

### An illegal reflective access operation has occured

If you see an error similar to the one below, it's likely that you're using a Java version later than 8:

    WARNING: An illegal reflective access operation has occurred
    WARNING: Illegal reflective access by javax.media.j3d.JoglPipeline (rsrc:j3dcore.jar) to method sun.awt.AppContext.getAppContext()
    WARNING: Please consider reporting this to the maintainers of javax.media.j3d.JoglPipeline
    WARNING: Use --illegal-access=warn to enable warnings of further illegal reflective access operations
    WARNING: All illegal access operations will be denied in a future release
    Inconsistency detected by ld.so: dl-lookup.c: 112: check_match: Assertion version->filename == NULL || ! _dl_name_match_p (version->filename, map)' failed!
    

Follow the steps above to make sure Java 8 is installed and selected.

### java.awt.AWTError: Assistive Technology not found: org.GNOME.Accessibility.AtkWrapper

    Exception in thread "main" java.lang.reflect.InvocationTargetException
    at sun.reflect.NativeMethodAccessorImpl.invoke0(Native Method)
    at sun.reflect.NativeMethodAccessorImpl.invoke(NativeMethodAccessorImpl.java:62)
    at sun.reflect.DelegatingMethodAccessorImpl.invoke(DelegatingMethodAccessorImpl.java:43)
    at java.lang.reflect.Method.invoke(Method.java:498)
    at org.eclipse.jdt.internal.jarinjarloader.JarRsrcLoader.main(JarRsrcLoader.java:58)
    Caused by: java.awt.AWTError: Assistive Technology not found: org.GNOME.Accessibility.AtkWrapper
    at java.awt.Toolkit.loadAssistiveTechnologies(Toolkit.java:807)
    at java.awt.Toolkit.getDefaultToolkit(Toolkit.java:886)
    at java.awt.Window.getToolkit(Window.java:1358)
    at java.awt.Window.init(Window.java:506)
    at java.awt.Window.(Window.java:537)
    at java.awt.Frame.(Frame.java:420)
    at java.awt.Frame.(Frame.java:385)
    at javax.swing.JFrame.(JFrame.java:189)
    at me.drton.jmavsim.Visualizer3D.(Visualizer3D.java:104)
    at me.drton.jmavsim.Simulator.(Simulator.java:157)
    at me.drton.jmavsim.Simulator.main(Simulator.java:678)
    

If you see this error, try this workaround:

Edit the **accessibility.properties** file:

    sudo gedit /etc/java-8-openjdk/accessibility.properties
    

and comment out the line indicated below:

    #assistive_technologies=org.GNOME.Acessibility.AtkWrapper
    

For more info check [this GitHub issue](https://github.com/PX4/Firmware/issues/9557). The fix was found in [askubuntu.com](https://askubuntu.com/questions/695560).