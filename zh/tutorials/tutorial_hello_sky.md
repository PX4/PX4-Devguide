---
translated_page: https://github.com/PX4/Devguide/blob/master/en/tutorials/tutorial_hello_sky.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
translated: true
---

# 第一个应用程序教程(Hello Sky)
本教程详细解释了如何创建一个新的板载应用程序，以及如何运行它。

##前提条件

- Pixhawk 或者 Snapdragon兼容的自驾仪
- [安装](../setup/dev_env.md)PX4 工具链
- Github 账号  ([免费注册](https://github.com/signup/free))

## 第一步：文件设置 

为了更方便的管理你的个人代码和上传更新到原始代码仓库，强烈建议使用fork命令通过GIT版本控制系统得到一个新的PX4固件。

- 在github上[注册](https://github.com/signup/free )一个账户；
- 登陆[px4的固件托管网址](https://github.com/px4/Firmware/)，然后点击右上方的**FORK**按钮；
- 点击到你fork后的站点，复制你的私人PX4代码仓库的URL地址。
- 克隆你的代码仓库到你的硬盘中，在命令行中输入：
  `git clone https://github.com/<youraccountname>/Firmware.git`
  对于windows用户，请参考[github帮助](https://help.github.com/desktop/guides/getting-started-with-github-desktop/installing-github-desktop/)中的介绍。例如在github创建的官方应用程序中使用fork/clone命令克隆仓库到本地硬盘中。
- 更新px4代码包含的git子模块：运行命令行工具（在windows上运行PX4终端）


```
cd Firmware
git submodule init
git submodule update --recursive
```


打开你本地硬盘克隆的软件仓库的`Firmware/src/examples/`文件夹，查看里面的文件。


## 第二步：最小的应用


在`Firmware/src/examples/px4_simple_app`下，创建一个新的C语言文件`px4_simple_app.c`。(该文件已存在，你可以直接删掉它，来完全自主编辑学习。)


从默认头文件和main主函数开始，编辑这个文件。


> 注意这个文件中的代码风格，所有PX4的软件版本都将遵守这个风格。


```C
/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");
	return OK;
}
```


## 第三步：在NuttShell中注册应用程序，然后编译

现在应用程序已经完成并且能够运行，但还没有注册成NuttShell命令行工具。如果想要将应用程序编译到固件中，将它加到下面的模块编译列表中：

* PX4 SITL (Simulator): [Firmware/boards/px4/sitl/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/sitl/default.cmake)
* Pixhawk v1/2: [Firmware/boards/px4/fmu-v2/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v2/default.cmake)
* Pixracer (px4/fmu-v4): [Firmware/boards/px4/fmu-v4/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v4/default.cmake)
* *cmake* files for other boards can be found in [Firmware/boards/](https://github.com/PX4/Firmware/tree/master/boards)


在你的应用程序的下面的文件中添加一行：

  `examples/px4_simple_app`


编译它：


- Pixhawk v1/2: `make px4_fmu-v2_default`
- Pixhawk v3: `make px4_fmu-v4_default`


## 第四步：上传并且测试应用程序

使能uploader然后重置开发板：

- Pixhawk v1/2: `make px4_fmu-v2_default upload`
- Pixhawk v3: `make px4_fmu-v4_default upload`

在你重置开发板之前，会在后面打印如下的编译信息：

  `Loaded firmware for X,X, waiting for the bootloader...`

一旦开发板重置成功并且应用程序上传成功，打印信息：

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

### 连接到终端

现在通过串口或USB连接到[系统命令行终端](../debug/system_console.md) （新手玩家第一次进行USB或者串口连接系统命令行终端，请点开上面的链接，按照要求进行系统控制台安装，安装完毕后，输入回车键打开终端（linux）或者命令行工具（windows））。点击回车键能打开shell命令行工具：

```sh
  nsh>
```

输入“help”然后回车

```sh
  nsh> help
    help usage:  help [-v] [<cmd>]
  
    [           df          kill        mkfifo      ps          sleep       
    ?           echo        losetup     mkrd        pwd         test        
    cat         exec        ls          mh          rm          umount      
    cd          exit        mb          mount       rmdir       unset       
    cp          free        mkdir       mv          set         usleep      
    dd          help        mkfatfs     mw          sh          xd          
  
  Builtin Apps:
    reboot
    perf
    top
    ..
    px4_simple_app
    ..
    sercon
    serdis
```

现在‘px4_simple_app’已经是一个可用的命令了。输入`px4_simple_app`命令然后回车：

```sh
  nsh> px4_simple_app
  Hello Sky!
```

现在应用程序已经成功注册到系统中，能够被扩展并且运行一些有用的任务。

## 第五步：读取传感器数据

> 为了实现一些功能，应用程序需要读取传感器的输入然后反应到对电机或者舵机的输出中。请注意，PX4平台真正的硬件抽象的概念在这里体现--无需与传感器驱动程序以任何方式交互，如果你更新了主板或传感器，也无需更新应用程序。


在PX4中，应用程序之间的各个消息通道称为“topics”（话题）。在本教程中，我们关心的topic是“多传感器间的uORB消息机制”[sensor_combined](https://github.com/PX4/Firmware/blob/master/msg/sensor_combined.msg) [topic](../middleware/uorb.md)。这些消息机制使得整个系统能够同步传感器数据。

订阅一个话题是非常迅速并且简洁的：

```C++
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
```

“sensor_sub_fd” 是一个topic句柄，它能非常高效地为新数据执行阻塞等待。当有新数据产生时，一个正处于休眠状态对线程就会自动地被调度程序唤醒。因此在等待数据时，不会占用任何CPU周期。为了实现这个功能，我们使用[poll()](http://pubs.opengroup.org/onlinepubs/007908799/xsh/poll.html)函数，即POSIX系统调用。

在消息订阅中加入“poll()”函数，看起来如下（伪代码，完整程序代码可在下面找到）：

```C++
#include <poll.h>
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

/* one could wait for multiple topics with this technique, just using one here */
px4_pollfd_struct_t fds[] = {
	{ .fd = sensor_sub_fd,   .events = POLLIN },
};

while (true) {
	/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
	int poll_ret = px4_poll(fds, 1, 1000);
..
	if (fds[0].revents & POLLIN) {
		/* obtained data for the first file descriptor */
		struct sensor_combined_s raw;
		/* copy sensors raw data into local buffer */
		orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
		printf("[px4_simple_app] Accelerometer:\t%8.4f\t%8.4f\t%8.4f\n",
					(double)raw.accelerometer_m_s2[0],
					(double)raw.accelerometer_m_s2[1],
					(double)raw.accelerometer_m_s2[2]);
	}
}
```

编译应用程序：

<div class="host-code"></div>

```
  make
```

### 第六步：测试uORB消息读取机制

最后一步，启动你的应用程序作为后台应用：

```
  px4_simple_app &
```

你的应用程序会向串口输出当前传感器的值：

```
  [px4_simple_app] Accelerometer:   0.0483          0.0821          0.0332
  [px4_simple_app] Accelerometer:   0.0486          0.0820          0.0336
  [px4_simple_app] Accelerometer:   0.0487          0.0819          0.0327
  [px4_simple_app] Accelerometer:   0.0482          0.0818          0.0323
  [px4_simple_app] Accelerometer:   0.0482          0.0827          0.0331
  [px4_simple_app] Accelerometer:   0.0489          0.0804          0.0328
```

它会在输出5次数据后退出。下一篇教程中会介绍如何编写一个能通过命令行控制的后台应用。

## 第七部：发布数据

为了能获取到计算后的数据，下一步就是“发布”这些结果。如果我们知道某一个消息是使用mavlink协议转发给地面控制站的，就可以通过这个消息去查看结果。基于此目的，我们来拦截姿态topic。

接口非常简单:初始化即将发布的话题（topic）的结构，然后通告（advertise）这个话题：

```C
#include <uORB/topics/vehicle_attitude.h>
..
/* advertise attitude topic */
struct vehicle_attitude_s att;
memset(&att, 0, sizeof(att));
orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
```

在主循环中，当消息准备好时，发布这条消息：

```C
orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);
```
修改后的完整的示例代码如下：

```C
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	orb_set_interval(sensor_sub_fd, 1000);

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("[px4_simple_app] Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("[px4_simple_app] ERROR return value from poll(): %d"
				       , poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_WARN("[px4_simple_app] Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				       (double)raw.accelerometer_m_s2[0],
				       (double)raw.accelerometer_m_s2[1],
				       (double)raw.accelerometer_m_s2[2]);

				/* set att and publish this information for other apps */
				att.rollspeed = raw.accelerometer_m_s2[0];
				att.pitchspeed = raw.accelerometer_m_s2[1];
				att.yawspeed = raw.accelerometer_m_s2[2];
				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}
	PX4_INFO("exiting");

	return 0;
}
```

## 第八步：运行整个示例

运行你的应用程序：

```sh
  px4_simple_app
```
如果你打开了QGroundControl，你就能通过实时绘图程序plot(Tools -> Analyze)来获得传感器数据。

## 小结

这个教程包含了所有开发一个“增量式”的PX4自驾仪应用程序需要的东西。注意，关于uORB消息的完整列表可以从[这里](https://github.com/PX4/Firmware/tree/master/msg/)获得 。其中，已经写好了每个消息的标题，可作为参考。
