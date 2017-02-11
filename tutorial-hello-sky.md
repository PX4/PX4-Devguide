# First App Tutorial (Hello Sky)

This tutorial explains in detail how to create a new onboard application and how to run it.

## Prerequisites

  * Pixhawk or Snapdragon compatible autopilot
  * PX4 Toolchain [installed](starting-installing.md)
  * Github Account ([sign up for free](https://github.com/signup/free))

## Step 1: File Setup

To conveniently manage your custom code and pull in updates from the main repository, it is recommended to fork the Firmware repository with the GIT version control system:

  - [Sign up](https://github.com/signup/free) for Github
  - Go to the [Firmware repository website](https://github.com/px4/Firmware/) and click **FORK** on the upper right part.
  - If you are not already there, open the website of your fork and copy the private repository URL in the center.
  - Clone the repository to your hard drive, e.g. on the command line via `git clone https://github.com/<youraccountname>/Firmware.git`. Windows users please [refer to the Github help](https://help.github.com/articles/set-up-git#platform-windows) and e.g. fork / clone with their Github for Windows app.
  - Update the git submodules: Run in your shell (on Windows in the PX4 console). 

<div class="host-code"></div>

```sh
cd Firmware
git submodule init
git submodule update --recursive
```
  
Enter the `Firmware/src/examples/` directory on your local hard drive and look at the files in the directory.

## Step 2: Minimal Application

Create a new C file named `px4_simple_app.c` in the `px4_simple_app` folder (it will already be present, delete the existing file for the maximum educational effect).

Edit it and start with the default header and a main function.

<aside class="tip">
Note the code style in this file - all contributions to PX4 should adhere to it.
</aside>

```C
/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

## Step 3: Register the Application in NuttShell and build it

The application is now complete and could be run, but it is not registered as NuttShell command yet. To enable the compilation of the application into the firmware, add it to the list of modules to build, which is here: 

  * Posix SITL: [Firmware/cmake/configs/posix_sitl_default.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/posix_sitl_default.cmake)
  * Pixhawk v1/2: [Firmware/cmake/configs/nuttx_px4fmu-v2_default.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/nuttx_px4fmu-v2_default.cmake)
  * Pixracer: [Firmware/cmake/configs/nuttx_px4fmu-v4_default.cmake](https://github.com/PX4/Firmware/blob/master/cmake/configs/nuttx_px4fmu-v4_default.cmake)

Create a new line for your application somewhere in the file:

  `examples/px4_simple_app`

Build it:

  * Pixhawk v1/2: `make px4fmu-v2_default`
  * Pixhawk v3: `make px4fmu-v4_default`

## Step 4: Upload and Test the app

Enable the uploader and then reset the board:

  * Pixhawk v1/2: `make px4fmu-v2_default upload`
  * Pixhawk v3: `make px4fmu-v4_default upload`

It should print before you reset the board a number of compile messages and at the end:

  `Loaded firmware for X,X, waiting for the bootloader...`

Once the board is reset, and uploads, it prints:

<div class="host-code"></div>

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

### Connect the console

Now connect to the [system console](advanced-system-console.md) either via serial or USB. Hitting ENTER will bring up the shell prompt:

```sh
  nsh>
```


Type ''help'' and hit ENTER

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

Note that `px4_simple_app` is now part of the available commands. Start it by typing `px4_simple_app` and ENTER:

```sh
  nsh> px4_simple_app
  Hello Sky!
```

The application is now correctly registered with the system and can be extended to actually perform useful tasks.

## Step 5: Subscribing Sensor Data

To do something useful, the application needs to subscribe inputs and publish outputs (e.g. motor or servo commands). Note that the *true* hardware abstraction of the PX4 platform comes into play here -- no need to interact in any way with sensor drivers and no need to update your app if the board or sensors are updated.

Individual message channels between applications are called *topics* in PX4. For this tutorial, we are interested in the [sensor_combined](https://github.com/PX4/Firmware/blob/master/src/modules/uORB/topics/sensor_combined.h) [topic](advanced-uorb.md), which holds the synchronized sensor data of the complete system.

Subscribing to a topic is swift and clean:

```C++
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
```

The `sensor_sub_fd` is a topic handle and can be used to very efficiently perform a blocking wait for new data. The current thread goes to sleep and is woken up automatically by the scheduler once new data is available, not consuming any CPU cycles while waiting. To do this, we use the [poll()](http://pubs.opengroup.org/onlinepubs/007908799/xsh/poll.html) POSIX system call.

Adding `poll()` to the subscription looks like (*pseudocode, look for the full implementation below*):

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
		PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					(double)raw.accelerometer_m_s2[0],
					(double)raw.accelerometer_m_s2[1],
					(double)raw.accelerometer_m_s2[2]);
	}
}
```

Compile the app now by issuing

<div class="host-code"></div>

```
  make
```

### Testing the uORB Subscription

The final step is to start your application as background application.

```
  px4_simple_app &
```

Your app will flood the console with the current sensor values:

```
  [px4_simple_app] Accelerometer:   0.0483          0.0821          0.0332
  [px4_simple_app] Accelerometer:   0.0486          0.0820          0.0336
  [px4_simple_app] Accelerometer:   0.0487          0.0819          0.0327
  [px4_simple_app] Accelerometer:   0.0482          0.0818          0.0323
  [px4_simple_app] Accelerometer:   0.0482          0.0827          0.0331
  [px4_simple_app] Accelerometer:   0.0489          0.0804          0.0328
```

It will exit after printing five values. The next tutorial page will explain how to write a deamon which can be controlled from the commandline.

## Step 7: Publishing Data

To use the calculated outputs, the next step is to *publish* the results. If we use a topic from which we know that the ''mavlink'' app forwards it to the ground control station, we can even look at the results. Let's hijack the attitude topic for this purpose.

The interface is pretty simple: Initialize the struct of the topic to be published and advertise the topic:

```C
#include <uORB/topics/vehicle_attitude.h>
..
/* advertise attitude topic */
struct vehicle_attitude_s att;
memset(&att, 0, sizeof(att));
orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
```

In the main loop, publish the information whenever its ready:

```C
orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);
```

The modified complete example code is now:

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
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

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
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);

				/* set att and publish this information for other apps */
				att.roll = raw.accelerometer_m_s2[0];
				att.pitch = raw.accelerometer_m_s2[1];
				att.yaw = raw.accelerometer_m_s2[2];
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

## Running the final example

And finally run your app:

```sh
  px4_simple_app
```

If you start QGroundControl, you can check the sensor values in the real time plot (Tools -> Analyze)

## Wrap-Up

This tutorial covered everything needed to develop a "grown up" PX4 autopilot application. Keep in mind that the full list of uORB messages / topics is [available here](https://github.com/PX4/Firmware/tree/master/msg/) and that the headers are well documented and serve as reference.
