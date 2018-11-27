# 첫번째 App 만들기 튜토리얼 (Hello Sky)

여기서는 새로운 온보드 어플리케이션을 만드는 방법과 실행하는 방법을 상세하게 설명합니다.

## 전제 조건

  * Pixhawk나 Snapdragon 호환 autopilot
  * PX4 툴체인 [설치](../setup/dev_env.md)
  * Github 계정 ([무료 가입](https://github.com/signup/free))

## Step 1: 파일 셋업

편라히게 커스텀 코드를 관리하기 위해서 main 저장소에서 업데이트된 내용을 가지고 옵니다. GIT 버전 관리 시스템으로 Firmware 저장소를 fork하는 것을 추천합니다. :

  - Github [가입하기](https://github.com/signup/free)
  - [Firmware repository 사이트](https://github.com/px4/Firmware/)가서 오른쪽 상단의 **FORK** 를 클릭
  - 여기까지 준비가 안된 경우라면 fork 웹사이트를 열고 가운데 있는 개별 저장소 URL을 복사
  - 저장소를 하드 드라이브로 clone한다. `git clone https://github.com/<youraccountname>/Firmware.git` 명령을 사용. Windows 사용자는 [Github 도움말](https://help.github.com/desktop/guides/getting-started-with-github-desktop/installing-github-desktop/)을 참고하세요.
  - git submodules 업데이트 : 쉘에서 실행(윈도우에서는 PX4 콘솔에서 실행)

<div class="host-code"></div>

```sh
cd Firmware
git submodule init
git submodule update --recursive
```

하드디스크의 `Firmware/src/examples/` 디렉토리로 진입해서 디렉토리 내부에 잇는 파일들을 살펴봅시다.

## Step 2: 최소 단위 Application

`px4_simple_app` 폴더에 `px4_simple_app.c`라는 파일을 생성합니다. (이미 존재하는 경우 교육효과를 극대화하기 위해서 기존 파일을 지우고 따라 합니다)

이 파일을 편집하고 기본 헤더와 main 함수로 시작합니다.

> **Tip** 이 파일에서 사용하는 코드 스타일은 PX4를 개발할 때 동일하게 적용됩니다.

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

## Step 3: 이 Application을 NuttSheel에 등록하고 빌드하기

이제 이 application은 완성되어 실행할 수 있습니다. 하지만 NuttShell command로 등록하지는 않았습니다. firmware 내부로 컴파일 가능하도록 하기 위해, build 대상 module 목록에 추가합니다. 위치는 아래와 같습니다. :

* jMAVSim Simulator: `make px4_sitl_default jmavsim`
* Pixhawk v1/2: `make px4_fmu-v2_default` (or just `make px4_fmu-v2`)
* Pixhawk v3: `make px4_fmu-v4_default`
* Other boards: [Building the Code](../setup/building_px4.md#building_nuttx)

파일내에서 특정 위치에 여러분이 작성한 application을 위해 한 줄 추가합니다.

  `examples/px4_simple_app`

빌드하기:

* Pixhawk v1/2: `make px4_fmu-v2_default upload`
* Pixhawk v3: `make px4_fmu-v4_default upload`

## Step 4: app을 업로드하고 테스트하기

uploader를 활성화시키고 board를 리셋 :

  * Pixhawk v1/2: `make px4_fmu-v2_default upload`
  * Pixhawk v3: `make px4_fmu-v4_default upload`

보드를 리셋하기 전에 많은 컴파일 메시지를 프린트해야 하며 마지막에는 :

  `Loaded firmware for X,X, waiting for the bootloader...`

일단 board가 리셋되고 upload되고 이를 프린트한다. :

<div class="host-code"></div>

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

### 콘솔 연결하기

이제 시리얼이나 USB로 [시스템 콘솔](../debug/system_console.md)에 연결합니다. ENTER를 눌르면 쉘 프롬프트가 나타납니다:

```sh
  nsh>
```


''help''를 입력하고 엔터를 누르면

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

이제 `px4_simple_app`는 사용할 수 있는 명령이 되었습니다. `px4_simple_app`을 입력하고 ENTER를 치면 :

```sh
  nsh> px4_simple_app
  Hello Sky!
```

어플리케이션은 이제 시스템에 올바로 등록되었고 실제 task를 수행할 수 있게 확장할 수 있습니다.

## Step 5: 센서 데이터 subscribing

유용한 작업을 하기 위해서, 어플리케이션은 입력을 subscribe하고 출력(모터나 서보 명령)을 publish해야 합니다. 실제 PX4 플랫폼의 하드웨어 추상화는 여기서 중요한 역할을 하게 됩니다. -- 즉 board나 센서가 업데이트되더라도 센서 driver와 상호동작 그리고 여러분의 app을 업데이트하지 않아도 됩니다.

어플리케이션 사이에 개별 메시지 채널을 PX4에서 *topics* 이라고 부릅니다. 여기서는 [sensor_combined](https://github.com/PX4/Firmware/blob/master/msg/sensor_combined.msg) [topic](../middleware/uorb.md) 에 초점을 둡니다. 이 topic은 시스템에서 동기화된 센서 데이터를 가집니다.

topic을 subscribe하는 것은 빠르고 간편합니다 :

```C++
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
```

`sensor_sub_fd`는 topic handle로서 새 데이터를 기다리는 작업을 매우 효과적으로 처리할 수 있습니다. 현재 thread가 sleep으로 빠지고 새 데이터가 들어오면 스케쥴러가 자동으로 깨워줍니다. 기다리는 동안 CPU cycle을 사용하지 않습니다. 이렇게 하기 위해서,  [poll()](http://pubs.opengroup.org/onlinepubs/007908799/xsh/poll.html) POSIX system call을 사용합니다.

`poll()`을 subscription에 추가하는 방법은 다음과 같습니다. (*pseudocode로 전체 구현은 아래에서 다룹니다.*)

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

이제 app을 컴파일합니다.

<div class="host-code"></div>

```
  make
```

### uORB Subscription 테스팅

마지막 단계는 여러분의 application을 백그라운드로 구동합니다.

```
  px4_simple_app &
```

여러분의 app은 현재 센서 값을 콘솔에서 계속 출력합니다.:

```
  [px4_simple_app] Accelerometer:   0.0483          0.0821          0.0332
  [px4_simple_app] Accelerometer:   0.0486          0.0820          0.0336
  [px4_simple_app] Accelerometer:   0.0487          0.0819          0.0327
  [px4_simple_app] Accelerometer:   0.0482          0.0818          0.0323
  [px4_simple_app] Accelerometer:   0.0482          0.0827          0.0331
  [px4_simple_app] Accelerometer:   0.0489          0.0804          0.0328
```

5개 값을 프린팅한 후에도 계속 살아 있습니다. 다음 튜터리얼 페이지에는 commandline으로 제어할 수 있는 deamon 작성 방법을 설명하겠습니다.

## Step 7: 데이터 Publishing

계산한 output을 사용하기 위해, 다음 단계로 결과를 *publish* 합니다. topic을 사용해서 ''mavlink'' app이 ground control station으로 전달한다면, 결과를 살펴볼 수 있습니다. 이런 목적으로 attitude topic을 가로채기합니다.

인터페이스는 매우 단순합니다. : publish할 topic의 구조를 초기화하고 topic을 advertise합니다. :

```C
#include <uORB/topics/vehicle_attitude.h>
..
/* advertise attitude topic */
struct vehicle_attitude_s att;
memset(&att, 0, sizeof(att));
orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
```

main loop에서 준비가 되면, 정보를 publish합니다. :

```C
orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);
```

수정한 전체 예제 코드는 아래와 같습니다. :

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

## 최종 예제 실행하기

이제 마지막으로 여러분의 app을 실행 :

```sh
  px4_simple_app
```

QGroundControl을 시작하면, 실시간 plot으로 센서 값을 검사할 수 있습니다. (Tools -> Analyze)

## 정리

이 튜터리얼에서 PX4 autopilot application을 개발하는데 필요한 모든 것을 다뤘습니다. uORB 메시지의 전체 목록 / topic은 [여기에서](https://github.com/PX4/Firmware/tree/master/msg/) 확인할 수 있으며 헤더는 문서화가 잘 되어있고 레퍼런스 역할을 할 수 있다는 사실을 명심하세요.
