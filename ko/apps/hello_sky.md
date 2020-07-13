# 첫번째 어플리케이션 튜토리얼 (Hello Sky)

이번 주제에서는 여러분의 첫번째 온보드 어플리케이션의 생성 및 실행 방법에 대해 설명합니다. PX4 기반의 어플리케이션 개발에 필요한 API와 기본 컨셉을 다룹니다.

> **주의** 기능의 시작/중단, 명령행 인자와 같은 고급 기능은 간결성을 위하여 제외하였습니다. [Application/Module Template](../apps/module_template.md)에서 이러한 내용을 다룹니다..

## 사전 준비 사항

다음의 항목이 필요합니다.

* [PX4 SITL Simulator](../simulation/README.md) * 혹은 * [PX4-compatible flight controller](https://docs.px4.io/master/en/flight_controller/#documented-boards).
* 사용할 타겟용 [PX4 개발 툴체인](../setup/dev_env.md).
* Github에서 [PX4 소스 코드 다운로드](../setup/building_px4.md#get_px4_code)

본 튜토리얼 진행 중 어려움에 막혔을 때 참고할 수 있는 완성된 버전은 [Firmware/src/examples/px4_simple_app](https://github.com/PX4/Firmware/tree/master/src/examples/px4_simple_app) 소스 코드 디렉터리에 있습니다.

* **px4_simple_app** 디렉터리의 이름을 변경하시오 (혹은 삭제). 

## 미니멀 어플리케이션

여기에서는 단지 `Hello Sky!`를 출력하는 *미니멀 어플리케이션*을 만듭니다. 하나의 *C* 파일과 하나의 *cmake* 정의(어플리케이션을 빌드하는 방법을 툴체인에게 지시)로 구성됩니다.

1. **Firmware/src/examples/px4_simple_app** 디렉터리를 신규 생성.
2. **px4_simple_app.c** 이름의 C 파일을 신규 생성:

* 기본 헤더를 페이지 최상단에 복사. 이 내용은 모든 기여 파일(contributed files)에 있어야 함. 
    
    ```c /**************************************************************************** *
    
    * Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
    * 
    * Redistribution and use in source and binary forms, with or without
    * modification, are permitted provided that the following conditions
    * are met:
    * 
    * 1. Redistributions of source code must retain the above copyright
    * notice, this list of conditions and the following disclaimer.
    * 2. Redistributions in binary form must reproduce the above copyright
    * notice, this list of conditions and the following disclaimer in
    * the documentation and/or other materials provided with the
    * distribution.
    * 3. Neither the name PX4 nor the names of its contributors may be
    * used to endorse or promote products derived from this software
    * without specific prior written permission.
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
    * ****************************************************************************/ ```

* 기본 헤더 아래에 다음 코드를 복사. 이 내용은 모든 기여 파일에 있어야 함.
    
    ```c /**
    
    * @file px4_simple_app.c
    * Minimal application example for PX4 autopilot
    * 
    * @author Example User [&#x6d;&#97;&#105;l&#x40;&#x65;&#120;&#97;&#109;&#x70;&#x6c;&#x65;&#46;&#99;o&#x6d;](&#x6d;&#x61;&#105;&#108;&#116;&#x6f;&#x3a;&#x6d;&#97;&#105;l&#x40;&#x65;&#120;&#97;&#109;&#x70;&#x6c;&#x65;&#46;&#99;o&#x6d;) */
        
        # include <px4_platform_common>
        
        __EXPORT int px4_simple_app_main(int argc, char *argv[]);
        
        int px4_simple_app_main(int argc, char *argv[]) { PX4_INFO("Hello Sky!"); return OK; } ```
        
        > **Tip** 메인 함수의 이름은 반드시 `<module_name>_main` 로 작성하며 위와 같이 모듈에서 추출(export)되어야 함.
        
        <span></span>
        
        > **Tip** PX4 쉘에서 `PX4_INFO`는 `printf`에 해당한다. (**px4_platform_common/log.h**에서 include됨) 4 단계의 로글 레벨이 존재: `PX4_INFO`, `PX4_WARN`, `PX4_ERR`, `PX4_DEBUG`. 경고와 에러는 [ULog](../log/ulog_file_format.md)에 추가적으로 포함되고 [Flight Review](https://logs.px4.io/)에서 나타난다.

1. **CMakeLists.txt**이름의 *cmake* 정의 파일을 만들고 열기. 아래 텍스트를 파일에 복사.
    
    ```cmake
    ############################################################################
    #
    #   Copyright (c) 2015 PX4 Development Team. All rights reserved.
    #
    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions
    # are met:
    #
    # 1. Redistributions of source code must retain the above copyright
    #    notice, this list of conditions and the following disclaimer.
    # 2. Redistributions in binary form must reproduce the above copyright
    #    notice, this list of conditions and the following disclaimer in
    #    the documentation and/or other materials provided with the
    #    distribution.
    # 3. Neither the name PX4 nor the names of its contributors may be
    #    used to endorse or promote products derived from this software
    #    without specific prior written permission.
    #
    # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    # OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    # POSSIBILITY OF SUCH DAMAGE.
    #
    ############################################################################
    px4_add_module(
    MODULE examples__px4_simple_app
    MAIN px4_simple_app
    STACK_MAIN 2000
    SRCS
        px4_simple_app.c
    DEPENDS
    )
    ```
    
    `px4_add_module()` 메써드는 모듈 디스크립션에 있는 내용에서 정적 라이브러리를 빌드합니다. `MAIN` 블럭에 모듈의 이름을 기재하며 이는 NuttX에 명령어로 등록되어 PX4 쉘이나 SITL 콘솔에서 호출할 수 있습니다.
    
    > **Tip** `px4_add_module()` 의 포맷은 [Firmware/cmake/px4_add_module.cmake](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/cmake/px4_add_module.cmake)에 기술되어 있음.
    
    <span></span>
    
    > **Note** `px4_add_module`의 옵션을 `DYNAMIC`로 지정한 경우에는, 정적 라이브러리 대신 POSIX 플랫폼상의 *공유 라이브러리*가 생성됨. (PX4를 재컴파일하지 않고서도 이를 로딩할 수 있으며, 다른 사람들에게 소스 코드 대신 바이너리로 공유가 가능함). 이 경우 앱은 내장 명령어가 되지 않고, `examples__px4_simple_app.px4mod`와 같은 별도의 파일이 됨. 런타임에서 `dyn` 명령어를 이용하여 이 파일을 로드한 후 실행할 수 있음: `dyn ./examples__px4_simple_app.px4mod`

## 어플리케이션/펌웨어 빌드

이제 어플리케이션은 완결되었습니다. 이를 실행하기 위해서는 우선 이 어플리케이션을 PX4와 함께 빌드해야할 필요가 있습니다. 여러분의 타겟용 보드 레벨 *cmake* 파일의 build/firmware에 어플리케이션을 추가합니다. 

* PX4 SITL (시뮬레이터): [Firmware/boards/px4/sitl/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/sitl/default.cmake)
* Pixhawk v1/2: [Firmware/boards/px4/fmu-v2/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v2/default.cmake)
* Pixracer (px4/fmu-v4): [Firmware/boards/px4/fmu-v4/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v4/default.cmake)
* 다른 보드용 *cmake* 파일은 [Firmware/boards/](https://github.com/PX4/Firmware/tree/master/boards)에서 확인 가능

어플리케이션을 펌웨어로 포함해서 컴파일하기 위해서는 *cmake* 파일내에 어플리케이션과 관련된 새로운 라인을 작성합니다.

    examples/px4_simple_app
    

> **Note** 예제는 기본적으로 펌웨어로 포함되어 있기 때문에, 대부분의 파일에 이 라인은 이미 존재함.

보드에 특정된 명령을 이용하여 예제를 빌드:

* jMAVSim 시뮬레이터: `make px4_sitl_default jmavsim`
* Pixhawk v1/2: `make px4_fmu-v2_default` (혹은 단지 `make px4_fmu-v2`)
* Pixhawk v3: `make px4_fmu-v4_default`
* 다른 보드: [Building the Code](../setup/building_px4.md#building_nuttx)

## 어플리케이션 테스트 (하드웨어)

### 보드에 펌웨어 업로드

업로더를 활성화하고 보드를 리셋:

* Pixhawk v1/2: `make px4_fmu-v2_default upload`
* Pixhawk v3: `make px4_fmu-v4_default upload`

여러분이 보드를 리셋하기 전에 컴파일 메시지가 출력되고 마지막에는 :

```sh
Loaded firmware for X,X, waiting for the bootloader...
```

보드가 리셋되면 업로드가 진행되고 다음을 출력:

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

### 콘솔 연결

이제 시리얼이나 USB를 통해 [system console](../debug/system_console.md)에 연결합니다. **ENTER** 를 누르면 쉘 프롬프트가 나타납니다.:

```sh
nsh>
```

''help''를 입력하고 ENTER를 누릅니다.

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

이제 `px4_simple_app`가 사용가능한 명령어 중 일부임을 주목합니다. `px4_simple_app`와 ENTER를 입력하여 어플리케이션을 시작:

```sh
nsh> px4_simple_app
Hello Sky!
```

이제 어플리케이션은 시스템에 올바르게 등록되었고 실제로 유용한 일을 수행하도록 이를 확장할 수 있습니다.

## 어플리케이션 테스트 (SITL)

SITL을 사용하는 경우, *PX4 콘솔*은 자동적으로 실행됩니다 (참고 [Building the Code > First Build (Using the jMAVSim Simulator)](../setup/building_px4.md#jmavsim_build)). *nsh 콘솔*처럼(이전 섹션 참고) `help`를 입력하여 내장된 어플리케이션의 목록을 볼 수 있습니다.

`px4_simple_app` 입력하여 미니멀 어플리케이션을 실행.

```sh
pxh> px4_simple_app
INFO  [px4_simple_app] Hello Sky!
```

이제 실제로 유용할 일을 수행하기 위해 이 어플리케이션을 확장할 수 있습니다.

## 센서 데이터 구독 (Subscribing to Sensor Data)

유용한 일을 수행하기 위해, 어플리케이션은 입력값을 구독(subscribe)하고 출력값을 발행(publish)할 필요가 있습니다(예, 모터 혹은 서보 명령).

> **Tip** 이지점에서 PX4 하드웨어 추상화의 이점이 나타납니다! There is no need to interact in any way with sensor drivers and no need to update your app if the board or sensors are updated.

Individual message channels between applications are called [topics](../middleware/uorb.md). For this tutorial, we are interested in the [sensor_combined](https://github.com/PX4/Firmware/blob/master/msg/sensor_combined.msg) topic, which holds the synchronized sensor data of the complete system.

Subscribing to a topic is straightforward:

```cpp
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
```

The `sensor_sub_fd` is a topic handle and can be used to very efficiently perform a blocking wait for new data. The current thread goes to sleep and is woken up automatically by the scheduler once new data is available, not consuming any CPU cycles while waiting. To do this, we use the [poll()](http://pubs.opengroup.org/onlinepubs/007908799/xsh/poll.html) POSIX system call.

Adding `poll()` to the subscription looks like (*pseudocode, look for the full implementation below*):

```cpp
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

Compile the app again by entering:

```sh
make
```

### Testing the uORB Subscription

The final step is to start your application as a background process/task by typing the following in the nsh shell:

```sh
px4_simple_app &
```

Your app will display 5 sensor values in the console and then exit:

```sh
[px4_simple_app] Accelerometer:   0.0483          0.0821          0.0332
[px4_simple_app] Accelerometer:   0.0486          0.0820          0.0336
[px4_simple_app] Accelerometer:   0.0487          0.0819          0.0327
[px4_simple_app] Accelerometer:   0.0482          0.0818          0.0323
[px4_simple_app] Accelerometer:   0.0482          0.0827          0.0331
[px4_simple_app] Accelerometer:   0.0489          0.0804          0.0328
```

> **Tip** The [Module Template for Full Applications](../apps/module_template.md) can be used to write background process that can be controlled from the command line.

## Publishing Data

To use the calculated outputs, the next step is to *publish* the results. Below we show how to publish the attitude topic.

> **Note** We've chosen `attitude` because we know that the *mavlink* app forwards it to the ground control station - providing an easy way to look at the results.

The interface is pretty simple: initialize the `struct` of the topic to be published and advertise the topic:

```c
#include <uORB/topics/vehicle_attitude.h>
..
/* advertise attitude topic */
struct vehicle_attitude_s att;
memset(&att, 0, sizeof(att));
orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
```

In the main loop, publish the information whenever its ready:

```c
orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);
```

## Full Example Code

The [complete example code](https://github.com/PX4/Firmware/blob/master/src/examples/px4_simple_app/px4_simple_app.c) is now:

```c
/****************************************************************************
 *

 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

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

                /* set att and publish this information for other apps
                 the following does not have any meaning, it's just an example
                */
                att.q[0] = raw.accelerometer_m_s2[0];
                att.q[1] = raw.accelerometer_m_s2[1];
                att.q[2] = raw.accelerometer_m_s2[2];

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

## Running the Complete Example

And finally run your app:

```sh
px4_simple_app
```

If you start *QGroundControl*, you can check the sensor values in the real time plot ([Analyze > MAVLink Inspector](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_inspector.html)).

## Wrap-Up

This tutorial covered everything needed to develop a basic PX4 autopilot application. Keep in mind that the full list of uORB messages/topics is [available here](https://github.com/PX4/Firmware/tree/master/msg/) and that the headers are well documented and serve as reference.

Further information and troubleshooting/common pitfalls can be found here: [uORB](../middleware/uorb.md).

The next page presents a template for writing a full application with start and stop functionality.