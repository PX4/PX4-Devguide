# 첫번째 어플리케이션 튜토리얼 (Hello Sky)

이번 주제에서는 여러분의 첫번째 온보드 어플리케이션의 생성 및 실행 방법에 대해 설명합니다. PX4 기반의 어플리케이션 개발에 필요한 API와 기본 컨셉을 다룹니다.

> **Note** 기능의 시작/중단, 명령행 인자와 같은 고급 기능은 간결성을 위하여 제외하였습니다. [Application/Module Template](../apps/module_template.md)에서 이러한 내용을 다룹니다..

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

1. **CMakeLists.txt**이름의 *cmake* 정의 파일을 만들고 여십시오. 아래 텍스트를 파일에 복사하십시오.
    
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

이제 어플리케이션은 완결되었습니다. 이를 실행하려면 우선 이 어플리케이션을 PX4와 함께 빌드해야합니다. 대상 보드에 넣을 적당한 보드레벨 *cmake* 파일에 빌드와 펌웨어로 들어갈 프로그램을 추가했습니다.

* PX4 SITL (시뮬레이터): [Firmware/boards/px4/sitl/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/sitl/default.cmake)
* Pixhawk v1/2: [Firmware/boards/px4/fmu-v2/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v2/default.cmake)
* Pixracer (px4/fmu-v4): [Firmware/boards/px4/fmu-v4/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v4/default.cmake)
* 다른 보드용 *cmake* 파일은 [Firmware/boards/](https://github.com/PX4/Firmware/tree/master/boards)에서 확인 가능

어플리케이션을 펌웨어에 넣어 컴파일하려면 *cmake* 파일내에 어플리케이션과 관련된 새 라인을 작성하십시오:

    examples/px4_simple_app
    

> **Note** 예제에는 기본적으로 펌웨어로 넣었기 때문에, 대부분의 파일에 이 라인이 있습니다.

보드별 지정 명령으로 예제를 빌드하십시오:

* jMAVSim 시뮬레이터: `make px4_sitl_default jmavsim`
* Pixhawk v1/2: `make px4_fmu-v2_default` (혹은 단지 `make px4_fmu-v2`)
* Pixhawk v3: `make px4_fmu-v4_default`
* 다른 보드: [Building the Code](../setup/building_px4.md#building_nuttx)

## 어플리케이션 테스트 (하드웨어)

### 보드에 펌웨어 업로드

업로더를 활성화하고 보드를 리셋하십시오:

* Pixhawk v1/2: `make px4_fmu-v2_default upload`
* Pixhawk v3: `make px4_fmu-v4_default upload`

보드를 리셋하기 전 여러줄의 컴파일 메시지가 나타나고 마지막에 다음과 같은 줄이 떠야 합니다:

```sh
Loaded firmware for X,X, waiting for the bootloader...
```

보드가 리셋되면 업로드를 진행하면서 다음 내용이 나타납니다:

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

### 콘솔 연결

이제 시리얼이나 USB를 통해 [system console](../debug/system_console.md)에 연결합니다. **ENTER** 를 누르면 쉘 프롬프트가 나타납니다:

```sh
nsh>
```

''help''를 입력하고 ENTER를 누르십시오.

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

> **Tip** 이 지점에서 PX4 하드웨어 추상화의 이점이 나타납니다! 보드 혹은 센서가 변경되는 경우에도 센서 드라이버와 어떤 방식의 상호작용을 하거나 어플리케이션을 업데이트할 필요는 없습니다.

어플리케이션 간의 개별 메시지 체널을 [topic](../middleware/uorb.md)이라고 합니다. 본 튜토리얼에서는 전체 시스템의 동기화된 센서 데이터를 가지고 있는 [sensor_combined](https://github.com/PX4/Firmware/blob/master/msg/sensor_combined.msg) topic에 살펴보겠습니다.

Topic 구독은 간단합니다:

```cpp
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
```

`sensor_sub_fd` 은 topic handle이며, 새로운 데이터를 블로킹 방식으로 대기하는데 효율적으로 사용될 수 있습니다. 현 쓰레드는 슬립상태로 들어가고 새로운 데이터가 있을때 스케쥴러에의해 자동적으로 깨어나며, 대기시 CPU 사이클을 소비하지 않습니다. 이러한 용도로 [poll()](http://pubs.opengroup.org/onlinepubs/007908799/xsh/poll.html) POSIX 시스템 콜을 사용합니다.

구독하는 쪽에 `poll()`을 추가한 경우 (*의사코드임, 전체 구현은 아래를 볼 것*):

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

다음을 입력하여 어플리케이션 컴파일:

```sh
make
```

### uORB 구독 테스트

마지막 단계로 nsh 쉘에 다음을 입력하여 어플리케이션을 백그라운드 프로스세/태스크로 시작:

```sh
px4_simple_app &
```

어플리케이션은 콘솔에 5개의 센서 값을 표시하고 종료:

```sh
[px4_simple_app] Accelerometer:   0.0483          0.0821          0.0332
[px4_simple_app] Accelerometer:   0.0486          0.0820          0.0336
[px4_simple_app] Accelerometer:   0.0487          0.0819          0.0327
[px4_simple_app] Accelerometer:   0.0482          0.0818          0.0323
[px4_simple_app] Accelerometer:   0.0482          0.0827          0.0331
[px4_simple_app] Accelerometer:   0.0489          0.0804          0.0328
```

> **Tip** [Module Template for Full Applications](../apps/module_template.md)을 사용하여, 명령행에서 제어할 수 있는 백그라운드 프로세스 작성 가능.

## 데이터 발행 (Publishing Data)

계산이 완료된 출력값을 사용하기 위한 다음 단계로 결과값을 *발행(publish)*합니다. 다음에서 고도 topic을 발행하는 방법을 보여줍니다.

> **Note** *mavlink* 앱이 지상 관제소에 `attitude`를 전달하며 이 결과를 쉽게 볼수 있어 이를 선정함.

인터페이스는 매우 간단함: 발행될 topic의 `구조체(struct)`를 초기화하고 topic을 알림(advertise):

```c
#include <uORB/topics/vehicle_attitude.h>
..
/* advertise attitude topic */
struct vehicle_attitude_s att;
memset(&att, 0, sizeof(att));
orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
```

main 루프에서 정보가 준비될 때마다 이를 발행함:

```c
orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);
```

## 전체 예제 코드

[전체 예제 코드](https://github.com/PX4/Firmware/blob/master/src/examples/px4_simple_app/px4_simple_app.c)는 다음과 같다:

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

## 전체 예제 실행

마지막으로 어플리케이션을 실행:

```sh
px4_simple_app
```

*QGroundControl*을 실행하면, 센서 값을 실시간 플롯으로 확인 가능합니다 ([Analyze > MAVLink Inspector](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_inspector.html)).

## 마무리

본 튜토리얼에서 PX4 오토파일럿 어플리케이션 개발에 필요한 모든 것을 다루었습니다. 전체 uORB 메시지/토픽의 리스트는 [여기](https://github.com/PX4/Firmware/tree/master/msg/)에 있으며 헤더에 문서화가 잘되어 있으며 참고로 삼을 수 있음을 잊지 마시기 바랍니다.

보다 상세한 정보와 트러블슈팅/흔한 함정에 대한 내용은 여기에서 찾을 수 있습니다: [uORB](../middleware/uorb.md).

다음 페이지에서는 시작/종료 기능을 가지는 완전한 어플리케이션(full application)을 작성하기위한 템플릿을 제공합니다.