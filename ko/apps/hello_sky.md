# 첫 프로그램 자습서(Hello Sky)

이 주제에서는 보드상에서 처음 실행하는 프로그램을 작성하고 실행하는 방법을 설명합니다. PX4 기반 프로그램을 개발할 때 필요한 기본 개념과 API를 다룹니다.

> **Note** 기능의 시작/중단, 명령행 인자와 같은 고급 기능은 간결성을 위하여 제외하였습니다. [프로그램/모듈 서식](../apps/module_template.md)에서 이러한 내용을 다룹니다.

## 사전 준비 사항

다음의 항목이 필요합니다.

* [PX4 SITL 모의 시험환경](../simulation/README.md) * 혹은 * [PX4-호환 비행체 제어 장치](https://docs.px4.io/master/en/flight_controller/#documented-boards).
* 사용할 타겟용 [PX4 개발 툴체인](../setup/dev_env.md).
* Github에서 [PX4 소스 코드 다운로드](../setup/building_px4.md#get_px4_code)

본 튜토리얼 진행 중 어려움에 막혔을 때 참고할 수 있는 완성된 버전은 [Firmware/src/examples/px4_simple_app](https://github.com/PX4/Firmware/tree/master/src/examples/px4_simple_app) 소스 코드 디렉터리에 있습니다.

* **px4_simple_app** 디렉터리의 이름을 변경(또는 삭제)하십시오. 

## 간단한 프로그램

여기에서는 `Hello Sky!` 만을 출력하는 *간단한 프로그램*을 만들어보겠습니다. 하나의 *C* 파일과 하나의 *cmake* 정의(프로그램 빌드 방법을 툴체인에게 지시)로 구성됩니다.

1. **Firmware/src/examples/px4_simple_app** 디렉터리를 새로 만드십시오.
2. **px4_simple_app.c** 이름의 C 파일을 새로 만드십시오:

* 기본 헤더를 페이지 최상단에 복사하십시오. 이 BSD 3-Clauses 라이선스 주석은 모든 기여 파일에 있어야 합니다.
    
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

* 기본 헤더 아래에 다음 코드를 복사하십시오. 소스 코드 설명과 작성자 시그니쳐는 모든 파일에 있어야합니다!
    
    ```c /**
    
    * @file px4_simple_app.c
    * Minimal application example for PX4 autopilot
    * 
    * @author Example User [&#x6d;&#97;&#105;l&#x40;&#x65;&#120;&#97;&#109;&#x70;&#x6c;&#x65;&#46;&#99;o&#x6d;](&#x6d;&#x61;&#105;&#108;&#116;&#x6f;&#x3a;&#x6d;&#97;&#105;l&#x40;&#x65;&#120;&#97;&#109;&#x70;&#x6c;&#x65;&#46;&#99;o&#x6d;) */
        
        # include <px4_platform_common>
        
        __EXPORT int px4_simple_app_main(int argc, char *argv[]);
        
        int px4_simple_app_main(int argc, char *argv[]) { PX4_INFO("Hello Sky!"); return OK; } ```
        
        > **Tip** 메인 함수의 이름은 `<module_name>_main`으로 붙어야 하며, 나타난 바와 같이 모듈로 뺄 수 있어야합니다.
        
        <span></span>
        
        > **Tip** PX4 쉘에서 `PX4_INFO`는 `printf`에 해당합니다(**px4_platform_common/log.h**에서 끌어옴) 4단계의 로그 레벨 `PX4_INFO`, `PX4_WARN`, `PX4_ERR`, `PX4_DEBUG`이 있습니다. 경고와 오류는 [ULog](../log/ulog_file_format.md)에 추가로 들어가고 [Flight Review](https://logs.px4.io/)에서 나타납니다.

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
    
    `px4_add_module()` 메서드는 모듈 설명에 따라 정적 라이브러리를 빌드합니다. `MAIN` 블럭에는 모듈의 이름이 나타납니다 - NuttX에서 명령을 등록하여 PX4 셸 또는 SITL 콘솔에서 호출할 수 있습니다.
    
    > **Tip** `px4_add_module()` 의 형식은 [Firmware/cmake/px4_add_module.cmake](https://github.com/PX4/Firmware/blob/{{ book.px4_version }}/cmake/px4_add_module.cmake)에 들어 있습니다.
    
    <span></span>
    
    > **Note** `px4_add_module`의 옵션을 `DYNAMIC`로 지정한 경우, 정적 라이브러리 대신 POSIX 플랫폼상의 *공유 라이브러리*를 만듭니다. (PX4를 다시 컴파일하지 않아도 불러올 수 있으며, 다른 사람에게 소스 코드 대신 바이너리로 공유할 수 있습니다). 이 경우 앱은 내장 명령어로 나오지 않지만, `examples__px4_simple_app.px4mod`와 같은 별도의 파일이 나옵니다. 런타임에서 `dyn` 명령어를 이용하여 이 파일을 로드한 후 실행할 수 있음: `dyn ./examples__px4_simple_app.px4mod`

## 프로그램/펌웨어 빌드

이제 프로그램 작성이 끝났습니다. 이를 실행하려면 우선 이 프로그램을 PX4의 일부로 빌드해야합니다. 대상 보드에 넣을 적당한 보드레벨 *cmake* 파일에 빌드와 펌웨어로 들어갈 프로그램을 추가했습니다:

* PX4 SITL (시뮬레이터): [Firmware/boards/px4/sitl/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/sitl/default.cmake)
* Pixhawk v1/2: [Firmware/boards/px4/fmu-v2/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v2/default.cmake)
* Pixracer (px4/fmu-v4): [Firmware/boards/px4/fmu-v4/default.cmake](https://github.com/PX4/Firmware/blob/master/boards/px4/fmu-v4/default.cmake)
* 다른 보드용 *cmake* 파일은 [Firmware/boards/](https://github.com/PX4/Firmware/tree/master/boards)에서 확인 가능

프로그램을 펌웨어에 넣어 컴파일하려면 *cmake* 파일내에 프로그램 포함 내용 한 줄을 새로 작성하십시오:

    examples/px4_simple_app
    

> **Note** 예제에는 기본적으로 펌웨어로 넣었기 때문에, 대부분의 파일에 이 라인이 있습니다.

보드별 지정 명령으로 예제를 빌드하십시오:

* jMAVSim 시뮬레이터: `make px4_sitl_default jmavsim`
* Pixhawk v1/2: `make px4_fmu-v2_default` (혹은 단지 `make px4_fmu-v2`)
* Pixhawk v3: `make px4_fmu-v4_default`
* 다른 보드: [Building the Code](../setup/building_px4.md#building_nuttx)

## 프로그램 테스트 (하드웨어)

### 보드에 펌웨어 업로드

업로더를 활성화하고 보드를 리셋하십시오:

* Pixhawk v1/2: `make px4_fmu-v2_default upload`
* Pixhawk v3: `make px4_fmu-v4_default upload`

보드를 리셋하기 전 여러줄의 컴파일 메시지가 나타나고 마지막에 다음과 같은 줄이 떠야 합니다:

```sh
Loaded firmware for X,X, waiting for the bootloader...
```

보드를 재부팅하면 업로드를 진행하면서 다음 내용이 나타납니다:

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

### 콘솔 연결

이제 시리얼이나 USB를 통해 [시스템 콘솔](../debug/system_console.md)에 연결하십시오. **ENTER** 를 누르면 셸 프롬프트가 나타납니다:

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

이제 `px4_simple_app`가 사용가능한 명령어 중 일부임을 주목합니다. `px4_simple_app`와 ENTER를 입력하여 프로그램을 시작하십시오:

```sh
nsh> px4_simple_app
Hello Sky!
```

이제 프로그램을 시스템에 올바르게 등록했고 실제로 쓸만한 동작을 하도록 프로그램을 확장할 수 있습니다.

## 프로그램 테스트 (SITL)

SITL을 사용하는 경우, *PX4 콘솔*을 자동으로 실행합니다 ([Building the Code > First Build (Using the jMAVSim Simulator)](../setup/building_px4.md#jmavsim_build) 참고). *nsh 콘솔*처럼(이전 섹션 참고) `help`를 입력하여 내장 프로그램 목록을 볼 수 있습니다.

`px4_simple_app` 입력하여 간단한 프로그램을 실행하십시오.

```sh
pxh> px4_simple_app
INFO  [px4_simple_app] Hello Sky!
```

이제 진짜로 쓸만한 동작으로 프로그램을 확장할 수 있습니다.

## 센서 데이터 정기 수신

뭔가 쓸만한 동작을 하려면, 프로그램에서는 입력 값을 정기적으로 수신하고 출력 값(예: 모터 혹은 서보 명령)을 내보낼 필요가 있습니다.

> **Tip** 이 시점에서 PX4 하드웨어 추상화의 이점이 나타납니다! 보드 또는 센서를 업데이트했을 때 센서 드라이버와 직접 통신하거나 프로그램을 업데이트할 필요가 없습니다.

프로그램간 주고 받는 개별 메세지 채널을 [토픽](../middleware/uorb.md)이라고 합니다. 이 자습서에서는 온전한 시스템에서 센서 데이터를 동기화 유지하는 [sensor_combined](https://github.com/PX4/Firmware/blob/master/msg/sensor_combined.msg) 토픽을 살펴보겠습니다.

토픽을 정기적으로 수신하는 방법은 간단합니다:

```cpp
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
```

`sensor_sub_fd` 은 토픽 핸들이며, 새 데이터를 수신할 때 블로킹 대기를 매우 효율적으로 수행하는 목적으로 활용할 수 있습니다. 현재 스레드는 대기 상태로 진입하며, 기다리는 동안 CPU 사이클을 소모하지 않고, 새 데이터가 나타나면 스케쥴러에서 자동으로 깨웁니다. 이러한 용도로 [poll()](http://pubs.opengroup.org/onlinepubs/007908799/xsh/poll.html) POSIX 시스템 콜을 사용합니다.

정기 수신 노드에 `poll()`을 추가한 경우 (*의사코드임, 전체 구현은 아래를 볼 것*):

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

다음 명령을 입력하여 프로그램을 컴파일하십시오:

```sh
make
```

### uORB 정기 수신 테스트

마지막 단계로, 다음 명령을 nsh 셸에서 입력하여 프로그램을 백그라운드 프로세스 작업으로 시작하십시오:

```sh
px4_simple_app &
```

프로그램은 센서 값 5개를 콘솔에 나타내고 끝냅니다:

```sh
[px4_simple_app] Accelerometer:   0.0483          0.0821          0.0332
[px4_simple_app] Accelerometer:   0.0486          0.0820          0.0336
[px4_simple_app] Accelerometer:   0.0487          0.0819          0.0327
[px4_simple_app] Accelerometer:   0.0482          0.0818          0.0323
[px4_simple_app] Accelerometer:   0.0482          0.0827          0.0331
[px4_simple_app] Accelerometer:   0.0489          0.0804          0.0328
```

> **Tip** [전체 프로그램용 모듈 서식](../apps/module_template.md)으로 명령행에서 제어할 수 있는 백그라운드 프로세스를 작성할 수 있습니다.

## 데이터 내보내기

계산을 끝낸 출력 값을 사용할 다음 단계는 결과 값을 *내보내*는 동작입니다. 다음을 통해 고도 데이터만 따로 내보내는 방법을 보여드리겠습니다.

> **Note** *mavlink*앱이 지상 통제 장치에 `attitude`를 전달하는 결과를 쉽게 볼 수 있어 이를 선정했습니다.

인터페이스는 매우 간단합니다. 내보낼 토픽의 `구조체(struct)`를 초기화하고 토픽을 내보냅니다:

```c
#include <uORB/topics/vehicle_attitude.h>
..
/* advertise attitude topic */
struct vehicle_attitude_s att;
memset(&att, 0, sizeof(att));
orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
```

main 루프에서 정보를 준비하는 대로 바로 내보냅니다:

```c
orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);
```

## 전체 예제 코드

[전체 예제 코드](https://github.com/PX4/Firmware/blob/master/src/examples/px4_simple_app/px4_simple_app.c)는 다음과 같습니다:

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

마지막으로 프로그램을 실행해보십시오:

```sh
px4_simple_app
```

*QGroundControl*을 실행하면, 센서 값을 실시간 플롯으로 확인할 수 있습니다 ([Analyze > MAVLink Inspector](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_inspector.html)).

## 마무리

본 자습서에서 PX4 오토파일럿 프로그램 개발에 필요한 모든 내용을 다루었습니다. 전체 uORB 메시지/토픽의 리스트는 [여기](https://github.com/PX4/Firmware/tree/master/msg/)에 있고, 헤더에 문서화가 잘되어 있으며, 참고삼을 수 있음을 잊지 마시기 바랍니다.

보다 상세한 정보와 문제 해결/흔히 빠지는 함정의 내용은 여기에서 찾을 수 있습니다: [uORB](../middleware/uorb.md).

다음 페이지에서는 시작/멈춤 기능을 가진 완전한 프로그램 작성용 서식을 보여드리겠습니다.