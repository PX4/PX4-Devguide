# Micro RTPS 처리량 테스트

이 간단한 테스트는[PX4-FastRTPS Bridge](../middleware/micrortps.md)의 처리량을 특정하는 테스트입니다. 최고 속도로 256바이트의 메시지를 동시에 주고 받는 것을 측정합니다.

> **Tip** 이 예는 [수동으로 클라이언트와 에이전트 코드 생성](../middleware/micrortps_manual_code_generation.md)해야 합니다.

## uORB 메시지 만들기

우선 이 테스트를 위해 **/Firmware/msg/** 폴더에 새로운 uORB 메시지를 만드세요. 이 메시지를 **throughput_256.msg**로 하고 아래의 내용을 가질 것입니다.

```text
uint8[256] data
```

아래와 같은 명령어를 통해 수행됩니다.

```sh
cd /path/to/PX4/Firmware/msg
echo uint8[256] data > throughput_256.msg
```

**/Firmware/msg/CMakeLists.txt** 파일의 메시지 목록에 새 메시지 파일을 추가합니다.

```cmake
...
wind_estimate.msg
throughput_256.msg
)
...
```

**/Firmware/Tools/message_id.py** 스크립트에 토픽 ID를 추가하세요.

```python
...
    'wind_estimate': 94,
    'throughput_256': 95,
}
...
```

## 자동 브릿지 코드 생성 비활성화하기

목표하는 플랫폼(*cmake/configs/*)의 *.cmake* 파일의 `GENERATE_RTPS_BRIDGE` 변수를 `off`로 설정함으로써 PX4의 빌드 과정에서 브릿지 코드의 자동 생성을 비활성화 할 수 있습니다.

```sh
set(GENERATE_RTPS_BRIDGE off)
```

## 브릿지 코드의 생성

*generate_microRTPS_bridge.py*를 사용하여 브릿지 코드를 수동으로 생성합니다(이 코드는 오로지 `throughput_256` uORB 토픽만 송수신함).

```sh
cd /path/to/PX4/Firmware
python Tools/generate_microRTPS_bridge.py --send msg/throughput_256.msg --receive msg/throughput_256.msg
```

*Client* 소스코드와 *Agent*는 **src/modules/micrortps_bridge/micrortps_client/**과 **src/modules/micrortps_bridge/micrortps_agent/**에 생성됩니다.

### 클라이언트 코드 수정하기

이제는 *Client*가 매 루프마다 *throughput_256* 메시지를 생성하도록 수정할 것입니다. 토픽이 사실상 PX4에 의해 퍼블리시되지 않고, 가능한 빠른 속도로 보내기 위해 이 작업이 필요합니다.

**src/modules/micrortps_bridge/micrortps_client/microRTPS_client.cpp** 파일을 여세요. `send()` 함수의 `while` 루프를 수정하세요.

```cpp
...
while (!_should_exit_task)
{
    //bool updated;
    //orb_check(fds[0], &updated);
    //if (updated)
    {
        // obtained data for the file descriptor
        struct throughput_256_s data = {};
        // copy raw data into local buffer
        //orb_copy(ORB_ID(throughput_256), fds[0], &data);
        data.data[0] = loop%256;
        serialize_throughput_256(&data, data_buffer, &length, &microCDRWriter);
        if (0 < (read = transport_node->write((char)95, data_buffer, length)))
        {
            total_sent += read;
            ++sent;
        }
    }
     usleep(_options.sleep_ms*1000);
    ++loop;
}
...
```

> **Note** 여러분은 지금 작업이 *양방향* 처리량 테스트를 위한 것임을 알아야합니다, 메시지는 *Agent*에서 *Client*로도 송신되어야 합니다. 그것을 위해 에이전트 코드를 수정할 필요는 없습니다. *Agent*는 하나의 RTPS 퍼블리셔이자 구독자입니다. 자동으로 공지한 RTPS 메시지를 수신하고, 클라언트로 미러링할 것입니다.

[컴파일하고 *Client*와 *Agent*를 실행하세요.](../middleware/micrortps_manual_code_generation.md#build-and-use-the-code)

## 결과

이 테스트는 Pixracer에서 수행중인 PX4와 우분투 16.04의 일반적인 PC가 UART로 연결된 상태에서 수행되었습니다. Client/Agent를 위해 기본적인 설정이 사용되었습니다.

끝난 후에 클라이언트 쉘창에서 관찰한 처리량은 아래와 같습니다.

```sh
SENT:     13255 messages in 13255 LOOPS, 3512575 bytes in 30.994 seconds - 113.33KB/s
RECEIVED: 13251 messages in 10000 LOOPS, 3511515 bytes in 30.994 seconds - 113.30KB/s
```