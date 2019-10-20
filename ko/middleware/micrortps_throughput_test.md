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

Disable automatic generation of bridge code (as part of the PX4 build process) by setting the variable `GENERATE_RTPS_BRIDGE` to `off` in the *.cmake* file for the target platform (*cmake/configs/*):

```sh
set(GENERATE_RTPS_BRIDGE off)
```

## Generate the bridge code

Manually generate bridge code using *generate_microRTPS_bridge.py* (the code will send and receive "just" our `throughput_256` uORB topic):

```sh
cd /path/to/PX4/Firmware
python Tools/generate_microRTPS_bridge.py --send msg/throughput_256.msg --receive msg/throughput_256.msg
```

The *Client* source code is generated in **src/modules/micrortps_bridge/micrortps_client/** and the *Agent* in **src/modules/micrortps_bridge/micrortps_agent/**.

### Modify the client code

Next we modify the *Client* to send a *throughput_256* message on every loop. This is required because the topic is not actually being published by PX4, and because we want to ensure that it is sent at the greatest possible rate.

Open the file **src/modules/micrortps_bridge/micrortps_client/microRTPS_client.cpp**. Update the `while` loop in the `send()` function to look like this:

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

> **Note** You may recall this is intended to be a *bidirectional* throughput test, where messages must also be sent from the *Agent* to the *Client*. You do not need to modify the Agent code to make this happen. As the *Agent* is an RTPS publisher and subscriber, it will automatically get notified of the RTPS messages it sends, and will then mirror these back to the client.

[Compile and launch](../middleware/micrortps_manual_code_generation.md#build-and-use-the-code) both the *Client* and the *Agent*.

## Result

The test was executed with PX4 running on Pixracer, connected via a UART to an ordinary PC running Ubuntu 16.04. The default configuration was used for both the Client/Agent.

The throughput that was observed in the client shell window on completion is shown below:

```sh
SENT:     13255 messages in 13255 LOOPS, 3512575 bytes in 30.994 seconds - 113.33KB/s
RECEIVED: 13251 messages in 10000 LOOPS, 3511515 bytes in 30.994 seconds - 113.30KB/s
```