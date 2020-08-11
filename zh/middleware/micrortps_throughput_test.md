# Micro RTPS 吞吐量测试

这是一个测量 [PX4-FastRTPS Bridge](../middleware/micrortps.md) 吞吐量的简单测试。 最大速率下，同时收发 256 字节的报文，并且输出结果。

> **Tip** 该示例需要你 [手动生成客户端和代理代码](../middleware/micrortps_manual_code_generation.md)。

## 使用 uORB 报文

首先，在 **Firmware/msg/** 目录下创建一个新的 uORB 报文。 可以命名为 **throughput_256.msg** 并包含如下内容：

```text
uint8[256] data
```

可以使用如下命令：

```sh
cd /path/to/PX4/Firmware/msg
echo uint8[256] data > throughput_256.msg
```

注册新的报文，添加到如下文件中：**/Firmware/msg/CMakeLists.txt**：

```cmake
...
wind_estimate.msg
throughput_256.msg
)
...
```

通过在脚本 **/Firmware/Tools/message_id.py** 中添加一行，制定一个该报文的话题 Id ：

```python
...
    'wind_estimate': 94,
    'throughput_256': 95,
}
...
```

## 禁用自动桥接代码生成

找到对应的目标平台（*cmake/configs/*），通过设置 *.cmake* 文件中的变量 `GENERATE_RTPS_BRIDGE` 来禁用自动桥接代码生成（作为 PX4 构建进程的一部分）：

```sh
set(GENERATE_RTPS_BRIDGE off)
```

## 生成桥接代码

使用 *generate_microRTPS_bridge.py* 手动生成桥接代码（代码会发送和接收我们刚刚加入的 `throughput_256` uORB 话题报文）：

```sh
cd /path/to/PX4/Firmware
python Tools/generate_microRTPS_bridge.py --send msg/throughput_256.msg --receive msg/throughput_256.msg
```

*Client* 源代码生成在 **src/modules/micrortps_bridge/micrortps_client/**，*Agent* 则在 **src/modules/micrortps_bridge/micrortps_agent/**。

### 更改客户端代码

接下来，我们修改 *Client* 用来在每次循环中发送 *throughput_256* 报文。 这是必需的，因为 PX4 实际上并没有发布该主题，而且我们希望确保以尽可能高的速率发送该主题。

打开文件 **src/modules/micrortps_bridge/micrortps_client/microRTPS_client.cpp**。 更新 `send()` 函数中的 `while` 循环，使其如下所示:

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

> **Note** 你可能还记得，这是一个 *bidirectional* 吞吐量测试，其中还必须将消息从 *Agent* 发送到 *Client*。 你不需要修改代理代码就可以实现这一点。 由于 *Agent* 是 RTPS 发布者和订阅者，它将自动收到有关其发送的 RTPS 消息的通知，然后将这些消息镜像回客户端。

[Compileand launch](../middleware/micrortps_manual_code_generation.md#build-and-use-the-code) ： *Client* 和 *Agent*。

## 结果

测试是在 Pixracer 上运行的 PX4, 通过 UART 连接到运行 Ubuntu 16.04 的普通 PC。 默认配置用于两个客户/代理。

吞吐量可以在 shell 窗口中观察到的结果如下：

```sh
SENT:     13255 messages in 13255 LOOPS, 3512575 bytes in 30.994 seconds - 113.33KB/s
RECEIVED: 13251 messages in 10000 LOOPS, 3511515 bytes in 30.994 seconds - 113.30KB/s
```