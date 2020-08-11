# 手动生成客户端和代理端代码

本主题演示如何手动生成客户端和代理的代码（而不是编译 PX4 时[自动生成](../middleware/micrortps.md)的）。

代码是使用 python 脚本生成的：**/Tools/generate_microRTPS_bridge.py**。

## 禁用自动桥接代码生成

首先禁用桥接代码的自动生成。 将目标平台的 **.cmake** 文件中的变量 `GENERATE_RTPS_BRIDGE` 设置为 *off*：

```sh
set(GENERATE_RTPS_BRIDGE off)
```

## 使用 generate_microRTPS_bridge. py

*generate_microRTPS_bridge* 工具的命令语法如下所示:

```sh
$ cd /path/to/PX4/Firmware/msg/tools
$ python generate_microRTPS_bridge.py -h
usage: generate_microRTPS_bridge.py [-h] [-s *.msg [*.msg ...]]
                                    [-r *.msg [*.msg ...]] [-a] [-c]
                                    [-t MSGDIR] [-o AGENTDIR] [-u CLIENTDIR]
                                    [-f FASTRTPSGEN]

optional arguments:
  -h, --help            显示这个帮助信息并退出
  -s *.msg [*.msg ...], --send *.msg [*.msg ...]
                        要发送的 Topic
  -r *.msg [*.msg ...], --receive *.msg [*.msg ...]
                        要接收的 Topic
  -a, --agent           生成 agent 的参数。 默认值为 true。
  -c, --client          生成客户端的标志位。 默认值为 true。
  -t MSGDIR, --topic-msg-dir MSGDIR
                        主题消息目录。 默认为： msg/
  -o AGENTDIR, --agent-outdir AGENTDIR
                        Agent 输出目录。 Src/modules/micrortps_bridge/micrortps_agent
  -u CLIENTDIR, --client-outdir CLIENTDIR
                        客户端输出目录。 默认是：
                        src/modules/micrortps_bridge/micrortps_client
  -f FASTRTPSGEN, --fastrtpsgen-dir FASTRTPSGEN
                        fastrtpsgen 安装目录。 默认是： /bin
  --delete-tree         删除目录树
```

> **Caution** 在创建新文件和文件夹之前，使用 `--delete-tree` 选项会删除 `CLIENTDIR` 和 `AGENTDIR` 的内容。

- `--send/-s` 的参数和 `--receive/-r` 指定可以从 PX4 接收到的 uORB 主题。 将仅为指定的消息生成代码。
- 输出显示在 `CLIENTDIR` (默认情况下 ` src/modules/micrortps_bridge/micrortps_client</0 >) 和 <code>AGENTDIR` (默认情况下 `-u src/modules/micrortps_bridge/micrortps_agent</0 >) 中。</li>
<li>如果未指定标志 <code>-a` 或 `-c`，则将生成并安装客户端和代理。
- 如果未在默认位置（`-f /path/to/fastrtps/installation/bin`）安装 *Fast rtps*，则可能需要 `-f` 选项。

下面的示例演示如何生成桥接代码以发布/订阅 `sensor_baro` 单个 uORB 主题。

```sh
$ cd /path/to/PX4/Firmware
$ python Tools/generate_microRTPS_bridge.py -s msg/sensor_baro.msg -r msg/sensor_combined.msg
```

## 生成代码

为 *Client*、*Agent*、*CDR serialization/deserialization* 的 uORB 消息以及关联的 RTPS 报文 (IDL 文件) 的定义生成代码。

可以在此处找到网桥的手动生成的代码（默认情况下）：

- *客户端*: **src/modules/micrortps_bridge/micrortps_client/**
- *代理端*: **src/modules/micrortps_bridge/micrortps_agent/**

### uORB 序列化代码

序列化函数是为所有 uORB 主题生成的，作为正常 PX4 编译过程（以及手动生成）的一部分。 例如，将为 *sensor_combined.msg* 生成以下函数：

```sh
void serialize_sensor_combined(const struct sensor_combined_s *input, char *output, uint32_t *length, struct microCDR *microCDRWriter);
void deserialize_sensor_combined(struct sensor_combined_s *output, char *input, struct microCDR *microCDRReader);
```

### RTPS 报文 IDL 文件

IDL 文件是从网桥生成过程中的 uORB 的 **.msg** 文件中 ([选定的 uORB 主题 ](../middleware/micrortps.md#supported-uorb-messages)) 生成的。 这些可以在 **src/modules/micrortps_bridge/micrortps_agent/idl/** 中找到。

*FastRTSP* 使用 IDL 文件来定义 RTPS 消息的结构（在本例中，映射到 uORB 主题的 RTPS 消息）。 它们用于生成 *Agent* 的代码，和 *FastRTSP* 需要发布/订阅 uORB 主题的应用程序。

> **Note** IDL 文件由 *fastrtpsgen* 工具编译到 c++。

## 代码生成验证

可以通过检查输出目录是否与下面显示的列表匹配来验证成功的代码生成（在 Linux 上，`tree` 命令可用于列出文件结构）。

代理目录:

```sh
$ tree src/modules/micrortps_bridge/micrortps_agent
src/modules/micrortps_bridge/micrortps_agent
├── build
├── CMakeLists.txt
├── idl
│   ├── sensor_baro_.idl
│   └── sensor_combined_.idl
├── microRTPS_agent.cpp
├── microRTPS_transport.cpp
├── microRTPS_transport.h
├── RtpsTopics.cpp
├── RtpsTopics.h
├── sensor_baro_.cpp
├── sensor_baro_.h
├── sensor_baro_Publisher.cpp
├── sensor_baro_Publisher.h
├── sensor_baro_PubSubTypes.cpp
├── sensor_baro_PubSubTypes.h
├── sensor_combined_.cpp
├── sensor_combined_.h
├── sensor_combined_PubSubTypes.cpp
├── sensor_combined_PubSubTypes.h
├── sensor_combined_Subscriber.cpp
└── sensor_combined_Subscriber.h
 2 directories, 20 files
```

客户端目录：

```sh
$ tree src/modules/micrortps_bridge/micrortps_client
src/modules/micrortps_bridge/micrortps_client
├── CMakeLists.txt
├── microRTPS_client.cpp
├── microRTPS_client_dummy.cpp
├── microRTPS_client_main.cpp
├── microRTPS_transport.cpp
└── microRTPS_transport.h
 0 directories, 4 files
```

## 构建并使用代码

The manually generated *Client* code is built and used in *exactly* the same way as [automatically generated Client code](../middleware/micrortps.md#client_firmware).

特别的，一旦手动生成，*Client* 代码就被编译并构建作为 PX4 固件的一个正常编译的一部分。 例如，编译代码，将其加入 NuttX/Pixhawk 固件：

```sh
make px4_fmu-v4_default upload
```

> **Note** 必须首先 [disable automatic bridge code generation ](#disable-automatic-bridge-code-generation)，以便工具链使用手动生成的源代码（并且不尝试重新生成）。

The manually generated *Agent* code is also compiled and used in the same way as the [automatically generated code](../middleware/micrortps.md#agent-in-a-ros-independent-offboard-fast-rtps-interface). 唯一的区别是手动源代码是以 **src/modules/micrortps_bridge/micrortps_agent** 而不是 **<emphasis>build/BUILDPLATFORM</emphasis>****/src/modules/micrortps_bridge/micrortps_agent/** 创建的。