# 手动生成客户端和代理端代码

本主题演示如何在编译 PX4 固件时手动生成客户端和代理的代码（而不是 [automatically generating](../middleware/micrortps.md)）。

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

> **Caution** Using with `--delete-tree` option erases the content of the `CLIENTDIR` and the `AGENTDIR` before creating new files and folders.

- The arguments `--send/-s` and `--receive/-r` specify the uORB topics that can be sent/received from PX4. Code will only be generated for specified messages.
- The output appears in `CLIENTDIR` (`-o src/modules/micrortps_bridge/micrortps_client`, by default) and in the `AGENTDIR` (`-u src/modules/micrortps_bridge/micrortps_agent`, by default).
- If no flag `-a` or `-c` is specified, both the client and the agent will be generated and installed.
- The `-f` option may be needed if *Fast RTPS* was not installed in the default location (`-f /path/to/fastrtps/installation/bin`).

The example below shows how you can generate bridge code to publish/subscribe just the `sensor_baro` single uORB topic.

```sh
$ cd /path/to/PX4/Firmware
$ python Tools/generate_microRTPS_bridge.py -s msg/sensor_baro.msg -r msg/sensor_combined.msg
```

## 生成代码

Code is generated for the *Client*, *Agent*, *CDR serialization/deserialization* of uORB messages, and the definition of the associated RTPS messages (IDL files).

Manually generated code for the bridge can be found here (by default):

- *Client*: **src/modules/micrortps_bridge/micrortps_client/**
- *Agent*: **src/modules/micrortps_bridge/micrortps_agent/**

### uORB serialization code

Serialization functions are generated for all the uORB topics as part of the normal PX4 compilation process (and also for manual generation). For example, the following functions would be generated for the *sensor_combined.msg*:

```sh
void serialize_sensor_combined(const struct sensor_combined_s *input, char *output, uint32_t *length, struct microCDR *microCDRWriter);
void deserialize_sensor_combined(struct sensor_combined_s *output, char *input, struct microCDR *microCDRReader);
```

### RTPS message IDL files

IDL files are generated from the uORB **.msg** files ([for selected uORB topics](../middleware/micrortps.md#supported-uorb-messages)) in the generation of the bridge. These can be found in: **src/modules/micrortps_bridge/micrortps_agent/idl/**

*FastRTSP* uses IDL files to define the structure of RTPS messages (in this case, RTPS messages that map to uORB topics). They are used to generate code for the *Agent*, and *FastRTSP* applications that need to publish/subscribe to uORB topics.

> **Note** IDL files are compiled to C++ by the *fastrtpsgen* tool.

## 代码生成验证

You can verify successful code generation by checking that the output directories match the listing shown below (On Linux, the `tree` command can be used for listing the file structure).

Agent directory:

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

The manually generated *Client* code is built and used in *exactly* the same way as [automatically generated Client code](../middleware/micrortps.md#client-px4-firmware).

Specifically, once manually generated, the *Client* source code is compiled and built into the PX4 firmware as part of the normal build process. For example, to compile the code and include it in firmware for NuttX/Pixhawk targets:

```sh
make px4_fmu-v4_default upload
```

> **Note** You must first [disable automatic bridge code generation](#disable-automatic-bridge-code-generation) so that the toolchain uses the manually generated source code (and does not attempt to regenerate it).

The manually generated *Agent* code is also compiled and used in the same way as the [automatically generated code](../middleware/micrortps.md#agent-off-board-fastrtps-interface). The only difference is that the manually source code is created in **src/modules/micrortps_bridge/micrortps_agent** instead of **<emphasis>build/BUILDPLATFORM</emphasis>****/src/modules/micrortps_bridge/micrortps_agent/**.