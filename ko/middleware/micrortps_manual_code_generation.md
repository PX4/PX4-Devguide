# 수동으로 클라이언트와 에이전트 코드 생성하기

이번 토픽에서는 어떻게 수동적으로 클라이언트와 에이전트 코드를 생성할지 다룰 것입니다(PX4 펌웨어가 컴파일될 때 [자동적으로 생성되는 것](../middleware/micrortps.md) 대신에)

코드들은 파이썬 스크립트를 통해 생성됩니다: **/Tools/generate_microRTPS_bridge.py**.

## 자동 브릿지 코드 생성 비활성화하기

우선 브릿지 코드 자동생성을 비활성화 해야합니다. 설정하고자 하는 플랫폼의 **.cmake**의 `GENERATE_RTPS_BRIDGE` 변수를 *off*로 설정하세요.

```sh
set(GENERATE_RTPS_BRIDGE off)
```

## generate_microRTPS_bridge.py 스크립트 사용하기

*generate_microRTPS_bridge* 스크립트의 사용법은 아래와 같습니다.

```sh
$ cd /path/to/PX4/Firmware/msg/tools
$ python generate_microRTPS_bridge.py -h
usage: generate_microRTPS_bridge.py [-h] [-s *.msg [*.msg ...]]
                                    [-r *.msg [*.msg ...]] [-a] [-c]
                                    [-t MSGDIR] [-o AGENTDIR] [-u CLIENTDIR]
                                    [-f FASTRTPSGEN]

optional arguments:
  -h, --help            show this help message and exit
  -s *.msg [*.msg ...], --send *.msg [*.msg ...]
                        Topics to be sent
  -r *.msg [*.msg ...], --receive *.msg [*.msg ...]
                        Topics to be received
  -a, --agent           Flag to generate the agent. Default is true.
  -c, --client          Flag to generate the client. Default is true.
  -t MSGDIR, --topic-msg-dir MSGDIR
                        Topics message dir. Default is: msg/
  -o AGENTDIR, --agent-outdir AGENTDIR
                        Agent output dir. Default is:
                        src/modules/micrortps_bridge/micrortps_agent
  -u CLIENTDIR, --client-outdir CLIENTDIR
                        Client output dir. Default is:
                        src/modules/micrortps_bridge/micrortps_client
  -f FASTRTPSGEN, --fastrtpsgen-dir FASTRTPSGEN
                        fastrtpsgen installation dir. Default is: /bin
  --delete-tree         Delete dir tree output dir(s)
```

> **Caution** `--delete-tree` 옵션을 사용하면 새로운 파일과 폴더를 생서하기 전에 `CLIENTDIR`과 `AGENTDIR`의 내용을 지웁니다.

- 파라미터 `--send/-s`과 `--receive/-r`는 PX4가 송수신할 수 있는 uORB 토픽들을 설정합니다. 설정된 메시지들에 대해서만 토픽이 생성됩니다.
- `CLIENTDIR`과 `AGENTDIR` 디렉토리에 파일이 생성되고 (`-o src/modules/micrortps_bridge/micrortps_client`, `-u src/modules/micrortps_bridge/micrortps_agent` 가 기본값).
- `-a`나 `-c` 플래그가 설정되지 않으면, 클라리언트와 에이전트파일이 생성되고 설치됩니다.
- `-f` 옵션은 *Fast RTPS*가 기본 위치에 설치되어 있지 않을때 필요합니다.(`-f /path/to/fastrtps/installation/bin`).

아래의 예제는 하나의 uORB 토픽을 설정한 센서 `sensor_baro`의 브릿지 Pub/Sub 코드를 생성하는 것을 보여줍니다. 

```sh
$ cd /path/to/PX4/Firmware
$ python Tools/generate_microRTPS_bridge.py -s msg/sensor_baro.msg -r msg/sensor_combined.msg
```

## 생성된 코드

*Client*, *Agent*, uORB메시지의 *CDR serialization/deserialization*, 연관된 RTPS 메시지들 정의(IDL 파일들)를 위한 파일들이 생성됩니다.

수동으로 생성된 브릿지 코드는 기본적으로 아래의 폴더에서 찾을 수 있습니다.

- *Client*: **src/modules/micrortps_bridge/micrortps_client/**
- *Agent*: **src/modules/micrortps_bridge/micrortps_agent/**

### uORB serialization 코드

uORB 토픽들을 직렬화하기 위한 함수들은 일반적인 PX4 컴파일 과정에서 생성됩니다(물론 수동으로 만들때도 생성됨). 예를 들어, *sensor_combined.msg*를 위해 다음의 함수들이 생성됩니다.

```sh
void serialize_sensor_combined(const struct sensor_combined_s *input, char *output, uint32_t *length, struct microCDR *microCDRWriter);
void deserialize_sensor_combined(struct sensor_combined_s *output, char *input, struct microCDR *microCDRReader);
```

### RTPS 메시지 IDL 파일

IDL 파일들은 브릿지 코드 생성과정에서 uORB **.msg** 파일로 부터 생성됩니다. ([선택된 uORB 토픽들](../middleware/micrortps.md#supported-uorb-messages)). 생성된 파일은**src/modules/micrortps_bridge/micrortps_agent/idl/** 에서 찾을 수 있습니다.

*FastRTSP*는 RTPS 메시지의 구조체를 정의하기 위핸 IDL 파일을 사용합니다. 이 파일들은 *Agent*와 *FastRTSP* 어플케이션에서 uORB 토픽들을 Pub/Sub하기 위한 코드생성에 사용됩니다.

> **Note** IDL 파일은 *fastrtpsgen* 툴을 이용해 C++로 컴파일 됩니다.

## 코드 생성 검증하기

출력 폴더에서 아래의 목록과 일치하는지 확인함으로써 코드가 제대로 생성되었는지 검증할 수 있습니다(리눅스에서 `tree` 명령어가 도움이 될 것입니다).

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

Client directory:

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

## 빌드, 코드 사용하기

수동으로 생성된 *Client* 코드는 빌드후에 *정확히* [자동으로 생성된 코드](../middleware/micrortps.md#client_firmware)와 동일한 방법으로 사용됩니다.

특히, 수동으로 한번 생성되면 일반적인 빌드 과정에서 *Client* 소스 코드는 컴파일되고 빌드되어 PX4의 펌웨어에 포함됩니다. 예를 들어, 컴파일 하고 NuttX/Pixhawk 타켓 펌웨어에 포함하려고 하는 경우:

```sh
make px4_fmu-v4_default upload
```

> **Note** 우선 반드시 [자동 코드 생성을 비활성화](#disable-automatic-bridge-code-generation) 해야합니다. 그래야 툴체인이 수동으로 생성되 코드를 사용합니다(다시 생성하는 걸 시도하지도 않음).

수동으로 생성된 *Agent* 코드는 빌드후에 [자동으로 생성된 코드](../middleware/micrortps.md#agent-in-a-ros-independent-offboard-fast-rtps-interface)와 동일한 방법으로 사용됩니다. 수동 코드생성의 유일한 차이는 **<emphasis>build/BUILDPLATFORM</emphasis>****/src/modules/micrortps_bridge/micrortps_agent/** 대신에 **src/modules/micrortps_bridge/micrortps_agent**에 생성되는 것입니다.