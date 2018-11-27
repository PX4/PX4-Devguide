# 수동으로 Client와 Agent 코드 생성

여기서는 수동으로 client와 agent에 대한 코드를 생성하는 방법을 설명합니다.(PX4 펌웨어가 컴파일될 때 [자동 생성](../middleware/micrortps.md)됩니다.)

생성된 코드는 파이썬 스크립트를 사용합니다 : **/Tools/generate_microRTPS_bridge.py**


## 자동 브리지 코드 생성 비활성화 {#disable-automatic-bridge-code-generation}

먼저 브리지 코드의 자동 생성을 비활성화 시킵니다. 타겟 플랫폼에 대해서 **.cmake** 파일에 있는 `GENERATE_RTPS_BRIDGE` 변수를 *off* 로 설정합니다:

```sh
set(GENERATE_RTPS_BRIDGE off)
```

## generate_microRTPS_bridge.py 사용하기

*generate_microRTPS_bridge* 툴의 명령 문법은 아래와 같습니다:

```sh
$ cd /path/to/PX4/Firmware/Tools
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

> **Caution**  `--delete-tree` 옵션을 사용하면 새로운 파일과 폴더를 생성하기 전에 `CLIENTDIR`와 `AGENTDIR`의 내용을 삭제합니다.

- `--send/-s`와 `--receive/-r`는 PX4와 주고 받을 수 있는 uORB topic들을 지정합니다. 코드는 지정한 메시지에 대해서만 생성됩니다.
- `CLIENTDIR` (`-o src/modules/micrortps_bridge/micrortps_client`, 기본)와 `AGENTDIR` (`-u src/modules/micrortps_bridge/micrortps_agent`, 기본)에 출력됩니다.
- `-a`나 `-c` flag가 지정되지 않으면 client와 agent 모두 생성 및 설치됩니다.
- *Fast RTPS*가 기본 위치(`-f /path/to/fastrtps/installation/bin`)에 설치되지 않았다면 `-f` 옵션이 필요할 수도 있습니다.

아래 예제에서는 `sensor_baro` single uORB topic publish/subscribe에 대한 브리지 코드를 생성하는 방법을 보여줍니다.

```sh
$ cd /path/to/PX4/Firmware
$ python Tools/generate_microRTPS_bridge.py -s msg/sensor_baro.msg -r msg/sensor_combined.msg
```

## 생성된 코드

*Client*, *Agent*, uORB messages의 *CDR serialization/deserialization* 그리고 RTPS 메시지와 관련된 정의(IDL 파일)에 대해서 코드가 생성됩니다.

브리지를 위해서 수동으로 생성된 코드는 다음에서 찾을 수 있습니다.(기본 설정) :

- *Client*: **src/modules/micrortps_bridge/micrortps_client/**
- *Agent*: **src/modules/micrortps_bridge/micrortps_agent/**


### uORB serialization 코드

Serialization functions are generated for all the uORB topics as part of the normal PX4 compilation process (and also for manual generation). For example, the following functions would be generated for the *sensor_combined.msg*:

```sh
void serialize_sensor_combined(const struct sensor_combined_s *input, char *output, uint32_t *length, struct microCDR *microCDRWriter);
void deserialize_sensor_combined(struct sensor_combined_s *output, char *input, struct microCDR *microCDRReader);
```

### RTPS message IDL files

IDL 파일은 ([선택한 uORB topics](../middleware/micrortps.md#supported-uorb-messages))에 대해서 uORB **.msg** 파일로부터 생성됩니다. 다음에서 찾을 수 있습니다. : **src/modules/micrortps_bridge/micrortps_agent/idl/**

*FastRTSP* IDL 파일을 사용해서 RTPS 메시지의 구조를 정의합니다.(이 경우 uORB topic으로 RTPS 메시지를 매핑) uORB topic에 publish/subsribe에 필요한 *Agent*와 *FastRTSP* 어플리케이션을 위한 코드를 생성하는데 사용합니다.

> **Note** IDL 파일은 *fastrtpsgen* 툴을 사용해서 C++로 컴파일됩니다.


## 코드 생성 검증

결과 디렉토리를 검사해서 성공적으로 코드가 생성되었는지를 검증할 수 있습니다. (Linux에서 `tree` 명령을 사용해서 파일 구조를 살펴볼 수 있습니다.)

Agent 디렉토리:
```sh
$ tree src/modules/micrortps_bridge/micrortps_agent
src/modules/micrortps_bridge/micrortps_agent
├── build
├── CMakeLists.txt
├── idl
│   ├── sensor_baro_.idl
│   └── sensor_combined_.idl
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

Client 디렉토리:
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

## 빌드와 코드 사용 {#build-and-use-the-code}

수동으로 생성한 *Client* 코드가 빌드해서  [자동으로 생성한 Client code](../middleware/micrortps.md#client-px4-firmware)와 정확히 같은 방식으로 사용합니다.

특히 한번 수동으로 생성되면, *Client* 소스 코드가 컴파일되고 일반 빌드 프로세스로 PX4 펌웨어로 빌드됩니다. 예제로 코드를 컴파일하려면 펌웨어에 Nuttx/Pixhawk 타겟에 include합니다.:

```sh
make px4_fmu-v4_default upload
```

> **Note** 먼저 [자동 bridge 코드 생성 비활성화](#disable-automatic-bridge-code-generation)시켜서 툴체인이 수동으로 생성한 소스 코드를 사용하도록 합니다. (그리고 재생성하지 않도록 합니다)

수동으로 생성한 *Agent* 코드도 컴파일되고 [자동으로 생성된 코드](../middleware/micrortps.md#agent-off-board-fastrtps-interface)와 같은 방식으로 사용됩니다. 유일한 차이점은 수동으로 생성된 소스코드는 <strong><emphasis>build/BUILDPLATFORM</emphasis></strong>**/src/modules/micrortps_bridge/micrortps_agent/** 대신에 **src/modules/micrortps_bridge/micrortps_agent**에 생성됩니다.
