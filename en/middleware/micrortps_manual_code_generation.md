# Manually generate Client and Agent code

It is also possible to generate and install the code for the client and the agent outside the normal PX4 build process using the python script
**Tools/generate_microRTPS_bridge.py**.

First at all, we need to disable the automatic generation in the PX4 compiling process setting the variable `GENERATE_RTPS_BRIDGE` to *off* inside the *.cmake* file for the target platform:

```sh
set(GENERATE_RTPS_BRIDGE off)
```

The tool's command syntax is shown below:

```sh
$ cd /path/to/PX4/Firmware
$ python Tools/generate_microRTPS_bridge.py -h
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
  -a, --agent           Flag for generate the agent, by default is true if -c
                        is not specified
  -c, --client          Flag for generate the client, by default is true if -a
                        is not specified
  -t MSGDIR, --topic-msg-dir MSGDIR
                        Topics message dir, by default msg/
  -o AGENTDIR, --agent-outdir AGENTDIR
                        Agent output dir, by default
                        src/modules/micrortps_bridge/micrortps_agent
  -u CLIENTDIR, --client-outdir CLIENTDIR
                        Client output dir, by default
                        src/modules/micrortps_bridge/micrortps_client
  -f FASTRTPSGEN, --fastrtpsgen-dir FASTRTPSGEN
                        fastrtpsgen installation dir, by default /bin
  --delete-tree         Delete dir tree output dir(s)
```
  > **Caution with --delete-tree option** so erases the content of the `CLIENTDIR` and the `AGENTDIR` before creating new files and folders.

- The argument `--send/-s` means that the application from PX4 side will send these messages, and the argument `--receive/-r` specifies which messages are going to be received.
- The output appears in `CLIENTDIR` (`-o src/modules/micrortps_bridge/micrortps_client`, by default) and in the `AGENTDIR` (`-u src/modules/micrortps_bridge/micrortps_agent`, by default).
- If no flag `-a` or `-c` is specified, both the client and the agent will be generated and installed.
- The `-f` option may be needed if *Fast RTPS* was not installed in the default location (`-f /path/to/fastrtps/installation/bin`).

An example of use:

```sh
$ cd /path/to/PX4/Firmware
$ python Tools/generate_microRTPS_bridge.py -s msg/sensor_baro.msg -r msg/sensor_combined.msg
```

Checking the correct generation and installation:

```sh
$ tree src/modules/micrortps_bridge/micrortps_agent
src/modules/micrortps_bridge/micrortps_agent
├── build
├── CMakeLists.txt
├── idl
│   ├── sensor_baro_.idl
│   └── sensor_combined_.idl
├── microRTPS_agent.cxx
├── microRTPS_transport.cxx
├── microRTPS_transport.h
├── RtpsTopics.cxx
├── RtpsTopics.h
├── sensor_baro_.cxx
├── sensor_baro_.h
├── sensor_baro_Publisher.cxx
├── sensor_baro_Publisher.h
├── sensor_baro_PubSubTypes.cxx
├── sensor_baro_PubSubTypes.h
├── sensor_combined_.cxx
├── sensor_combined_.h
├── sensor_combined_PubSubTypes.cxx
├── sensor_combined_PubSubTypes.h
├── sensor_combined_Subscriber.cxx
└── sensor_combined_Subscriber.h
 2 directories, 20 files
```

```sh
$ tree src/modules/micrortps_bridge/micrortps_client
src/modules/micrortps_bridge/micrortps_client
├── CMakeLists.txt
├── microRTPS_client.cpp
├── microRTPS_transport.cxx
└── microRTPS_transport.h
 0 directories, 4 files
```

