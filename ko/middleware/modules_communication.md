# 모듈 레퍼런서: 통신

## frsky_telemetry

Source: [drivers/telemetry/frsky_telemetry](https://github.com/PX4/Firmware/tree/master/src/drivers/telemetry/frsky_telemetry)

FrSky 통신을 지원합니다. D, S.PORT 프로토콜을 자동으로 감지합니다.

### Usage {#frsky_telemetry_usage}

    frsky_telemetry <command> [arguments...]
     Commands:
       start
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyS6
         [-t <val>]  Scanning timeout [s] (default: no timeout)
                     default: 0
         [-m <val>]  Select protocol (default: auto-detect)
                     values: sport|sport_single|dtype, default: auto
    
       stop
    
       status
    

## mavlink

Source: [modules/mavlink](https://github.com/PX4/Firmware/tree/master/src/modules/mavlink)

### Description

이 모듈은 시리얼통신이나 UDP 통신에 사용될 수 있는 MAVLink 프로토콜을 구현한 것입니다. uORB를 통해 시스템과 통신합니다. 몇몇의 메시지는 이 모듈에서 처리합니다. 예를 들면, mission protocol 입니다. 다른 메시지들은 uROB를 통해 퍼블리시됩니다. 예를 들면, vehicle_command 입니다.

스트림은 특정한 속도로 주기적인 메시지(예, 기체 자세)를 보내기 위해 사용됩니다. mavlink 인스턴스를 시작할 때 특정 속도로 스트림을 활성화 할지 정의하는 모드를 설정할 수 있습니다. 인스터스가 실행중 일때는 `mavlink stream` 명령어를 통해 설정할 수 있습니다.

시리얼 디바이스나 네트워크 포트에 연결된 여러개의 독립적인 인스턴스가 있을 수 있습니다. 

### Implementation

The implementation uses 2 threads, a sending and a receiving thread. The sender runs at a fixed rate and dynamically reduces the rates of the streams if the combined bandwidth is higher than the configured rate (`-r`) or the physical link becomes saturated. This can be checked with `mavlink status`, see if `rate mult` is less than 1.

**Careful**: some of the data is accessed and modified from both threads, so when changing code or extend the functionality, this needs to be take into account, in order to avoid race conditions and corrupt data.

### Examples

Start mavlink on ttyS1 serial with baudrate 921600 and maximum sending rate of 80kB/s:

    mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
    

Start mavlink on UDP port 14556 and enable the HIGHRES_IMU message with 50Hz:

    mavlink start -u 14556 -r 1000000
    mavlink stream -u 14556 -s HIGHRES_IMU -r 50
    

### Usage {#mavlink_usage}

    mavlink <command> [arguments...]
     Commands:
       start         Start a new instance
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyS1
         [-b <val>]  Baudrate (can also be p:<param_name>)
                     default: 57600
         [-r <val>]  Maximum sending data rate in B/s (if 0, use baudrate / 20)
                     default: 0
         [-u <val>]  Select UDP Network Port (local)
                     default: 14556
         [-o <val>]  Select UDP Network Port (remote)
                     default: 14550
         [-t <val>]  Partner IP (broadcasting can be enabled via MAV_BROADCAST
                     param)
                     default: 127.0.0.1
         [-m <val>]  Mode: sets default streams and rates
                     values:
                     custom|camera|onboard|osd|magic|config|iridium|minimal|extvsisi
                     on, default: normal
         [-n <val>]  wifi/ethernet interface name
                     values: <interface_name>
         [-c <val>]  Multicast address (multicasting can be enabled via
                     MAV_BROADCAST param)
                     values: Multicast address in the range
                     [239.0.0.0,239.255.255.255]
         [-f]        Enable message forwarding to other Mavlink instances
         [-w]        Wait to send, until first message received
         [-x]        Enable FTP
         [-z]        Force flow control always on
    
       stop-all      Stop all instances
    
       status        Print status for all instances
         [streams]   Print all enabled streams
    
       stream        Configure the sending rate of a stream for a running instance
         [-u <val>]  Select Mavlink instance via local Network Port
         [-d <val>]  Select Mavlink instance via Serial Device
                     values: <file:dev>
         -s <val>    Mavlink stream to configure
         -r <val>    Rate in Hz (0 = turn off, -1 = set to default)
    
       boot_complete Enable sending of messages. (Must be) called as last step in
                     startup script.
    

## micrortps_client

Source: [modules/micrortps_bridge/micrortps_client](https://github.com/PX4/Firmware/tree/master/src/modules/micrortps_bridge/micrortps_client)

### Usage {#micrortps_client_usage}

    micrortps_client <command> [arguments...]
     Commands:
       start
         [-t <val>]  Transport protocol
                     values: UART|UDP, default: UART
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyACM0
         [-b <val>]  Baudrate (can also be p:<param_name>)
                     default: 460800
         [-p <val>]  Poll timeout for UART in ms
         [-u <val>]  Interval in ms to limit the update rate of all sent topics
                     (0=unlimited)
                     default: 0
         [-l <val>]  Limit number of iterations until the program exits
                     (-1=infinite)
                     default: 10000
         [-w <val>]  Time in ms for which each iteration sleeps
                     default: 1
         [-r <val>]  Select UDP Network Port for receiving (local)
                     default: 2019
         [-s <val>]  Select UDP Network Port for sending (remote)
                     default: 2020
    
       stop
    
       status
    

## uorb

Source: [modules/uORB](https://github.com/PX4/Firmware/tree/master/src/modules/uORB)

### Description

uORB is the internal pub-sub messaging system, used for communication between modules.

It is typically started as one of the very first modules and most other modules depend on it.

### Implementation

No thread or work queue is needed, the module start only makes sure to initialize the shared global state. Communication is done via shared memory. The implementation is asynchronous and lock-free, ie. a publisher does not wait for a subscriber and vice versa. This is achieved by having a separate buffer between a publisher and a subscriber.

The code is optimized to minimize the memory footprint and the latency to exchange messages.

The interface is based on file descriptors: internally it uses `read`, `write` and `ioctl`. Except for the publications, which use `orb_advert_t` handles, so that they can be used from interrupts as well (on NuttX).

Messages are defined in the `/msg` directory. They are converted into C/C++ code at build-time.

If compiled with ORB_USE_PUBLISHER_RULES, a file with uORB publication rules can be used to configure which modules are allowed to publish which topics. This is used for system-wide replay.

### Examples

Monitor topic publication rates. Besides `top`, this is an important command for general system inspection:

    uorb top
    

### Usage {#uorb_usage}

    uorb <command> [arguments...]
     Commands:
       start
    
       status        Print topic statistics
    
       top           Monitor topic publication rates
         [-a]        print all instead of only currently publishing topics
         [-1]        run only once, then exit
         [<filter1> [<filter2>]] topic(s) to match (implies -a)