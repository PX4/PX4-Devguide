# ULog 파일 형식

ULog는 시스템 데이터를 기록할 떄 사용하는 파일 형식입니다.

형식은 자체적 설명이 들어있으며, 예를 들면, 로깅한 내용에 형식 정보와 메세지 형식이 들어있습니다(참고로 [시스템 로거](../log/logging.md)에서는 로그 토픽의 *기본 설정*을 SD 카드의 설정으로의 대체를 허용합니다).

장치 입력(센서, 등), 내부 상태(CPU 부하, 고도, 등), `printf` 로그 메세지의 로깅에 활용할 수 있습니다.

모든 바이너리 형식에 리틀 엔디안 방식을 적용합니다.

## 데이터 형식

다음 바이너리 형식을 사용합니다. C의 자료형에 해당합니다:

| 형식                  | 바이트 크기 |
| ------------------- | ------ |
| int8_t, uint8_t   | 1      |
| int16_t, uint16_t | 2      |
| int32_t, uint32_t | 4      |
| int64_t, uint64_t | 8      |
| float               | 4      |
| double              | 8      |
| bool, char          | 1      |

게다가 다음과 같은 배열에도 활용할 수 있습니다. `float[5]`가 이에 해당합니다. 보통 모든 문자열(`char[length]`)의 마지막에 `'\0'`이 있는건 아닙니다. 문자열 비교시 대소문자를 구분합니다.

## 파일 구조

파일은 다음과 같이 세 부분으로 나뉩니다:

    ----------------------
    |       Header       |
    ----------------------
    |    Definitions     |
    ----------------------
    |        Data        |
    ----------------------
    

### 헤더 섹션

The header is a fixed-size section and has the following format (16 bytes):

    ----------------------------------------------------------------------
    | 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x01         | uint64_t       |
    | File magic (7B)                    | Version (1B) | Timestamp (8B) |
    ----------------------------------------------------------------------
    

Version is the file format version, currently 1. Timestamp is a `uint64_t` integer, denotes the start of the logging in microseconds.

### Definitions Section

Variable length section, contains version information, format definitions, and (initial) parameter values.

The Definitions and Data sections consist of a stream of messages. Each starts with this header:

```c
struct message_header_s {
  uint16_t msg_size;
  uint8_t msg_type
};
```

`msg_size` is the size of the message in bytes without the header (`hdr_size`= 3 bytes). `msg_type` defines the content and is one of the following:

- 'B': Flag bitset message.
  
      struct ulog_message_flag_bits_s {
        struct message_header_s;
        uint8_t compat_flags[8];
        uint8_t incompat_flags[8];
        uint64_t appended_offsets[3]; ///< file offset(s) for appended data if appending bit is set
      };
      
  
  This message **must** be the first message, right after the header section, so that it has a fixed constant offset.
  
  - `compat_flags`: compatible flag bits. None of them is currently defined and all must be set to 0. These bits can be used for future ULog changes that are compatible with existing parsers. It means parsers can just ignore the bits if one of the unknown bits is set.
  - `incompat_flags`: incompatible flag bits. The LSB bit of index 0 is set to one if the log contains appended data and at least one of the `appended_offsets` is non-zero. All other bits are undefined and must be set to 0. If a parser finds one of these bits set, it must refuse to parse the log. This can be used to introduce breaking changes that existing parsers cannot handle.
  - `appended_offsets`: File offsets (0-based) for appended data. If no data is appended, all offsets must be zero. This can be used to reliably append data for logs that may stop in the middle of a message.
    
    A process appending data should do:
    
    - set the relevant `incompat_flags` bit,
    - set the first `appended_offsets` that is 0 to the length of the log file,
    - then append any type of messages that are valid for the Data section.
  
  It is possible that there are more fields appended at the end of this message in future ULog specifications. This means a parser must not assume a fixed length of this message. If the message is longer than expected (currently 40 bytes), the exceeding bytes must just be ignored.

- 'F': format definition for a single (composite) type that can be logged or used in another definition as a nested type.
  
      struct message_format_s {
        struct message_header_s header;
        char format[header.msg_size];
      };
      
  
  `format`: plain-text string with the following format: `message_name:field0;field1;` There can be an arbitrary amount of fields (at least 1), separated by `;`. A field has the format: `type field_name` or `type[array_length] field_name` for arrays (only fixed size arrays are supported). `type` is one of the basic binary types or a `message_name` of another format definition (nested usage). A type can be used before it's defined. There can be arbitrary nesting but no circular dependencies.
  
  Some field names are special:
  
  - `timestamp`: every logged message (`message_add_logged_s`) must include a timestamp field (does not need to be the first field). Its type can be: `uint64_t` (currently the only one used), `uint32_t`, `uint16_t` or `uint8_t`. The unit is always microseconds, except for in `uint8_t` it's milliseconds. A log writer must make sure to log messages often enough to be able to detect wrap-arounds and a log reader must handle wrap-arounds (and take into account dropouts). The timestamp must always be monotonic increasing for a message series with the same `msg_id`.
  - Padding: field names that start with `_padding` should not be displayed and their data must be ignored by a reader. These fields can be inserted by a writer to ensure correct alignment.
    
    If the padding field is the last field, then this field will not be logged, to avoid writing unnecessary data. This means the `message_data_s.data` will be shorter by the size of the padding. However the padding is still needed when the message is used in a nested definition.

- 'I': information message.
  
  ```c
  struct message_info_s {
    struct message_header_s header;
    uint8_t key_len;
    char key[key_len];
    char value[header.msg_size-1-key_len]
  };
  ```
  
  `key` is a plain string, as in the format message (can also be a custom type), but consists of only a single field without ending `;`, eg. `float[3] myvalues`. `value` contains the data as described by `key`.
  
  Note that an information message with a certain key must occur at most once in the entire log. Parsers can store information messages as a dictionary.
  
  Predefined information messages are:

| key                                 | Description                                 | Example for value  |
| ----------------------------------- | ------------------------------------------- | ------------------ |
| char[value_len] sys_name          | Name of the system                          | "PX4"              |
| char[value_len] ver_hw            | Hardware version (board)                    | "PX4FMU_V4"        |
| char[value_len] ver_hw_subtype    | Board subversion (variation)                | "V2"               |
| char[value_len] ver_sw            | Software version (git tag)                  | "7f65e01"          |
| char[value_len] ver_sw_branch     | git branch                                  | "master"           |
| uint32_t ver_sw_release           | Software version (see below)                | 0x010401ff         |
| char[value_len] sys_os_name       | Operating System Name                       | "Linux"            |
| char[value_len] sys_os_ver        | OS version (git tag)                        | "9f82919"          |
| uint32_t ver_os_release           | OS version (see below)                      | 0x010401ff         |
| char[value_len] sys_toolchain     | Toolchain Name                              | "GNU GCC"          |
| char[value_len] sys_toolchain_ver | Toolchain Version                           | "6.2.1"            |
| char[value_len] sys_mcu           | Chip name and revision                      | "STM32F42x, rev A" |
| char[value_len] sys_uuid          | Unique identifier for vehicle (eg. MCU ID)  | "392a93e32fa3"...  |
| char[value_len] log_type          | Type of the log (full log if not specified) | "mission"          |
| char[value_len] replay              | File name of replayed log if in replay mode | "log001.ulg"       |
| int32_t time_ref_utc              | UTC Time offset in seconds                  | -3600              |

    The format of `ver_sw_release` and `ver_os_release` is: 0xAABBCCTT, where AA is major, BB is minor, CC is patch and TT is the type. 
    Type is defined as following: `>= 0`: development, `>= 64`: alpha version, `>= 128`: beta version, `>= 192`: RC version, `== 255`: release version.
    So for example 0x010402ff translates into the release version v1.4.2.
    
    This message can also be used in the Data section (this is however the preferred section).
    

- 'M': information message multi.
  
  ```c
  struct ulog_message_info_multiple_header_s {
    struct message_header_s header;
    uint8_t is_continued; ///< can be used for arrays
    uint8_t key_len;
    char key[key_len];
    char value[header.msg_size-2-key_len]
  };
  ```
  
  The same as the information message, except that there can be multiple messages with the same key (parsers store them as a list). The `is_continued` can be used for split-up messages: if set to 1, it is part of the previous message with the same key. Parsers can store all information multi messages as a 2D list, using the same order as the messages occur in the log.

- 'P': parameter message. Same format as `message_info_s`. If a parameter dynamically changes during runtime, this message can also be used in the Data section. The data type is restricted to: `int32_t`, `float`.

This section ends before the start of the first `message_add_logged_s` or `message_logging_s` message, whichever comes first.

### 데이터 섹션

다음 메시지가 이 섹션에 해당합니다:

- 'A': 이름을 부여한 가입 메시지이며, `message_data_s`에서 활용하는 ID 값이 들어있습니다. 이 섹션은 `message_data_s`에 해당하는 첫번째 부분 이전에 와야합니다.
  
  ```c
  struct message_add_logged_s {
    struct message_header_s header;
    uint8_t multi_id;
    uint16_t msg_id;
    char message_name[header.msg_size-3];
  };
  ```
  
  `multi_id`: 동일한 형식의 센서 두가지를 달고 있는 시스템의 경우를 예로 들어, 여러 인스턴스를 가질 수 있는 동일한 메세지 형식입니다. 기본값과 첫번째 인스턴스는 0이어야 합니다. `msg_id`: `message_data_s` 데이터에 일치하는 고유 ID입니다. 처음 활용 부분은 0으로 설정하고, 값을 늘려야 합니다. 동일한 `msg_id`를 각기 다른 가입 절차를 위해 두번 사용하면 안되며, 그 반대의 경우도 마찬가지입니다. `message_name`: 처리 과정에 가입할 메세지 이름입니다. `message_format_s`에 지정한 이름과 정확하게 일치해야합니다.

- 'R': 더이상 로그를 진행하지 않음을 알리는 탈외 메시지(현재 사용하지 않음)입니다.
  
  ```c
  struct message_remove_logged_s {
    struct message_header_s header;
    uint16_t msg_id;
  };
  ```

- 'D': 로그 데이터가 들어있습니다.
  
      struct message_data_s {
        struct message_header_s header;
        uint16_t msg_id;
        uint8_t data[header.msg_size-2];
      };
      
  
  `msg_id`: `message_add_logged_s` 메시지에서 정의한 그대로입니다. `data`에는 `message_format_s`에 지정한 대로 이진 형식의 로그 메시지가 들어있습니다. 필드 여백을 특별히 처리한 방법은 위를 참고하십시오.

- 'L': 로깅한 문자열 메시지입니다. 예: printf 출력.
  
      struct message_logging_s {
        struct message_header_s header;
        uint8_t log_level;
        uint64_t timestamp;
        char message[header.msg_size-9]
      };
      
  
  `timestamp`: 마이크로초 단위, `log_level`: 리눅스 커널과 동이:

| Name    | Level value | Meaning                          |
| ------- | ----------- | -------------------------------- |
| EMERG   | '0'         | System is unusable               |
| ALERT   | '1'         | Action must be taken immediately |
| CRIT    | '2'         | Critical conditions              |
| ERR     | '3'         | Error conditions                 |
| WARNING | '4'         | Warning conditions               |
| NOTICE  | '5'         | Normal but significant condition |
| INFO    | '6'         | Informational                    |
| DEBUG   | '7'         | Debug-level messages             |

- 'C': Tagged Logged string message
  
      struct message_logging_tagged_s {
        struct message_header_s header;
        uint8_t log_level;
        uint16_t tag;
        uint64_t timestamp;
        char message[header.msg_size-9]
      };
      
  
  `tag`: id representing source of logged message string. It could represent a process, thread or a class depending upon the system architecture. For example, a reference implementation for an onboard computer running multiple processes to control different payloads, external disks, serial devices etc can encode these process identifiers using a `uint16_t enum` into the tag attribute of `message_logging_tagged_s` struct as follows:
  
      enum class ulog_tag : uint16_t {
        unassigned,
        mavlink_handler,
        ppk_handler,
        camera_handler,
        ptp_handler,
        serial_handler,
        watchdog,
        io_service,
        cbuf,
        ulg
      };
      
  
  `timestamp`: in microseconds `log_level`: same as in the Linux kernel:

| Name    | Level value | Meaning                          |
| ------- | ----------- | -------------------------------- |
| EMERG   | '0'         | System is unusable               |
| ALERT   | '1'         | Action must be taken immediately |
| CRIT    | '2'         | Critical conditions              |
| ERR     | '3'         | Error conditions                 |
| WARNING | '4'         | Warning conditions               |
| NOTICE  | '5'         | Normal but significant condition |
| INFO    | '6'         | Informational                    |
| DEBUG   | '7'         | Debug-level messages             |

- 'S': synchronization message so that a reader can recover from a corrupt message by searching for the next sync message.
  
      struct message_sync_s {
        struct message_header_s header;
        uint8_t sync_magic[8];
      };
      
  
  `sync_magic`: [0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12]

- 'O': mark a dropout (lost logging messages) of a given duration in ms. Dropouts can occur e.g. if the device is not fast enough.
  
      struct message_dropout_s {
        struct message_header_s header;
        uint16_t duration;
      };
      

- 'I': information message. See above.

- 'M': information message multi. See above.

- 'P': parameter message. See above.

## 파서 요구 사항

온전히 동작하는 ULog 파서는 다음 요건을 만족해야합니다:

- 알 수 없는 메시지는 무시해야 함(그러나 경고를 출력할 수 있음).
- 이후/알 수 없는 파일 형식 버전도 해석(그러나 경고를 출력할 수 있음).
- 로그에 파서에서 처리할 수 없는 깨진 변경이 들어있음을 의미하는 알 수 없는 비호환성 비트(`ulog_message_flag_bits_s` 메세지의 `incompat_flags`) 설정 값이 들어있을 경우 로그 해석을 거절해야함.
- 로그가 메세지 중간에 갑자기 끝났을 경우에도 제대로 처리할 수 있어야 함. 끝나지 않은 메시지는 무시해야 함.
- 후위 첨가 데이터: 파서에서 데이터 섹션이 존재함을 가정할 수 있어야 함. 예: 정의 섹션 다음의 위치를 오프셋이 가리킴.
  
  후위 첨가 데이터는 정규 데이터 섹션의 일부로 다루어야 함.

## Known Implementations

- PX4 Firmware: C++ 
  - [logger module](https://github.com/PX4/Firmware/tree/master/src/modules/logger)
  - [replay module](https://github.com/PX4/Firmware/tree/master/src/modules/replay)
  - [hardfault_log module](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log): append hardfault crash data.
- [pyulog](https://github.com/PX4/pyulog): python, ULog parser library with CLI scripts.
- [FlightPlot](https://github.com/PX4/FlightPlot): Java, log plotter.
- [pyFlightAnalysis](https://github.com/Marxlp/pyFlightAnalysis): Python, log plotter and 3D visualization tool based on pyulog.
- [MAVLink](https://github.com/mavlink/mavlink): Messages for ULog streaming via MAVLink (note that appending data is not supported, at least not for cut off messages).
- [QGroundControl](https://github.com/mavlink/qgroundcontrol): C++, ULog streaming via MAVLink and minimal parsing for GeoTagging.
- [mavlink-router](https://github.com/01org/mavlink-router): C++, ULog streaming via MAVLink.
- [MAVGAnalysis](https://github.com/ecmnet/MAVGCL): Java, ULog streaming via MAVLink and parser for plotting and analysis.
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler): C++/Qt application to plot logs and time series. Supports ULog since version 2.1.3.
- [ulogreader](https://github.com/maxsun/ulogreader): Javascript, ULog reader and parser outputs log in JSON object format. 

## 파일 형식 버전 이력

### 버전 2 개정 사항

Addition of `ulog_message_info_multiple_header_s` and `ulog_message_flag_bits_s` messages and the ability to append data to a log. This is used to add crash data to an existing log. If data is appended to a log that is cut in the middle of a message, it cannot be parsed with version 1 parsers. Other than that forward and backward compatibility is given if parsers ignore unknown messages.