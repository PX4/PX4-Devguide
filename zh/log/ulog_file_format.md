# ULog 文件格式

ULog is the file format used for logging system data.

The format is self-describing, i.e. it contains the format and message types that are logged (note that the [system logger](../log/logging.md) allows the *default set* of logged topics to be replaced from an SD card).

It can be used for logging device inputs (sensors, etc.), internal states (cpu load, attitude, etc.) and `printf` log messages.

The format uses Little Endian for all binary types.

## 数据类型

The following binary types are used. They all correspond to the types in C:

| 类型                  | 大小（以字节为单位） |
| ------------------- | ---------- |
| int8_t, uint8_t   | 1          |
| int16_t, uint16_t | 2          |
| int32_t, uint32_t | 4          |
| int64_t, uint64_t | 8          |
| float               | 4          |
| double              | 8          |
| bool, char          | 1          |

Additionally all can be used as an array, eg. `float[5]`. In general all strings (`char[length]`) do not contain a `'\0'` at the end. String comparisons are case sensitive.

## 文件结构

The file consists of three sections:

    ----------------------
    |         头         |
    ----------------------
    |        定义        |
    ----------------------
    |        数据        |
    ----------------------
    

### 头部分

The header is a fixed-size section and has the following format (16 bytes):

    ----------------------------------------------------------------------
    | 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x01         | uint64_t       |
    | File magic(7B)                     | Version (1B) |  Timestamp (8B) |
    ----------------------------------------------------------------------
    

Version is the file format version, currently 1. Timestamp is a `uint64_t` integer, denotes the start of the logging in microseconds.

### 定义部分

Variable length section, contains version information, format definitions, and (initial) parameter values.

The Definitions and Data sections consist of a stream of messages. Each starts with this header:

```c
struct message_header_s {
  uint16_t msg_size;
  uint8_t msg_type
};
```

`msg_size` is the size of the message in bytes without the header (`hdr_size`= 3 bytes). `msg_type` defines the content and is one of the following:

- 'B' ：标记 bitset 报文。
  
      struct ulog_message_flag_bits_s {
        struct message_header_s;
        uint8_t compat_flags[8];
        uint8_t incompat_flags[8];
        uint64_t appended_offsets[3]; ///< file offset(s) for appended data if appending bit is set
      };
      
  
  这条消息**必须**是头后面的第一条消息，这样才有固定的常数偏移量。
  
  - `compat_flags`: 兼容的标志位。 它们目前都没有定义，都必须设置为 0。 这些位可用于将来的 Ulog 更改，即与现有解析器兼容。 这意味着, 如果设置了一个未知位，解析器就可以忽略。
  - `incompat_flags`: 不兼容的标志位。 如果日志包含附加数据，并且至少有一个 `appended_offset` 是非零的，那么索引 0 的 LSB 位被设置为 1。 其他位都是未定义的，必须将设置为 0。 如果解析器发现这些位置 1，它必须拒绝解析日志。 这可用于引入现有解析器无法处理的重大更改。
  - `appended_offsets`: 附加数据的文件偏移量 (基于 0)。 如果没有附加数据，则所有偏移量必须为零。 这可以用于消息中途暂停的情况下可靠的添加数据。
    
    附加数据的过程应该做到：
    
    - 置位相关的 `incompat_flags` 位，
    - 设置 `append_offsets` 的第一个元素为日志文件相对于 0 的长度，
    - 然后为数据部分添加有效的任何类型的消息。
  
  这使得在将来的 Ulog 规范中在末尾添加更多的字段成为可能。 这意味着解析器必须不能假定此消息的长度是固定的。 如果消息比预期的长（当前为 40 字节），则必须忽略超过的字节。

- 'F': 可以在另一个定义中作为嵌套类型记录或使用的单个 (组合) 类型的格式定义。
  
      struct message_format_s {
        struct message_header_s header;
        char format[header.msg_size];
      };
      
  
  `format`: 具有以下格式的纯文本字符串：`message_named: field0;field1;` 可以有任意数量的字段 (至少1个) ，采用 `;` 分隔。 字段的格式为：`type field_name` 或者 `type[array_length] field_name` 数组（只支持固定大小的数组）。 `type` 是一种基本的二进制类型或者是 `message_name` 的其他类型定义（嵌套使用）。 一个类型可以在定义之前使用。 可以任意嵌套，但没有循环依赖。
  
  有些字段名是特殊的：
  
  - `timestamp`：每个消息报文 (`message_add_logged_s`) 必须包含时间戳字段 (不必是第一个字段)。 它的类型可以是：`uint64_t` (目前唯一使用的)，`uint32_t`, `uint16_t` 或者是 `uint8_t` 。 它的单位一直是微秒，除了 `uint8_t`，它的单位是毫秒。 日志写入器必须确保足够频繁的写入报文使其能够检测到绕回，并且日志的读取器必须能够处理绕回 (还要把丢帧考虑在内)。 对于具有相同 `msg_id` 报文的时间戳必须是单调递增的。
  - Padding：以 `_padding` 开始的字段名应该不被显示并且必须被读取器忽略。 写入器可以通过插入这个字段确保正确对齐。
    
    如果 Padding 字段是最后一个字段，则不会记录该字段，以避免写入不必要的数据。 这意味着 `message_data_s.data` 会因为填充大小而更短。 但是当报文在嵌套定义中使用时任然需要填充。

- 'I'：信息报文。
  
  ```c
  struct message_info_s {
    struct message_header_s header;
    uint8_t key_len;
    char key[key_len];
    char value[header.msg_size-1-key_len]
  };
  ```
  
  `key` 是纯字符串，就像报文格式 (也可以是第三方类型)，但只包含一个没有结束符 `;` 的字段。 `float[3] myvalues`. `value` 包含 `key` 所描述的字段
  
  需要注意的是包含特定 key 的报文信息在整个日志中最多只能出现一次。 解析器可以将报文信息存储为字典。
  
  预定义的信息报文有：

| 键                                   | 描述                                          | 示例值                |
| ----------------------------------- | ------------------------------------------- | ------------------ |
| char[value_len] sys_name          | 系统名称                                        | "PX4"              |
| char[value_len] ver_hw            | 硬件版本 (主板)                                   | "PX4FMU_V4"        |
| char[value_len] ver_hw_subtype    | 主办子版本 (变化的)                                 | "V2"               |
| char[value_len] ver_sw            | 软件版本 (git 标签)                               | "7f65e01"          |
| char[value_len] ver_sw_branch     | git branch                                  | "master"           |
| uint32_t ver_sw_release           | 软件版本 (见下文)                                  | 0x010401ff         |
| char[value_len] sys_os_name       | 操作系统名称                                      | "Linux"            |
| char[value_len] sys_os_ver        | 操作系统版本 (git 标签)                             | "9f82919"          |
| uint32_t ver_os_release           | 操作系统版本 (见下文)                                | 0x010401ff         |
| char[value_len] sys_toolchain     | 工具链名称                                       | "GNU GCC"          |
| char[value_len] sys_toolchain_ver | 工具链版本                                       | "6.2.1"            |
| char[value_len] sys_mcu           | 芯片名称和修订                                     | "STM32F42x, rev A" |
| char[value_len] sys_uuid          | 车辆的唯一标识符 (例如 MCU ID)                        | "392a93e32fa3"...  |
| char[value_len] log_type          | Type of the log (full log if not specified) | "mission"          |
| char[value_len] replay              | File name of replayed log if in replay mode | "log001.ulg"       |
| int32_t time_ref_utc              | UTC Time offset in seconds                  | -3600              |

    `ver_sw_release` 和 `ver_os_release` 的类型是：0xAABBCCTT，其中 AA 是主要的，BB 是次要的，CC 是补丁，TT 是类型。 
    类型定义如下：`>= 0`：development 版本，`>= 64`：alpha 版本，`>= 128`：beta 版本，`>= 192`：RC 版本，`== 255`：release 版本。
    So for example 0x010402ff translates into the release version v1.4.2.
    
    This message can also be used in the Data section (this is however the preferred section).
    

- 'M'：多报文信息。
  
  ```c
  struct ulog_message_info_multiple_header_s {
    struct message_header_s header;
    uint8_t is_continued; ///< can be used for arrays
    uint8_t key_len;
    char key[key_len];
    char value[header.msg_size-2-key_len]
  };
  ```
  
  与报文消息相同，不同的是可以有多个具有相同密钥的消息 (解析器将它们存储为列表) 。 `is_continued` 可以用于分割报文：如果置 1，则它是具有相同键的前一条报文的一部分。 解析器可以将所有多报文信息存储为一个 2D 列表，使用与日志中报文相同的顺序。

- 'P'：报文参数。 格式与 `message_info_s` 相同。 如果参数在运行时动态变化，则此报文也可用于 Data 部分。 数据类型限制为：`int32_t`，`float` 。

This section ends before the start of the first `message_add_logged_s` or `message_logging_s` message, whichever comes first.

### 数据部分

The following messages belong to this section:

- 'A'：按名称订阅消息，并给它一个在 `message_data_s` 中使用的 id。 这必须在第一个对应的 `message_data_s` 之前。
  
  ```c
  struct message_add_logged_s {
    struct message_header_s header;
    uint8_t multi_id;
    uint16_t msg_id;
    char message_name[header.msg_size-3];
  };
  ```
  
  `multi_id`：相同的消息格式可以有多个实例，例如系统有两个相同类型的传感器。 默认值以及第一个实例一定是0. `msg_id`：匹配 `message_data_s` 数据的惟一 id。 第一次使用一定要设置为 0，然后递增。 相同的 `msg_id` 不能用于两次不同的订阅，甚至在取消订阅后也不行。 `msg_name`：订阅的消息名称。 必须匹配其中一个 `message_format_s` 的定义。

- 'R'：取消订阅一条消息，以标记它将不再被记录 (当前未使用)。
  
  ```c
  struct message_remove_logged_s {
    struct message_header_s header;
    uint16_t msg_id;
  };
  ```

- 'D'：包含日志数据。
  
      struct message_data_s {
        struct message_header_s header;
        uint16_t msg_id;
        uint8_t data[header.msg_size-2];
      };
      
  
  `msg_id`：由 `message_add_logged_s` 报文定义。 `data` 包含由 `message_format_s` 定义的二进制日志消息。 有关填充字段的特殊处理，请参见上文。

- 'L'：字符串日志报文，比如打印输出。
  
      struct message_logging_s {
        struct message_header_s header;
        uint8_t log_level;
        uint64_t timestamp;
        char message[header.msg_size-9]
      };
      
  
  `timestamp`: 以微秒为单位，`log_level`: 和 Linux 内核一样。

| 名称      | 对应值 | 含义       |
| ------- | --- | -------- |
| EMERG   | '0' | 系统无法使用   |
| ALERT   | '1' | 操作必须立即执行 |
| CRIT    | '2' | 紧急情况     |
| ERR     | '3' | 错误情况     |
| WARNING | '4' | 警告情况     |
| NOTICE  | '5' | 正常但重要的情况 |
| INFO    | '6' | 信息       |
| DEBUG   | '7' | 调试级别的消息  |

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

## 解析器的要求

A valid ULog parser must fulfill the following requirements:

- Must ignore unknown messages (but it can print a warning).
- Parse future/unknown file format versions as well (but it can print a warning).
- Must refuse to parse a log which contains unknown incompatibility bits set (`incompat_flags` of `ulog_message_flag_bits_s` message), meaning the log contains breaking changes that the parser cannot handle.
- A parser must be able to correctly handle logs that end abruptly, in the middle of a message. The unfinished message should just be discarded.
- For appended data: a parser can assume the Data section exists, i.e. the offset points to a place after the Definitions section.
  
  Appended data must be treated as if it was part of the regular Data section.

## 已知的实现

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

## 文件格式版本历史

### 版本 2 中的改变

Addition of `ulog_message_info_multiple_header_s` and `ulog_message_flag_bits_s` messages and the ability to append data to a log. This is used to add crash data to an existing log. If data is appended to a log that is cut in the middle of a message, it cannot be parsed with version 1 parsers. Other than that forward and backward compatibility is given if parsers ignore unknown messages.