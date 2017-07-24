---
translated_page: https://github.com/PX4/Devguide/blob/master/en/log/ulog_file_format.md
translated_sha: cd45f94f39aae63536cde77a0a8b959392d23d9e
---

# ULog File Format

Ulog是一种用来记录系统数据的日志格式。这种格式是自解释的，比如，他包含了日志的格式和消息类型。

他可以用来记录设备的输入（传感器等），内部状态（CPU负载，姿态等）以及打印日志信息。

采用小端格式。（译注：低字节存储在低地址）

## 数据类型

下面列举了使用的数据类型，他们都与C语言的类型相对应。

| Type                | Size in Bytes |
| ------------------- | ------------- |
| int8\_t,  uint8\_t  | 1             |
| int16\_t, uint16\_t | 2             |
| int32\_t, uint32\_t | 4             |
| int64\_t, uint64\_t | 8             |
| float               | 4             |
| double              | 8             |
| bool, char          | 1             |

此外所有类型都可以使用数组，比如`float[5]`。一般而言所有的字符串\(`char[length]`\)结尾都不包含 `'\0'`。字符串大小写敏感。

## 文件结构

文件包含三个部分:

```
----------------------
|       Header       |
----------------------
|    Definitions     |
----------------------
|        Data        |
----------------------
```

### 头部

头部大小固定，格式如下\(16 bytes\):


```
----------------------------------------------------------------------
| 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x00         | uint64_t       |
| File magic (7B)                    | Version (1B) | Timestamp (8B) |
----------------------------------------------------------------------
```


Version是文件格式的版本，当前是0。时间戳是uint64\_t类型，用微秒表示记录开始的时间。

### 定义部分（Definitions Section）

长度可变，包含版本信息，格式定义以及\(初始\) 参数值。

定义部分和数据部分由消息流组成，消息流以下面这样的头部开始：

```
struct message_header_s {
    uint16_t msg_size;
    uint8_t msg_type
};
```

`msg_size` 消息去掉头部的字节数
\(`hdr_size`= 3 bytes\). `msg_type`定义了内容，是下面可能的情况之一:

* 'F': format definition for a single \(composite\) type that can be logged or
  used in another definition as a nested type.

* 'F': 单一（混合）类型的格式定义，用于日志记录或者作为嵌套类型用在其他的定义中。

```
struct message_format_s {
    struct message_header_s header;
    char format[header.msg_size-hdr_size];
};
```

  `format`: 纯文本字符串，格式如下: `message_name:field0;field1;`可以有任意数量的field
   \(至少 1\), 用 `;`隔开。
   field 的格式: `type field_name` 或者数组形式 `type[array_length] field_name`\(只支持固定尺寸的数组\).
   `type` 可以是基本的数据类型，也可以是另一种格式定义的`message_name` \(嵌套用法\).  
   type可以在定义前使用。可以任意地嵌套，但是不要循环依赖。

有一些特殊的field:

* `timestamp`: 每个日志消息 \(`message_add_logged_s`\) 必须包含一个  
    timestamp field \(不必是第一个\). 他的type可以是:  
        `uint64_t` \(当前唯一被用到的\), `uint32_t`, `uint16_t` or  
        `uint8_t`. 除了 `uint8_t` 的单位是毫秒，其他单位都是微秒 。
        日志写入器必须确保记录日志消息足够频繁，能够检测环绕，一个日志读取器必须处理环绕
        \（并且考虑到数据丢失\）. 拥有相同`msg_id`的消息序列的timestamp必须单调增加.

* Padding: 以`_padding` 开头的field名称，不应该被显示，并且读取器应该忽略他们的数据should not be displayed and  
    their data must be ignored by a reader. 写入器插入这些 fields 用来确保正确的对齐。

  如果 padding field 是最后一个field, 那么这个field不会被记录,这样就避免了写入不必要的数据
    这使`message_data_s.data` 得以缩短 。然而当消息用于嵌套定义的时候依然需要padding

* 'I': information message.

```
struct message_info_s {
    struct message_header_s header;
    uint8_t key_len;
    char key[key_len];
    char value[header.msg_size-hdr_size-1-key_len]
};
```

`key` 是一个纯文本字符串, 只包含一个field，没有`;`结尾，例如  
 `float[3] myvalues`. `value` 含有用`key`描述的数据。  

预定义的 information messages :

| `key`                             | Description                              | Example for value  |
| --------------------------------- | ---------------------------------------- | ------------------ |
| char[value_len] sys_name          | Name of the system                       | "PX4"              |
| char[value_len] ver_hw            | Hardware version                         | "PX4FMU_V4"        |
| char[value_len] ver_sw            | Software version (git tag)               | "7f65e01"          |
| uint32_t ver_sw_release           | Software version (see below)             | 0x010401ff         |
| char[value_len] sys_os_name       | Operating System Name                    | "Linux"            |
| char[value_len] sys_os_ver        | OS version (git tag)                     | "9f82919"          |
| uint32_t ver_os_release           | OS version (see below)                   | 0x010401ff         |
| char[value_len] sys_toolchain     | Toolchain Name                           | "GNU GCC"          |
| char[value_len] sys_toolchain_ver | Toolchain Version                        | "6.2.1"            |
| char[value_len] sys_mcu           | Chip name and revision                   | "STM32F42x, rev A" |
| char[value_len] sys_uuid          | Unique identifier for vehicle (eg. MCU ID) | "392a93e32fa3"...  |
| char[value_len] replay            | File name of replayed log if in replay mode | "log001.ulg"       |
| int32_t time_ref_utc              | UTC Time offset in seconds               | -3600              |

 `ver_sw_release`和`ver_os_release`的格式是: 0xAABBCCTT,  AA
  是 major（主版本号）, BB 是 minor（次版本号）, CC 是 patch（补丁版本） and TT 是类型. 类型
  定义如下: `>= 0`: development, `>= 64`: alpha version, `>= 128`: beta
  version, `>= 192`: RC version, `== 255`: release version.
  例如 0x010402ff 转换成版本为 v1.4.2.

This message can also be used in the Data section (this is however the preferred section).

* 'P': 参数消息. 和`message_info_s`格式一样.
    如果一个参数在运行时实时改变, 那这个消息也可以用在数据部分\(Data section\).
        数据类型限制为: `int32_t`, `float`.

This section ends before the start of the first `message_add_logged_s` or `message_logging_s` message, whichever comes first.



### 数据部分（Data Section）

下列消息属于这一部分:

* 'A': 订阅一个message，并且赋予它一个用于`message_data_s`的id.
  This must come before the first corresponding
  `message_data_s`.

```
struct message_add_logged_s {
    struct message_header_s header;
    uint8_t multi_id;
    uint16_t msg_id;
    char message_name[header.msg_size-hdr_size-3];
};
```

`multi_id`: 相同的消息格式可以通过`multi_id`赋予多个实例。默认的第一个实例为0。
`msg_id`: 唯一的 id 用来匹配 `message_data_s` 数据.第一次用必须置0，然后增加\(The first use must set 
this to 0, then increase it.\) 不同的订阅必须使用不同的id,甚至在取消订阅之后也不能使用相同的id
`message_name`: 要订阅的消息名称. 必须与`message_format_s` 中的一个定义相匹配.

* 'R': 取消订阅一个message,标记这个消息不再被记录 \(当前没有使用\).

```
struct message_remove_logged_s {
    struct message_header_s header;
    uint16_t msg_id;
};
```

* 'D': 包含记录的数据.

```
struct message_data_s {
    struct message_header_s header;
    uint16_t msg_id;
    uint8_t data[header.msg_size-hdr_size];
};
```

`msg_id`: 被`message_add_logged_s`定义的 message. `data` 包含被 `message_format_s`定义的
 二进制消息. 关于padding特殊的处理机制查看上面.

* 'L': 记录的字符串消息, i.e. printf output.

```
struct message_logging_s {
    struct message_header_s header;
    uint8_t log_level;
    uint64_t timestamp;
    char message[header.msg_size-hdr_size-9]
};
```
  `timestamp`:微秒为单位, `log_level`: 与 Linux kernel 一样:

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


* synchronization message so that a reader can recover from a corrupt
  message by search for the next sync message \(not used currently\).
    'S': 同步消息，消息阅读器通过搜索下一个同步消息的方式从一个损坏的消息恢复。\(当前未使用\)
```
struct message_sync_s {
    struct message_header_s header;
    uint8_t sync_magic[8];
};
```

`sync_magic`: 待定义\(to be defined\).

* 'O': 标记一个在以ms给定的时间段内的数据丢失 \(丢失日志消息\)。
    比如设备不够快的时候就会发生消息丢失.

```
struct message_dropout_s {
    struct message_header_s header;
    uint16_t duration;
};
```

* 'I': information message. See above.


* 'P': parameter message. See above.


## Requirements for Parsers
A valid ULog parser must fulfill the following requirements:
- Must ignore unknown messages (but it can print a warning).
- Parse future/unknown file format versions as well (but it can print a warning).
- Must refuse to parse a log which contains unknown incompatibility bits set
  (`incompat_flags` of `ulog_message_flag_bits_s` message), meaning the log
  contains breaking changes that the parser cannot handle.
- A parser must be able to correctly handle logs that end abruptly, in the
  middle of a message. The unfinished message should just be discarged.
- For appended data: a parser can assume the Data section exists, i.e. the
  offset points to a place after the Definitions section.

  Appended data must be treated as if it was part of the regular Data section.


## Known Implementations
- PX4 Firmware: C++
  - [logger module](https://github.com/PX4/Firmware/tree/master/src/modules/logger)
  - [replay module](https://github.com/PX4/Firmware/tree/master/src/modules/replay)
  - [hardfault_log module](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log):
    append hardfault crash data.
- [pyulog](https://github.com/PX4/pyulog): python, ULog parser library with CLI
  scripts.
- [FlightPlot](https://github.com/PX4/FlightPlot): Java, log plotter.
- [MAVLink](https://github.com/mavlink/mavlink): Messages for ULog streaming via
  MAVLink (note that appending data is not supported, at least not for cut off
  messages).
- [QGroundControl](https://github.com/mavlink/qgroundcontrol): C++, ULog
  streaming via MAVLink and minimal parsing for GeoTagging.
- [mavlink-router](https://github.com/01org/mavlink-router): C++, ULog streaming
  via MAVLink.
- [MAVGAnalysis](https://github.com/ecmnet/MAVGCL): Java, ULog streaming via
  MAVLink and parser for plotting and analysis.


## File Format Version History
### Changes in version 2
Addition of `ulog_message_info_multiple_header_s` and `ulog_message_flag_bits_s`
messages and the ability to append data to a log. This is used to add crash data
to an existing log. If data is appended to a log that is cut in the middle of a
message, it cannot be parsed with version 1 parsers. Other than that forward and
backward compatibility is given if parsers ignore unknown messages.

