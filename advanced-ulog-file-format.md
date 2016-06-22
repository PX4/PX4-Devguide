# ULog File Format

ULog is the file format used for logging system data. The format is
self-describing, i.e. it contains the format and message types that are logged.

It can be used for logging device inputs (sensors, etc.), internal states (cpu
load, attitude, etc.) and printf log messages.

The format uses Little Endian for all binary types.

## Data types

The following binary types are used. They all correspond to the types in C:

| Type              | Size in Bytes |
| ----              | ------------- |
|int8_t,  uint8_t   |  1            |
|int16_t, uint16_t  |  2            |
|int32_t, uint32_t  |  4            |
|int64_t, uint64_t  |  8            |
|float              |  4            |
|double             |  8            |
|bool, char         |  1            |

Additionally all can be used as an array, eg. `float[5]`. In general all
strings (`char[length]`) do not contain a `'\0'` at the end. String comparisons
are case sensitive.


## File structure

The file consists of three sections:
```
----------------------
|       Header       |
----------------------
|    Definitions     |
----------------------
|        Data        |
----------------------
```

### Header Section
The header is a fixed-size section and has the following format (16 bytes):
```
----------------------------------------------------------------------
| 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x00         | uint64_t       |
| File magic (7B)                    | Version (1B) | Timestamp (8B) |
----------------------------------------------------------------------
```
Version is the file format version, currently 0. Timestamp is an
uint64_t integer, denotes the start of the logging in microseconds.


### Definitions Section
Variable length section, contains version information, format definitions, and
(initial) parameter values.

The Definitions and Data sections consist of a stream of messages. Each
starts with this header:
```
struct message_header_s {
	uint16_t msg_size;
	uint8_t msg_type
};
```
`msg_size` is the size of the message in bytes without the header
(`hdr_size`= 3 bytes). `msg_type` defines the content and is one of the
following:

- 'F': format definition for a single (composite) type that can be logged or
  used in another definition as a nested type.
```
struct message_format_s {
	struct message_header_s header;
	char format[header.msg_size-hdr_size];
};
```
  `format`: plain-text string with the following format: `message_name:field0;field1;`
  There can be an arbitrary amount of fields (at least 1), separated by `;`. A
  field has the format: `type field_name` or `type[array_length] field_name` for
  arrays (only fixed size arrays are supported). `type` is one of the basic
  binary types or a `message_name` of another format definition (nested usage).
  A type can be used before it's defined. There can be arbitrary nesting but no
  circular dependencies.

  Some field names are special:
  - `timestamp`: every logged message (`message_add_logged_s`) must include a
	timestamp field (does not need to be the first field). Its type can be:
	`uint64_t` (currently the only one used), `uint32_t`, `uint16_t` or
	`uint8_t`. The unit is always microseconds, except for `uint8_t` it's
	milliseconds. A log writer must make sure to log messages often enough to be
	able to detect wrap-arounds and a log reader must handle wrap-arounds (and
	take into account dropouts). The timestamp must always be monotonic
	increasing for a message serie with the same `msg_id`.
  - Padding: field names that start with `_padding` should not be displayed and
	their data must be ignored by a reader. These fields can be inserted by a
	writer to ensure correct alignment.

	If the padding field is the last field, then this field will not be logged,
	to avoid writing unnecessary data. This means the `message_data_s.data`
	will be shorter by the size of the padding. However the padding is still
	needed when the message is used in a nested definition.

- 'I': information message.
```
struct message_info_s {
	struct message_header_s header;
	uint8_t key_len;
	char key[key_len];
	char value[header.msg_size-hdr_size-1-key_len]
};
```
  `key` is a plain string, as in the format message, but consists of only a
  single field without ending `;`, eg. `float[3] myvalues`. `value` contains the
  data as described by `key`.

  Predefined information messages are:

| `key`                     | Description            | Example for value |
| -----                     | -----------            | -------------     |
| char[value_len] sys_name  | Name of the system     |  "PX4"            |
| char[value_len] ver_hw    | Hardware version       |  "PX4FMU_V4"      |
| char[value_len] ver_sw    | Software version       |  "7f65e01"        |
| char[value_len] replay    | File name of replayed log if in replay mode | "log001.ulg" |
| int32_t time_ref_utc      | UTC Time offset in seconds |  -3600        |


- 'P': parameter message. Same format as `message_info_s`.
  If a parameter dynamically changes during runtime, this message can also be
  used in the Data section.
  The data type is restricted to: `int32_t`, `float`.

This section ends before the start of the first `message_add_logged_s` or
`message_logging_s` message, whichever comes first.


### Data Section

The following messages belong to this section:
- 'A': subscribe a message by name and give it an id that is used in
  `message_data_s`. This must come before the first corresponding
  `message_data_s`.
```
struct message_add_logged_s {
	struct message_header_s header;
	uint8_t multi_id;
	uint16_t msg_id;
	char message_name[header.msg_size-hdr_size-3];
};
```
  `multi_id`: the same message format can have multiple instances, for example
  if the system has two sensors of the same type.
  The default and first instance must be 0.
  `msg_id`: unique id to match `message_data_s` data. The first use must set
  this to 0, then increase it. The same `msg_id` must not be used twice for
  different subscriptions, not even after unsubscribing.
  `message_name`: message name to subscribe to. Must match one of the
  `message_format_s` definitions.

- 'R': unsubscribe a message, to mark that it will not be logged anymore (not
  used currently).
```
struct message_remove_logged_s {
	struct message_header_s header;
	uint16_t msg_id;
};
```

- 'D': contains logged data.
```
struct message_data_s {
	struct message_header_s header;
	uint16_t msg_id;
	uint8_t data[header.msg_size-hdr_size];
};
```
  `msg_id`: as defined by a `message_add_logged_s` message. `data` contains the
  logged binary message as defined by `message_format_s`. See above for special
  treatment of padding fields.

- 'L': Logged string message, i.e. printf output.
```
struct message_logging_s {
	struct message_header_s header;
	uint8_t log_level;
	uint64_t timestamp;
	char message[header.msg_size-hdr_size-9]
};
```
  `timestamp`: in microseconds, `log_level`: same as in the Linux kernel:

| Name       | Level value  | Meaning                              |
| ----       | -----------  | -------                              |
| EMERG      |      '0'     | System is unusable                   |
| ALERT      |      '1'     | Action must be taken immediately     |
| CRIT       |      '2'     | Critical conditions                  |
| ERR        |      '3'     | Error conditions                     |
| WARNING    |      '4'     | Warning conditions                   |
| NOTICE     |      '5'     | Normal but significant condition     |
| INFO       |      '6'     | Informational                        |
| DEBUG      |      '7'     | Debug-level messages                 |

- 'S': synchronization message so that a reader can recover from a corrupt
  message by searching for the next sync message (not used currently).
```
struct message_sync_s {
	struct message_header_s header;
	uint8_t sync_magic[8];
};
```
`sync_magic`: to be defined.

- 'O': mark a dropout (lost logging messages) of a given duration in ms.
  Dropouts can occur e.g. if the device is not fast enough.
```
struct message_dropout_s {
	struct message_header_s header;
	uint16_t duration;
};
```



