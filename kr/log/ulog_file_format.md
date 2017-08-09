# ULog 파일 포맷

ULog는 logging 시스템 데이터에 사용하는 파일 포맷입니다. 이 포맷은 자기기술(self-describing)형태로 예를 들자면 log 포맷과 메시지 타입을 포함하고 있습니다.

logging 입력 장치(센서), 내부 상태와 log 메시지 출력하는데 사용할 수 있습니다.

모든 binary 타입에서 Little Endian을 사용합니다.

## Data 타입

다음과 같은 binary 타입을 사용합니다. 이 타입들은 C언어의 타입에 대응됩니다. :

| Type              | Size in Bytes |
| ----              | ------------- |
|int8_t,  uint8_t   |  1            |
|int16_t, uint16_t  |  2            |
|int32_t, uint32_t  |  4            |
|int64_t, uint64_t  |  8            |
|float              |  4            |
|double             |  8            |
|bool, char         |  1            |

추가로 모두 array(예로 `float[5]`)로 사용할 수 있습니다. 일반적으로 모든 string(`char[length]`)에 끝을 나타내는 `'\0'`을 포함하지 않습니다. string 비교에서는 대소문자를 구별합니다.


## 파일 구조

파일은 3개 섹션으로 구성:
```
----------------------
|       Header       |
----------------------
|    Definitions     |
----------------------
|        Data        |
----------------------
```

### 헤더 섹션
헤더는 고정길이를 갖는 섹션이며 다음과 같은 포맷을 가집니다. (16 bytes) :
```
----------------------------------------------------------------------
| 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x00         | uint64_t       |
| File magic (7B)                    | Version (1B) | Timestamp (8B) |
----------------------------------------------------------------------
```
Version은 파일 포맷 버전이며 현재는 0입니다. Timestamp은 uint64_t integer로 microseconds 단위로 logging의 시작을 의미합니다.


### Definitions Section
가변길이 섹션으로 version 정보, format 정의와 (초기) 파라미터 값을 포함합니다.

Definitions과 Data 섹션은 일종의 메시지의 stream으로 구성됩니다. 각각은 다음과 같은 헤더로 시작합니다. :
```
struct message_header_s {
	uint16_t msg_size;
	uint8_t msg_type
};
```
`msg_size`는 header(`hdr_size`= 3 bytes)가 없는 byte 단위의 메시지의 크기입니다. `msg_type`는 content를 정의하며 다음 중에 하나를 따릅니다. :

- 'F': 단일(복합) 타입에 대한 포맷 정의로 로깅하거나 nested 타입으로 다른 definition에서 사용.

```
struct message_format_s {
	struct message_header_s header;
	char format[header.msg_size-hdr_size];
};
```
  `format`: 다음 포맷을 가지는 일반 텍스트 문자열: `message_name:field0;field1;`
	구분자로 `;`를 사용하며 최소 1개 이상의 임의 필드를 가질 수 있음.
	필드는 다음 포맷을 가짐: `type field_name` 혹은 array(고정 길이 array만 지원)에 대한 `type[array_length] field_name`. `type`은 기본 binary 타입 중에 하나 혹은 다른 포맷 정의(nest로 사용)의 `message_name`.
	type은 정의하기 전에 사용할 수도 있음. 임의의 nesting일 수는 있지만 순환 종속성은 없어야 함.

	일부 필드 이름은 특별하게 :
	- `timestamp`: 모든 logged message(`message_add_logged_s`)는 반드시 timestamp field을 포함해야 함(첫번째 필드일 필요는 없음). 타입으로 `uint64_t` (현재 유일하게 사용), `uint32_t`, `uint16_t` 혹은
	`uint8_t`. `uint8_t` 인 경우만 milliseconds고 나머지는 항상 microseconds임. log writer는 wrap-arounds를 검출할 수 있게 메시지 로그를 해야하며 log reader는 반드시 wrap-arounds를 처리해야만 함.(드롭아웃에 대해서도 고려해야함) timestamp는 동일 `msg_id`를 가지는 일련의 메시지에 대해서 항상 증가.
	- Padding: 필드 이름은 `_padding`로 시작함. 표시되는 부분이 아니며 reader쪽에서 이 값은 무시됨. writerrk 이 필드를 채워줘서 올바른 형태로 만들어줌.

	padding 필드가 마지막 필드인 경우라면 불필요한 데이터를 쓰는 것을 피하기 위해서 이 필드는 기록하지 않는다. 즉 padding의 크기만큼 `message_data_s.data`는 줄어들게 됨. message를 nested definition에 사용할때도 padding은 여전히 필요함.

- 'I': information message.
```
struct message_info_s {
	struct message_header_s header;
	uint8_t key_len;
	char key[key_len];
	char value[header.msg_size-hdr_size-1-key_len]
};
```
  `key`는 일반 문자열이며 format message내에서 `;`로 마무리하지 않고 단일 필드로만 구성됨. 예제로  `float[3] myvalues`. `value`는 `key`에 설정한 데이터를 포함함.

  미리 정의된 information messages:

| `key`                        | Description               | Example for value |
| -----                        | -----------               | ----------------- |
| char[value_len] sys_name     | Name of the system        |  "PX4"            |
| char[value_len] ver_hw       | Hardware version          |  "PX4FMU_V4"      |
| char[value_len] ver_sw       | Software version (git tag)|  "7f65e01"        |
| uint32_t ver_sw_release      | Software version (see below)|  0x010401ff     |
| char[value_len] sys_os_name  | Operating System Name     |  "Linux"          |
| char[value_len] sys_os_ver   | OS version (git tag)      |  "9f82919"        |
| uint32_t ver_os_release      | OS version (see below)    |  0x010401ff       |
| char[value_len] sys_toolchain| Toolchain Name            |  "GNU GCC"        |
| char[value_len] sys_toolchain_ver| Toolchain Version     |  "6.2.1"          |
| char[value_len] sys_mcu      | Chip name and revision    |  "STM32F42x, rev A"|
| char[value_len] sys_uuid     | Unique identifier for vehicle (eg. MCU ID) |  "392a93e32fa3"...|
| char[value_len] replay       | File name of replayed log if in replay mode | "log001.ulg" |
| int32_t time_ref_utc         | UTC Time offset in seconds |  -3600        |

  `ver_sw_release`와 `ver_os_release`의 포맷은 : 0xAABBCCTT. AA는 major, BB는 minor, CC는 path, TT는 타입. type은 다음과 같이 정의 : `>= 0`: development, `>= 64`: alpha version, `>= 128`: beta version, `>= 192`: RC version, `== 255`: release version.
	따라서 0x010402ff는 release version v1.4.2으로 해석할 수 있음.

	이 메시지는 Data 섹션에서 사용할 수 있습니다.(하지만 이것이 선호되는 섹션)


- 'P': 파리미터 메시지. `message_info_s`와 동일한 포맷.
  만약 실행하는 동안 파라미터가 동적으로 변경되면, 이 메시지는 Data 섹션에서 사용할 수 있음.
	data type은 `int32_t`, `float`만 사용가능.

`message_add_logged_s`나 `message_logging_s` 메시지가 시작하기 전에 이 섹션은 종료된다. 둘 중에 어떤 것이든 먼저 올 수 있음.


### Data Section

다음과 같은 메시지가 이 섹션에 속합니다. :
- 'A': name으로 메시지를 subscribe하고 `message_data_s`내 사용할 id를 부여함. 이 과정은 반드시 첫번째 관련 `message_data_s` 전에 해야함.

```
struct message_add_logged_s {
	struct message_header_s header;
	uint8_t multi_id;
	uint16_t msg_id;
	char message_name[header.msg_size-hdr_size-3];
};
```
  `multi_id`: 동일한 메시지 포맷은 다양한 인스턴스를 가질 수 있음. 예로 만약 시스템이 동일한 타입의 2개 센서를 가지고 있다면 기본적으로 첫번째 인스턴스는 0이 되어야 함.
  `msg_id`: `message_data_s` data를 매칭하는데 고유 id 사용. 처음 사용에서 반드시 0으로 설정하고 증가시킴. 동일 `msg_id`는 중복해서 사용해서는 안되며 unsubscribe 하더라도 중복사용은 안됨.
	`message_name` : subscribe할 메시지 이름. `message_format_s` definition들 중에 하나와 매칭되어야 함.
```
struct message_remove_logged_s {
	struct message_header_s header;
	uint16_t msg_id;
};
```

- 'D': logged data 포함
```
struct message_data_s {
	struct message_header_s header;
	uint16_t msg_id;
	uint8_t data[header.msg_size-hdr_size];
};
```
  `msg_id`: `message_add_logged_s` 메시지에서 정의. `message_format_s` 정의에 따라 `data`는 logged binary message를 포함. 위에 padding 필드의 처리에 관련 부분을 참고.

- 'L': Logged string message. 예로 printf 출력.
```
struct message_logging_s {
	struct message_header_s header;
	uint8_t log_level;
	uint64_t timestamp;
	char message[header.msg_size-hdr_size-9]
};
```
  `timestamp`: microseconds 단위, `log_level`: Linux 커널에서와 동일 :

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

- 'S': 동기화 메시지로 reader가 오류가 있는 메시지를 복구하기 위해 다음 동기화 메시지를 이용함. (현재 사용 안됨)
```
struct message_sync_s {
	struct message_header_s header;
	uint8_t sync_magic[8];
};
```
`sync_magic`: 정의할 예정

- 'O': 주어진 ms 시간 구간의 드롭아웃(로그 메시지 손실)을 표시함.
  드롭아웃은 장치의 처리 속도가 느린 경우에 발생할 수 있음.
```
struct message_dropout_s {
	struct message_header_s header;
	uint16_t duration;
};
```

- 'I': information message. 위를 참고.

- 'M': 다중 information message. 위를 참조.

- 'P': parameter message. 위를 참고.


## 파서(Parser) 요구사항
유효한 ULog 파서는 다음과 같은 요구사항을 만족해야만 합니다:
- 모르는 메시지는 반드시 무시 (warning을 출력할 수 있음)
- future/unknown 파일 포맷 버전 파싱 (warning을 출력할 수 있음)
- unknown 비호환 bit set을 포함하는 경우 로그 파싱이 반드시 거부해야함(`ulog_message_flag_bits_s` 메시지의 `incompat_flags`) 로그에 파서가 처리할 수 없는 변경이 포함되어 있다는 의미.
- 파서는 메시지 중간에 갑자기 종료되는 로그를 제대로 처리할 수 있어야 함. 이렇게 종료되지 않는 메시지는 폐기됨.
- 추가된 데이터 : 파서는 Data 섹션에 있다고 가정함. 오프셋은 Definitions 섹션 이후 위치를 가리킴.

  추가된 데이터는 일반 Data 섹션의 일부였던 것처럼 취급되어야만 합니다.


## 공개된 구현
- PX4 Firmware: C++
  - [logger module](https://github.com/PX4/Firmware/tree/master/src/modules/logger)
  - [replay module](https://github.com/PX4/Firmware/tree/master/src/modules/replay)
  - [hardfault_log module](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log):
	하드폴트 크러쉬 데이터 추가.
- [pyulog](https://github.com/PX4/pyulog): python, CLI 스크립트로 작성된 ULog parser library
- [FlightPlot](https://github.com/PX4/FlightPlot): Java, log plotter.
- [MAVLink](https://github.com/mavlink/mavlink): MAVLink를 통해 ULog 스트리밍에 관한 메시지(추가된 데이터는 지원하지 않으며 끊어진 메시지는 지원하지 않음.
- [QGroundControl](https://github.com/mavlink/qgroundcontrol): C++, MAVLink를 통한 ULog
  streaming과 GeoTagging에 대한 최소 파싱.
- [mavlink-router](https://github.com/01org/mavlink-router): C++, MAVLink를 통한 ULog streaming.
- [MAVGAnalysis](https://github.com/ecmnet/MAVGCL): Java, MAVLink를 통한 ULog streaming과 plotting과 분석을 위한 파서.


## 파일 포맷 버전 History
### 버전 2에서 변경사항
`ulog_message_info_multiple_header_s`와 `ulog_message_flag_bits_s` 메시지 추가 및 로그에 데이터 추가 가능한 기능. 이렇게 하면 기존 로그에 깨진 데이터가 추가될 수 있다. 메시지 중간이 잘린 상태로 데이터가 로그에 추가되는 것은 기본 버전 1 파서에서는 불가능했음. 파서가 unknown 메시지를 무시하게 되면 이전 버전에 대해서 호환이 가능하다.
