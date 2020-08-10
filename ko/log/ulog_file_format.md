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

헤더는 고정 길이(16 바이트) 섹션이며, 다음 형식을 갖추고 있습니다:

    ----------------------------------------------------------------------
    | 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x01         | uint64_t       |
    | File magic (7B)                    | Version (1B) | Timestamp (8B) |
    ----------------------------------------------------------------------
    

버전은 파일 형식 버전이며, 현재 값은 1입니다. 타임스탬프는 `uint64_t` 정수형이며, 로깅을 시작한 시각을 마이크로초 단위로 표기합니다.

### 섹션 정의

다양한 길이를 가진 섹션에는 버전 정보, 형식 정의, (초기) 매개변수 값이 들어있습니다.

데이터 섹션 정의는 메시지 스트림으로 구성합니다. 각 데이터 섹션은 이 헤더로 시작합니다:

```c
struct message_header_s {
  uint16_t msg_size;
  uint8_t msg_type
};
```

`msg_size`는 헤더 길이(`hdr_size`=3 바이트)를 뺀 바이트 단위의 메시지 길이입니다. `msg_type`은 내용과 다음 중 하나를 정의합니다:

- 'B': 플래그 비트 집합 메시지입니다.
  
      struct ulog_message_flag_bits_s {
        struct message_header_s;
        uint8_t compat_flags[8];
        uint8_t incompat_flags[8];
        uint64_t appended_offsets[3]; ///< file offset(s) for appended data if appending bit is set
      };
      
  
  이 메시지는 처음 메시지 **여야 합니다**. 그 다음에는 고정 상수 오프셋 값이 들어간 헤더 섹션이 옵니다.
  
  - `compat_flags`: 호환 플래그 비트값. 아직 정의하는 사항은 없으며 모든 경우에 0으로 설정해야합니다. 이 비트 값은 나중에 ULog 형식이 바뀌고 기존 파서와의 호환성을 비교할 때 활용합니다. 이는 알 수 없는 비트값을 설정했다면, 파서에서 이 비트값을 무시할 수 있음을 의미합니다.
  - `incompat_flags`: 비호환성 플래그 비트값. 로그에 후위 첨가 데이터가 있고 최소한 `appended_offsets` 값 중 하나가 0 값이 아닌 경우, 인덱스 0번의 최하위 비트 값을 1로 설정해야합니다. 다른 비트 값은 별도로 정의한 내용이 없어 0으로 설정해야합니다. 파서에서 이 비트 중 최소한 하나라도 설정 값을 발견했다면 로그 해석을 거절해야합니다. 기존 파서에서 처리할 수 없는 변경 손상 발견에 활용할 수 있습니다.
  - `appended_offsets`: 후위 첨가 데이터의 (0 부터 시작하는) 파일 오프셋 값입니다. 후위 첨가 데이터가 없을 경우, 모든 오프셋 값은 0이어야합니다. 이 값은 메시지 기록 중 중간에서 멈추더라도 로그의 데이터를 뒤에 확실하게 붙일 때 활용할 수 있습니다.
    
    데이터 후위 첨부 절차는 다음과 같이 진행합니다:
    
    - 관련 `incompat_flags` 비트 값을 설정합니다.
    - `append_offsets` 처음 값을 로그 파일 길이 값인 0으로 설정합니다.
    - 그 다음 데이터 섹션에 유효한 어떤 메시지든 뒤에 붙입니다.
  
  향후 ULog 명세에서는 이 메시지의 끝에 추가 필드가 붙을 수 있습니다. 그러니까, 이 메시지의 길이가 고정임을 간주해서는 안됩니다. 예상 길이보다 메시지의 길이가 길어지면(현재 40바이트), 초과 길이는 무시해야합니다.

- 'F': 중첩 형식으로 다른 정의에 활용할 수 있는 단일(요소)형식 정의 구성(포맷)
  
      struct message_format_s {
        struct message_header_s header;
        char format[header.msg_size];
      };
      
  
  `format`: `message_name:field0;field1;` 형식의 일반 텍스트 문자열입니다. `;` 문자로 나눈 임의 갯수의 필드(최소 1개)를 넣을 수 있습니다. 필드는 `type field_name` 형식 또는 `type[array_length] field_name` (고정 길이만 지원) 배열 형식을 지닙니다. `type` 은 기본 바이너리 형식 또는 기타 형식 정의의 `message_name`중 하나입니다(중첩 활용). 형식 자체를 정의하기 전에 먼저 활용할 수 있습니다. 임의대로 중첩할 수 있으나 순환 의존성을 지니면 안됩니다.
  
  일부 특별한 필드 이름은 다음과 같습니다:
  
  - `timestamp`: 매번 기록하는 메시지(`message_add_logged_s`)에는 타임스탬프 필드가 들어있어야합니다(처음 필드에는 필요하지 않음). 타임스탬프의 자료형은 `uint64_t` (현재 유일하게 사용하는 형식), `uint32_t`, `uint16_t`, `uint8_t` 중 하나로 설정할 수 있습니다. 단위는 마이크로초이며, `uint8_t` 형일 경우 밀리초입니다. 로그 기록 프로그램은 반드시 로그의 시작과 끝을 확인할 수 있도록 로그를 작성하는지 확인해야 하며, 로그 읽기 프로그램은 양 끝단을 처리해야 합니다(그리고 상황에 따라 폐기를 고려합니다). 타임스탬프 값은 동일한 `msg_id`를 지닌 메세지 모음에 대해 단조롭게 계속 늘어나야합니다.
  - Padding: `_padding`으로 시작하는 필드 이름을 가진 부분은 화면에 뿌리면 안되며, 리더에서 무시해야합니다. 이 필드는 데이터의 정렬을 제대로 하기 위해 로그 기록 프로그램에서 넣을 수 있습니다.
    
    패딩 필드가 마지막 필드라면, 이 필드는 필요없는 데이터 기록을 피하려 기록에 넣지 않습니다. 이는 `message_data_s.data` 데이터가 패딩 길이를 줄이는 만큼 더 짧아질 수 있음을 의미합니다. 그러나 패딩은 중첩 정의에서 메세지를 활용할 경우 여전히 필요합니다. 

- 'I': 정보 메세지.
  
  ```c
  struct message_info_s {
    struct message_header_s header;
    uint8_t key_len;
    char key[key_len];
    char value[header.msg_size-1-key_len]
  };
  ```
  
  `key`는 형식 메세지에서처럼 일반 문자열이지만(개별 설정 형식일 수 있음), `;` 문자로 끝나지 않는 단일 필드로 구성합니다. 예를 들면, `float[3] myvalues` 가 있습니다. `value` 에는 `key`가 설명하는 데이터가 들어있습니다.
  
  제각각의 키에 들어있는 정보 메세지는 전체 로그에 한번만 들어가야 합니다. 파서는 해당 메세지를 딕셔너리로 형태로 저장합니다.
  
  사전 정의 정보 메세지는 다음과 같습니다:

| 키                                   | 설명                    | 예제 값               |
| ----------------------------------- | --------------------- | ------------------ |
| char[value_len] sys_name          | 시스템 이름                | "PX4"              |
| char[value_len] ver_hw            | 하드웨어(보드) 버전           | "PX4FMU_V4"        |
| char[value_len] ver_hw_subtype    | 보드 하위 버전(변형판)         | "V2"               |
| char[value_len] ver_sw            | 소프트웨어 버전(git tag)     | "7f65e01"          |
| char[value_len] ver_sw_branch     | git 브랜치               | "master"           |
| uint32_t ver_sw_release           | 소프트웨어 버전 (아래 참고)      | 0x010401ff         |
| char[value_len] sys_os_name       | 운영 체제 명칭              | "Linux"            |
| char[value_len] sys_os_ver        | 운영체제 버전 (git tag)     | "9f82919"          |
| uint32_t ver_os_release           | 운영체제 버전 (아래 참고)       | 0x010401ff         |
| char[value_len] sys_toolchain     | 툴체인 명칭                | "GNU GCC"          |
| char[value_len] sys_toolchain_ver | 툴체인 버전                | "6.2.1"            |
| char[value_len] sys_mcu           | 칩 명칭과 버전              | "STM32F42x, rev A" |
| char[value_len] sys_uuid          | 기체 고유 식별자(예: MCU ID)  | "392a93e32fa3"...  |
| char[value_len] log_type          | 로그 형식(지정하지 않으면 전체 기록) | "mission"          |
| char[value_len] replay              | 재현 모드일 때 재현 파일 이름     | "log001.ulg"       |
| int32_t time_ref_utc              | 초 단위 UTC 시간 오프셋       | -3600              |

    `ver_sw_release`와 `ver_os_release` 형식은 0xAABBCCTT입니다. 여기서 AA는 메이저 버전, BB는 마이너 버전, CC는 패치 횟수, TT는 형식을 의미합니다. 
    형식 값은 다음을 따릅니다. >= 0`: 개발 버전, `>= 64`: 알파 버전, `>= 128`: 베타 버전, `>= 192`: 출시 후보, `== 255`: 출시 버전
    
    So for example 0x010402ff translates into the release version v1.4.2.
    
    This message can also be used in the Data section (this is however the preferred section).
    

- 'M': 다중 정보 메세지.
  
  ```c
  struct ulog_message_info_multiple_header_s {
    struct message_header_s header;
    uint8_t is_continued; ///< can be used for arrays
    uint8_t key_len;
    char key[key_len];
    char value[header.msg_size-2-key_len]
  };
  ```
  
  정보 메세지와 동일하지만 동일한 키를 지닌 다중 메세지가 되는 경우는 예외입니다(파서에서는 이 메세지를 목록으로 저장합니다). `is_continued`는 쪼갠 메세지에 사용할 수 있습니다. 1로 설정하면, 이전 메세지와 동일한 키를 지닌 부분입니다. 파서는 로그 메세지를 기록하는 순서 그대로 2차원 목록으로 다중 메세지의 모든 정보를 저장할 수 있습니다.

- 'P': 매개변수 메세지. `message_info_s`와 동일한 형식입니다. 매개변수가 실행 시간동안 수시로 바뀌는 경우, 이 메시지도 데이터 섹션에서 활용할 수 있습니다. 이 데이터 형식은 `int32_t`, `float` 두가지 형식으로 제한합니다.

이 절은 처음 `message_add_logged_s` 메세지 또는 `message_logging_s` 메세지 둘 중 어떤 메세지 하나를 시작하기 전에 끝납니다.

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
      
  
  `timestamp`: 마이크로초 단위, `log_level`: 리눅스 커널과 동일합니다:

| 이름      | 레벨 값 | 의미                  |
| ------- | ---- | ------------------- |
| EMERG   | '0'  | 시스템 사용 불가           |
| ALERT   | '1'  | 즉시 조치해야 함           |
| CRIT    | '2'  | 중대한 상황              |
| ERR     | '3'  | 오류 상황               |
| WARNING | '4'  | 경고 상황               |
| NOTICE  | '5'  | 보통의 상태이나 주시가 필요한 상황 |
| INFO    | '6'  | 정보                  |
| DEBUG   | '7'  | 디버깅 메시지             |

- 'C': 태그가 붙은 로깅 문자열 메세지
  
      struct message_logging_tagged_s {
        struct message_header_s header;
        uint8_t log_level;
        uint16_t tag;
        uint64_t timestamp;
        char message[header.msg_size-9]
      };
      
  
  `tag`: 로그 메세지 문자열의 원본을 나타내는 ID입니다. 과정, 스레드, 시스템 아키텍처별 클래스를 나타낼 수도 있습니다. 예를 들어, 제각각의 페이로드, 외부 디스크, 직렬 장치 등을 제어하려 다중 프로세스를 처리하는 온보드 컴퓨터용 참조 구현체는 다음과 같은 `message_logging_tagged_s` 구조체의 `uint16_t enum` 태그 속성으로 프로세스 식별자를 인코딩할 수 있습니다.
  
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
      
  
  `timestamp`: 마이크로초 단위, `log_level`: 리눅스 커널과 동일합니다:

| 이름      | 레벨 값 | 의미                  |
| ------- | ---- | ------------------- |
| EMERG   | '0'  | 시스템 사용 불가           |
| ALERT   | '1'  | 즉시 조치해야 함           |
| CRIT    | '2'  | 중대한 상황              |
| ERR     | '3'  | 오류 상황               |
| WARNING | '4'  | 경고 상황               |
| NOTICE  | '5'  | 보통의 상태이나 주시가 필요한 상황 |
| INFO    | '6'  | 정보                  |
| DEBUG   | '7'  | 디버깅 메시지             |

- 'S': 메시지를 동기화하여 리더 프로그램에서 다음 동기화 메시지를 검색하여 깨진 메세지를 복원할 수 있게 합니다.
  
      struct message_sync_s {
        struct message_header_s header;
        uint8_t sync_magic[8];
      };
      
  
  `sync_magic`: [0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12]

- 'O': 밀리초 단위의 주어진 시간을 초과하여 버린 메시지를 표기(손실 로깅 메세지)합니다. 장치 동작이 충분히 빠르지 않을 경우 로그메시지 손실이 나타날 수 있습니다.
  
      struct message_dropout_s {
        struct message_header_s header;
        uint16_t duration;
      };
      

- 'I': 정보 메세지. 상단 참고.

- 'M': 다중 정보 메세지. 상단 참고.

- 'P': 매개변수 메세지. 상단 참고.

## 파서 요구 사항

온전히 동작하는 ULog 파서는 다음 요건을 만족해야합니다:

- 알 수 없는 메시지는 무시해야 함(그러나 경고를 출력할 수 있음).
- 이후/알 수 없는 파일 형식 버전도 해석(그러나 경고를 출력할 수 있음).
- 로그에 파서에서 처리할 수 없는 깨진 변경이 들어있음을 의미하는 알 수 없는 비호환성 비트(`ulog_message_flag_bits_s` 메세지의 `incompat_flags`) 설정 값이 들어있을 경우 로그 해석을 거절해야함.
- 로그가 메세지 중간에 갑자기 끝났을 경우에도 제대로 처리할 수 있어야 함. 끝나지 않은 메시지는 무시해야 함.
- 후위 첨가 데이터: 파서에서 데이터 섹션이 존재함을 가정할 수 있어야 함. 예: 정의 섹션 다음의 위치를 오프셋이 가리킴.
  
  후위 첨가 데이터는 정규 데이터 섹션의 일부로 다루어야 함.

## 알려진 기존 구현체

- PX4 펌웨어: C++ 
  - [로거 모듈](https://github.com/PX4/Firmware/tree/master/src/modules/logger)
  - [재현 모듈](https://github.com/PX4/Firmware/tree/master/src/modules/replay)
  - [hardfault_log 모듈](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log): hardfault 치명 오류 데이터를 붙입니다.
- [pyulog](https://github.com/PX4/pyulog): CLI 스크립트가 들어있는 파이썬 ULog 파서 라이브러리.
- [FlightPlot](https://github.com/PX4/FlightPlot): Java언어로 작성한 로그 플로터.
- [pyFlightAnalysis](https://github.com/Marxlp/pyFlightAnalysis): pyulog 기반 로그 플로터, 3D 시각화 도구.
- [MAVLink](https://github.com/mavlink/mavlink): MAVLink를 통한 ULog 메세지 스트리밍 (후위 첨가 데이터는 지원하지 않습니다. 최소한 잘린 메세지의 무시는 지원하지 않습니다.).
- [QGroundControl](https://github.com/mavlink/qgroundcontrol): C++로 작성한 MAVLink ULog 스트리밍 및 간단한 GeoTagging 파싱 도구를 제공합니다.
- [mavlink-router](https://github.com/01org/mavlink-router): MAVLink ULog 스트리밍 기능을 제공하는 C++ 기반 프로그램.
- [MAVGAnalysis](https://github.com/ecmnet/MAVGCL): 자바 기반의 MAVLink ULog 스트리밍 기능과 도표 작성, 분석 기능을 제공하는 파서 기능을 갖춘 프로그램.
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler): 로그와 시간대를 플롯 차트로 출력하는 C++/Qt 기반 프로그램. 2.1.3부터 ULog를 지원합니다.
- [ulogreader](https://github.com/maxsun/ulogreader): ULog로그를 읽어 JSON 객체 포멧으로 출력하는 Javascript 기반 파서. 

## 파일 형식 버전 이력

### 버전 2 개정 사항

`ulog_message_info_multiple_header_s` 메시지와 `ulog_message_flag_bits_s` 메시지를 추가하고 로그에 데이터를 후위 첨가하는 기능을 붙였습니다. 기존 로그에 치명 오류 정보를 추가할 때 활용할 수 있습니다. 중간이 잘린 메세지에 데이터가 붙어 기록할 경우 버전 1의 해석 프로그램에서는 해석할 수 없습니다. 파서에서 알 수 없는 메세지를 무시한다면 전후 호환성을 확보할 수 있습니다.