# 오프보드 제어

> ** Warning ** 오프보드 제어는 위험합니다. 오프보드 비행 전에 안전을 위해 준비, 테스팅, 안전에 관해 예방책을 마련하는 것은 개발자의 몫입니다.

오프보드 제어의 기반 아이디어는 autopilot 외부에서 실행되는 소프트웨어로 px4 flight stack을 제어할 수 있다는 것입니다. Mavlink 프로토콜을 통해 이뤄지며 특별히 [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)와 [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) 메시지와 관련이 있습니다.

## 오프보드 제어 펌웨어 셋업
오프보드 개발을 시작하기 전에 펌웨어 쪽에 필요한 셋업 2가지

### 1. RC 스위치를 오프보드 모드 활성화로 매핑
이를 위해, QGroundControl내에서 파라미터를 로드하고 오프보드 모드를 활성화시키기 위해서 사용할려는 RC 채널을 할당하기 위한 RC_MAP_OFFB_SW 파라미터를 찾습니다. 오프보드 모드를 빠져나올때 position 제어로 들어가게 하는 방식이 유용할 수 있습니다.

비록 이 단계는 필수는 아니며 MAVLink 메시지를 사용해서 오프보드 모드를 활성화시킬 수 있습니다. 이 방식 훨씬 안전하다고 생각합니다.

### 2. 컴패니언 컴퓨터 인터페이스 활성화
[SYS_COMPANION](../advanced/parameter_reference.md#system) 파라미터를 찾고 921600(추천)이나 57600으로 설정합니다. 이 파라미터는 온보드 모드에 특화된 데이터 스트림을 가지는 Telem2 포트에 MAVLink 스트림을 활성화시킬 수 있습니다. 여기서 적절한 baud rate는 921600 8N1 혹은 57600 8N1입니다.

이런 데이터 스트림에 대한 상세 정보는 [소스 코드](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_main.cpp)에서 "MAVLINK_MODE_ONBOARD"를 찾습니다.

## 하드웨어 셋업

일반적인 오프보드 통신을 셋업하는 3가지 방식

### 1. 시리얼 라디오
1. 한쪽을 autopilot의 UART 포트에 연결
2. 한쪽을 ground station 컴퓨터에 연결

예제 라디오는 다음을 포함
* [Lairdtech RM024](http://www.lairdtech.com/products/rm024)
* [Digi International XBee Pro](http://www.digi.com/products/xbee-rf-solutions/modules)

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGduZFtHcm91bmQgU3RhdGlvbl0gLS1NQVZMaW5rLS0-IHJhZDFbR3JvdW5kIFJhZGlvXTtcbiAgcmFkMSAtLVJhZGlvUHJvdG9jb2wtLT4gcmFkMltWZWhpY2xlIFJhZGlvXTtcbiAgcmFkMiAtLU1BVkxpbmstLT4gYVtBdXRvcGlsb3RdOyIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGduZFtHcm91bmQgU3RhdGlvbl0gLS1NQVZMaW5rLS0-IHJhZDFbR3JvdW5kIFJhZGlvXTtcbiAgcmFkMSAtLVJhZGlvUHJvdG9jb2wtLT4gcmFkMltWZWhpY2xlIFJhZGlvXTtcbiAgcmFkMiAtLU1BVkxpbmstLT4gYVtBdXRvcGlsb3RdOyIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)

### 2. On-board 프로세서
autopilot에 연결하는 작은 컴퓨터는 UART를 통해 USB 어댑터에 연결합니다. autopilot에 추가로 명령을 보내고자 한다면 추가하는 온보드 프로세싱 종류에 따라서 다양한 선택이 가능합니다.

작고 낮은 성능을 필요로 하는 경우:
* [Odroid C1+](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143703355573) or [Odroid XU4](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143452239825)
* [Raspberry Pi](https://www.raspberrypi.org/)
* [Intel Edison](http://www.intel.com/content/www/us/en/do-it-yourself/edison.html)

좀더 크고 높은 성능을 필요로 하는 경우:
* [Intel NUC](http://www.intel.com/content/www/us/en/nuc/overview.html)
* [Gigabyte Brix](http://www.gigabyte.com/products/list.aspx?s=47&ck=104)
* [Nvidia Jetson TK1](https://developer.nvidia.com/jetson-tk1)

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGNvbXBbQ29tcGFuaW9uIENvbXB1dGVyXSAtLU1BVkxpbmstLT4gdWFydFtVQVJUIEFkYXB0ZXJdO1xuICB1YXJ0IC0tTUFWTGluay0tPiBBdXRvcGlsb3Q7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGNvbXBbQ29tcGFuaW9uIENvbXB1dGVyXSAtLU1BVkxpbmstLT4gdWFydFtVQVJUIEFkYXB0ZXJdO1xuICB1YXJ0IC0tTUFWTGluay0tPiBBdXRvcGlsb3Q7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)

### 3. 온보드 프로세서와 wifi로 ROS에 연결 (***추천***)
비행체에 부착하는 작은 컴퓨터는 UART USB 아답터를 통해 autopilot로 연결하며 ROS가 실행되고 있는 ground station에 WiFi 링크를 가질 수 있습니다. WiFi 아답터와 결합된 위 섹션에 소개한 어떤 컴퓨터도 가능합니다. 예로 Intel NUC D34010WYB는 [Intel Wifi Link 5000](http://www.intel.com/products/wireless/adapters/5000/) 아답터를 제공하는 PCI Express Half-Mini 커넥터가 있습니다.


[![](https://mermaid.ink/img/eyJjb2RlIjoiXHRncmFwaCBURFxuXHRzdWJncmFwaCBHcm91bmQgIFN0YXRpb25cblx0ICBnbmRbUk9TIEVuYWJsZWQgQ29tcHV0ZXJdIC0tLSBxZ2NbcUdyb3VuZENvbnRyb2xdXG5cdGVuZFxuXHRnbmQgLS1NQVZMaW5rL1VEUC0tPiB3W1dpRmldO1xuXHRxZ2MgLS1NQVZMaW5rLS0-IHc7XG5cdHN1YmdyYXBoIFZlaGljbGVcblx0ICBjb21wW0NvbXBhbmlvbiBDb21wdXRlcl0gLS1NQVZMaW5rLS0-IHVhcnRbVUFSVCBBZGFwdGVyXVxuXHR1YXJ0IC0tLSBBdXRvcGlsb3Rcblx0ZW5kXG5cdHcgLS0tIGNvbXAiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiXHRncmFwaCBURFxuXHRzdWJncmFwaCBHcm91bmQgIFN0YXRpb25cblx0ICBnbmRbUk9TIEVuYWJsZWQgQ29tcHV0ZXJdIC0tLSBxZ2NbcUdyb3VuZENvbnRyb2xdXG5cdGVuZFxuXHRnbmQgLS1NQVZMaW5rL1VEUC0tPiB3W1dpRmldO1xuXHRxZ2MgLS1NQVZMaW5rLS0-IHc7XG5cdHN1YmdyYXBoIFZlaGljbGVcblx0ICBjb21wW0NvbXBhbmlvbiBDb21wdXRlcl0gLS1NQVZMaW5rLS0-IHVhcnRbVUFSVCBBZGFwdGVyXVxuXHR1YXJ0IC0tLSBBdXRvcGlsb3Rcblx0ZW5kXG5cdHcgLS0tIGNvbXAiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)
