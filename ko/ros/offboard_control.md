!REDIRECT "https://docs.px4.io/master/ko/ros/offboard_control.html"

# 보드 외부 제어

> **Warning** [보드 외부](https://docs.px4.io/master/en/flight_modes/offboard.html) 제어는 위험합니다. 개발자에게는 충분히 보드 외부 제어 비행을 수행하기 전 준비하고 시험했으며 안전 예방책을 수립했는지 확인할 책임이 있습니다.

보드 외부 제어 개념은 오토파일럿 외부에서 동작하는 프로그램으로 PX4 비행 스택을 제어할 수 있음에 기인합니다. MAVLink 프로토콜에서 [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) 와 [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET) 메세지로 처리가 가능합니다.

## 보드 외부 제어 펌웨어 설치

보드 외부 개발을 시작하기 전 펌웨어 측면에서 설정해야 할 두가지가 있습니다.

### OFFBOARD 모드 활성화용 원격 조종 스위치 매핑

이 과정을 수행하려면, *QGroundControl*의 매개변수를 불러온 후 RC_MAP_OFFB_SW 매개변수를 찾아 OFFBOARD 모드를 활성화 할, 할당 가능한 원격 조종 채널값으로 설정합니다. OFFBOARD 모드에서 벗어나 위치 제어 상태로 진입해야 할 때 이런 식으로 원격 조종 스위치 입력을 대응을 해주면 도움이 될 수 있습니다.

MAVLink 메세지로 OFFBOARD 모드를 활성화할 수 있으므로, 이 단계를 반드시 거쳐야 하는건 아닙니다. 단, 이 방식이 훨씬 안전하다고 간주합니다.

### 보조 컴퓨터 인터페이스 활성화

보조 컴퓨터에 연결할 MAVLink 직렬 포트를 활성화하십시오([보조 컴퓨터 설정](../companion_computer/pixhawk_companion.md) 참고).

## 하드웨어 설정

보통 보드 외부 통신을 설정하는 방법에는 세가지가 있습니다.

### 직렬 무선 통신

1. UART 포트로 연결한 하나는 오토파일럿
2. 한쪽은 지상 통제 장치 컴퓨터에 연결합니다

무선 통신 장치의 예는 다음과 같습니다:

* [Lairdtech RM024](http://www.lairdtech.com/products/rm024)
* [Digi International XBee Pro](http://www.digi.com/products/xbee-rf-solutions/modules)

[![mermaid 도표: mavlink 채널](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGduZFtHcm91bmQgU3RhdGlvbl0gLS1NQVZMaW5rLS0-IHJhZDFbR3JvdW5kIFJhZGlvXTtcbiAgcmFkMSAtLVJhZGlvUHJvdG9jb2wtLT4gcmFkMltWZWhpY2xlIFJhZGlvXTtcbiAgcmFkMiAtLU1BVkxpbmstLT4gYVtBdXRvcGlsb3RdOyIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGduZFtHcm91bmQgU3RhdGlvbl0gLS1NQVZMaW5rLS0-IHJhZDFbR3JvdW5kIFJhZGlvXTtcbiAgcmFkMSAtLVJhZGlvUHJvdG9jb2wtLT4gcmFkMltWZWhpY2xlIFJhZGlvXTtcbiAgcmFkMiAtLU1BVkxpbmstLT4gYVtBdXRvcGlsb3RdOyIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)

<!-- original mermaid graph
graph TD;
  gnd[Ground Station] --MAVLink-- > rad1[Ground Radio];
  rad1 --RadioProtocol-- > rad2[Vehicle Radio];
  rad2 --MAVLink-- > a[Autopilot];
-->

### 보드 내장 프로세서

장치에 붙은 소형 컴퓨터는 UART to USB 어댑터로 오토파일럿에 연결합니다. 어떤 작업을 보드상에서 처리하느냐에 따라 오토파일럿에 추가로 명령을 보내 처리할 수 있습니다.

몇가지 저전력 보드를 예로 들면:

* [Odroid C1+](https://www.hardkernel.com/shop/odroid-c1/) 또는 [Odroid XU4](https://magazine.odroid.com/odroid-xu4)
* [라즈베리 파이](https://www.raspberrypi.org/)
* [인텔 에디슨](http://www.intel.com/content/www/us/en/do-it-yourself/edison.html)

좀 더 큰 고 전력 소비 보드를 예로 들면:

* [인텔 NUC](http://www.intel.com/content/www/us/en/nuc/overview.html)
* [기가바이트 브릭스](http://www.gigabyte.com/products/list.aspx?s=47&ck=104)
* [엔비디아 젯슨 TK1](https://developer.nvidia.com/jetson-tk1)

[![mermaid 구성도: 보조 mavlink](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGNvbXBbQ29tcGFuaW9uIENvbXB1dGVyXSAtLU1BVkxpbmstLT4gdWFydFtVQVJUIEFkYXB0ZXJdO1xuICB1YXJ0IC0tTUFWTGluay0tPiBBdXRvcGlsb3Q7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVEQ7XG4gIGNvbXBbQ29tcGFuaW9uIENvbXB1dGVyXSAtLU1BVkxpbmstLT4gdWFydFtVQVJUIEFkYXB0ZXJdO1xuICB1YXJ0IC0tTUFWTGluay0tPiBBdXRvcGlsb3Q7IiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQifSwidXBkYXRlRWRpdG9yIjpmYWxzZX0)

<!-- original mermaid graph
graph TD;
  comp[Companion Computer] --MAVLink-- > uart[UART Adapter];
  uart --MAVLink-- > Autopilot;
-->

### ROS로의 보드 내장 프로세서와 무선랜 링크(***추천***)

기체에 붙은 소형 컴퓨터는 오토파일럿에 UART to USB 어댑터로 연결하며, ROS 실행 환경에서는 지상 통제 장치와 무선랜으로 연결합니다. 위 절에서 언급한 어떤 컴퓨터는 무선랜 어댑터로 연결할 수 있습니다. 예를 들면, 인텔 NUC D34010WYB 에는 PCI 익스프레스 Half-Mini 커넥터가 있는데 여기에 [Intel WiFi Link 5000](http://www.intel.com/products/wireless/adapters/5000/) 어댑터를 붙일 수 있습니다.

[![mermaid 도표: ROS](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVERcbiAgc3ViZ3JhcGggR3JvdW5kICBTdGF0aW9uXG4gIGduZFtST1MgRW5hYmxlZCBDb21wdXRlcl0gLS0tIHFnY1txR3JvdW5kQ29udHJvbF1cbiAgZW5kXG4gIGduZCAtLU1BVkxpbmsvVURQLS0-IHdbV2lGaV07XG4gIHFnYyAtLU1BVkxpbmstLT4gdztcbiAgc3ViZ3JhcGggVmVoaWNsZVxuICBjb21wW0NvbXBhbmlvbiBDb21wdXRlcl0gLS1NQVZMaW5rLS0-IHVhcnRbVUFSVCBBZGFwdGVyXVxuICB1YXJ0IC0tLSBBdXRvcGlsb3RcbiAgZW5kXG4gIHcgLS0tIGNvbXAiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiZ3JhcGggVERcbiAgc3ViZ3JhcGggR3JvdW5kICBTdGF0aW9uXG4gIGduZFtST1MgRW5hYmxlZCBDb21wdXRlcl0gLS0tIHFnY1txR3JvdW5kQ29udHJvbF1cbiAgZW5kXG4gIGduZCAtLU1BVkxpbmsvVURQLS0-IHdbV2lGaV07XG4gIHFnYyAtLU1BVkxpbmstLT4gdztcbiAgc3ViZ3JhcGggVmVoaWNsZVxuICBjb21wW0NvbXBhbmlvbiBDb21wdXRlcl0gLS1NQVZMaW5rLS0-IHVhcnRbVUFSVCBBZGFwdGVyXVxuICB1YXJ0IC0tLSBBdXRvcGlsb3RcbiAgZW5kXG4gIHcgLS0tIGNvbXAiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlfQ)

<!-- original mermaid graph
graph TD
  subgraph Ground  Station
  gnd[ROS Enabled Computer] --- qgc[qGroundControl]
  end
  gnd --MAVLink/UDP-- > w[WiFi];
  qgc --MAVLink-- > w;
  subgraph Vehicle
  comp[Companion Computer] --MAVLink-- > uart[UART Adapter]
  uart --- Autopilot
  end
  w --- comp
-->