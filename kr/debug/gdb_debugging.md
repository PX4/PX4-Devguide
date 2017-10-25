# Embedded Debugging

PX4를 실행하는 autopilot에서 GDB나 LLDB를 통한 디버깅이 가능합니다.

## 많은 메모리 사용하는 부분 찾기

아래에서 이 명령을 이용하면 가장 많이 정적 메모리 할당한 목록을 보여줍니다. :

<div class="host-code"></div>

```bash
arm-none-eabi-nm --size-sort --print-size --radix=dec build/px4fmu-v2_default/src/firmware/nuttx/firmware_nuttx | grep " [bBdD] "
```

NSH 명령은 남은 메모리의 양을 알려줍니다.:

```bash
free
```

그리고 top 명령은 application당 stack 사용량을 보여줍니다. :

```
top
```

stack 사용량은 stack coloring으로 계산되며, 현재 사용량이 아니라 task를 시작한 이후 최대량을 뜻합니다.

### 힙 할당 (Heap allocations)
동적 힙 할당은 [gperftools](https://github.com/gperftools/gperftools)로 SITL에서 POSIX상에서 추적이 가능합니다.

#### 설치 명령
##### Ubuntu:
```bash
sudo apt-get install google-perftools libgoogle-perftools-dev
```

#### 힙 프로파일링 시작하기

무엇보다 먼저 펌웨어를 다음과 같이 빌드 :
```bash
make posix_sitl_default
```
jmavsim 시작하기: `./Tools/jmavsim_run.sh`

다른 터미널에서 다음을 입력:
```bash
cd build/posix_sitl_default/tmp
export HEAPPROFILE=/tmp/heapprofile.hprof
export HEAP_PROFILE_TIME_INTERVAL=30
```

여러분이 사용하는 시스템에 따라 다음을 입력:
##### Fedora:
```bash
env LD_PRELOAD=/lib64/libtcmalloc.so ../src/firmware/posix/px4 ../../posix-configs/SITL/init/lpe/iris
pprof --pdf ../src/firmware/posix/px4 /tmp/heapprofile.hprof.0001.heap > heap.pdf
```

##### Ubuntu:
```bash
env LD_PRELOAD=/usr/lib/libtcmalloc.so ../src/firmware/posix/px4 ../../posix-configs/SITL/init/lpe/iris
google-pprof --pdf ../src/firmware/posix/px4 /tmp/heapprofile.hprof.0001.heap > heap.pdf
```

힙 할당 그래프로 pdf를 생성합니다.
그래프에서 숫자는 모드 0이 됩니다. 왜냐하면 단위가 MB이기 때문입니다. 대신에 퍼센트를 살펴봅시다. node나 subtree의 live 메모리를 보여주며 마지막까지 사용할 메모리를 의미합니다.

보다 자세한 내용은 [gperftools docs](https://htmlpreview.github.io/?https://github.com/gperftools/gperftools/blob/master/docs/heapprofile.html)을 참고하세요.


## NuttX에서 하드 폴트 디버깅하기 (Debugging Hard Faults in NuttX)

하드 폴트는 OS가 실행할 유효 명령어가 존재하지 않는다는 것을 검출할 때를 말합니다. 일반적으로 RAM의 핵심 영역에 문제가 있는 경우입니다. 잘못된 메모리 접근으로 stack이 날라가고 프로세서는 메모리의 주소가 마이크로프로세서 RAM내에 유효한 주소가 아니라는 것을 인지하게 됩니다.

  * NuttX는 2개 스택을 유지 : IRQ stack과 user stack
  * stack은 아래로 증가합니다. 따라서 아래 예제에서 가장 높은 주소는 0x20021060이며 사이즈는 0x11f4 (4596 bytes)이고 결과적으로 가장 낮은 주소는 0x2001fe6c입니다.

```bash
Assertion failed at file:armv7-m/up_hardfault.c line: 184 task: ekf_att_pos_estimator
sp:     20003f90
IRQ stack:
  base: 20003fdc
  size: 000002e8
20003f80: 080d27c6 20003f90 20021060 0809b8d5 080d288c 000000b8 08097155 00000010
20003fa0: 20003ce0 00000003 00000000 0809bb61 0809bb4d 080a6857 e000ed24 080a3879
20003fc0: 00000000 2001f578 080ca038 000182b8 20017cc0 0809bad1 20020c14 00000000
sp:     20020ce8
User stack:
  base: 20021060
  size: 000011f4
20020ce0: 60000010 2001f578 2001f578 080ca038 000182b8 0808439f 2001fb88 20020d4c
20020d00: 20020d44 080a1073 666b655b 65686320 205d6b63 6f6c6576 79746963 76696420
20020d20: 65747265 63202c64 6b636568 63636120 63206c65 69666e6f 08020067 0805c4eb
20020d40: 080ca9d4 0805c21b 080ca1cc 080ca9d4 385833fb 38217db9 00000000 080ca964
20020d60: 080ca980 080ca9a0 080ca9bc 080ca9d4 080ca9fc 080caa14 20022824 00000002
20020d80: 2002218c 0806a30f 08069ab2 81000000 3f7fffec 00000000 3b4ae00c 3b12eaa6
20020da0: 00000000 00000000 080ca010 4281fb70 20020f78 20017cc0 20020f98 20017cdc
20020dc0: 2001ee0c 0808d7ff 080ca010 00000000 3f800000 00000000 080ca020 3aa35c4e
20020de0: 3834d331 00000000 01010101 00000000 01010001 000d4f89 000d4f89 000f9fda
20020e00: 3f7d8df4 3bac67ea 3ca594e6 be0b9299 40b643aa 41ebe4ed bcc04e1b 43e89c96
20020e20: 448f3bc9 c3c50317 b4c8d827 362d3366 b49d74cf ba966159 00000000 00000000
20020e40: 3eb4da7b 3b96b9b7 3eead66a 00000000 00000000 00000000 00000000 00000000
20020e60: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
20020e80: 00000016 00000000 00000000 00010000 00000000 3c23d70a 00000000 00000000
20020ea0: 00000000 20020f78 00000000 2001ed20 20020fa4 2001f498 2001f1a8 2001f500
20020ec0: 2001f520 00000003 2001f170 ffffffe9 3b831ad2 3c23d70a 00000000 00000000
20020ee0: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
20020f00: 00000000 00000000 00000000 00000000 2001f4f0 2001f4a0 3d093964 00000001
20020f20: 00000000 0808ae91 20012d10 2001da40 0000260b 2001f577 2001da40 0000260b
20020f40: 2001f1a8 08087fd7 08087f9d 080cf448 0000260b 080afab1 080afa9d 00000003
20020f60: 2001f577 0809c577 2001ed20 2001f4d8 2001f498 0805e077 2001f568 20024540
20020f80: 00000000 00000000 00000000 0000260b 3d093a57 00000000 2001f540 2001f4f0
20020fa0: 0000260b 3ea5b000 3ddbf5fa 00000000 3c23d70a 00000000 00000000 000f423f
20020fc0: 00000000 000182b8 20017cc0 2001ed20 2001f4e8 00000000 2001f120 0805ea0d
20020fe0: 2001f090 2001f120 2001eda8 ffffffff 000182b8 00000000 00000000 00000000
20021000: 00000000 00000000 00000009 00000000 08090001 2001f93c 0000000c 00000000
20021020: 00000101 2001f96c 00000000 00000000 00000000 00000000 00000000 00000000
20021040: 00000000 00000000 00000000 00000000 00000000 0809866d 00000000 00000000
R0: 20000f48 0a91ae0c 20020d00 20020d00 2001f578 080ca038 000182b8 20017cc0
R8: 2001ed20 2001f4e8 2001ed20 00000005 20020d20 20020ce8 0808439f 08087c4e
xPSR: 61000000 BASEPRI: 00000000 CONTROL: 00000000
EXC_RETURN: ffffffe9
```

하드폴트를 디코딩하기 위해서 *정확히 일치하는* binary를 디버거로 로드합니다. :

<div class="host-code"></div>

```bash
arm-none-eabi-gdb build/px4fmu-v2_default/src/firmware/nuttx/firmware_nuttx
```

GDB 프롬프트에서, R8에서 마지막 명령과 flash에서 첫번째 주소로 시작합니다.(`0x080`로 시작하고 첫번째 주소는 `0x0808439f`임) 하드폴트가 있기 전에 마지막 동작 중에 하나는 ```mavlink_log.c```가 뭔가를 publish를 시도했다는 것입니다.

<div class="host-code"></div>

```gdb
(gdb) info line *0x0808439f
Line 77 of "../src/modules/systemlib/mavlink_log.c" starts at address 0x8084398 <mavlink_vasprintf+36>
   and ends at 0x80843a0 <mavlink_vasprintf+44>.
```

<div class="host-code"></div>

```gdb
(gdb) info line *0x08087c4e
Line 311 of "../src/modules/uORB/uORBDevices_nuttx.cpp"
   starts at address 0x8087c4e <uORB::DeviceNode::publish(orb_metadata const*, void*, void const*)+2>
   and ends at 0x8087c52 <uORB::DeviceNode::publish(orb_metadata const*, void*, void const*)+6>.
```
