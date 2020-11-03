# 임베디드 디버깅

PX4를 실행하는 오토파일럿은 GDB/LLDB 디버깅을 지원합니다.

## 고용량 메모리 점유 프로그램 식별

아래 명령은 메모리 용량을 늘 가장 많이 차지하는 부분을 목록으로 보여줍니다:

```bash
arm-none-eabi-nm --size-sort --print-size --radix=dec build/px4_fmu-v2_default/px4_fmu-v2_default.elf | grep " [bBdD] "
```

NSH 명령은 남아있는 메모리 용량을 보여줍니다:

```bash
free
```

그리고 top 명령은 프로그램당 스택 사용량을 보여줍니다:

    top
    

스택 사용량은 스택에 쌓아둔 공간 용량을 계산하기에, 현재 사용량은 아니며 작업 시작 후 최대 점유량을 나타냅니다.

### 힙 할당

동적 힙 할당은 [gperttools](https://github.com/gperftools/gperftools)로 SITL의 POSIX 환경에서 추적할 수 있습니다.

#### 설치 방법

##### 우분투:

```bash
sudo apt-get install google-perftools libgoogle-perftools-dev
```

#### 힙 프로파일링 시작

우선 다음과 같이 펌웨어를 빌드하십시오:

```bash
make px4_sitl_default
```

jMAVSim을 시작하십시오: `./Tools/jmavsim_run.sh`

다른 터미널에서 다음 명령을 입력하십시오

```bash
cd build/px4_sitl_default/tmp/rootfs
export HEAPPROFILE=/tmp/heapprofile.hprof
export HEAP_PROFILE_TIME_INTERVAL=30
```

시스템에 따라 다음 명령행을 입력하십시오:

##### 페도라:

```bash
env LD_PRELOAD=/lib64/libtcmalloc.so PX4_SIM_MODEL=iris ../../bin/px4 ../../etc -s etc/init.d-posix/rcS
pprof --pdf ../src/firmware/posix/px4 /tmp/heapprofile.hprof.0001.heap > heap.pdf 
```

##### 우분투:

```bash
env LD_PRELOAD=/usr/lib/libtcmalloc.so PX4_SIM_MODEL=iris ../../bin/px4 ../../etc -s etc/init.d-posix/rcS
google-pprof --pdf ../src/firmware/posix/px4 /tmp/heapprofile.hprof.0001.heap > heap.pdf 
```

힙 할당 그래프를 넣은 pdf 문서가 나타납니다. 그래프의 숫자는 MB 단위기 때문에 모두 0입니다. 그래서 백분율만을 대신 보겠습니다. 실제 동작하는 메모리(노드, 하위트리의 점유 대상)를 보여주는데, 프로그램이 끝나기 전까지도 여전히 메모리를 사용중이었음을 의미합니다.

자세한 정보는 [gperftools 문서](https://htmlpreview.github.io/?https://github.com/gperftools/gperftools/blob/master/docs/heapprofile.html)를 살펴보십시오.

## 하드웨어 오류 디버깅

하드웨어 오류는 CPU 에서 잘못된 명령어를 받아 처리하려 하거나 잘못된 메모리 주소로 접근하려 할 경우 나타나는 상태입니다. RAM의 핵심 영역이 깨졌을 때 나타날 수 있는 현상입니다.

### 비디오

다음 동영상에서는 Eclipse와 JTAG 디버거로 하드웨어 오류 디버깅을 하는 방법을 시연합니다. 이 내용은 2019년 PX4 개발자 컨퍼런스에서 다루었습니다.

{% youtube %} https://www.youtube.com/watch?v=KZkAM_PVOi0 {% endyoutube %}

### NuttX에서 하드웨어 오류 디버깅

하드웨어 오류는 보통 프로세서에서 스택을 엎어쓰거나 하여 프로세서에서 스택 주소 오류를 반환하는 상황에서 나타날 수 있습니다. 이 상황은 아마도 코드의 포인터가 메모리 들판을 헤집고 다니다 스택을 깨먹거나, 작업 스택을 다른 작업이 엎어버리는 버그로 인해 나타납니다.

* NuttX는 두 종류의, 인터럽트를 처리하는 IRQ 스택과 사용자 스택을 관리합니다
* 스택의 점유량은 아래로 늘어납니다. 따라서 아래 예제에서 가장 높은 지점의 주소는 0x20021060 으로, 사이즈는 0x11f4(4596 바이트)이며, 고로 최하위 주소는 0x2001fe6c입니다.

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

하드웨어 오류를 디코딩하려면 *정확한* 바이너리를 디버거에 불러오십시오:

```bash
arm-none-eabi-gdb build/px4_fmu-v2_default/px4_fmu-v2_default.elf
```

그 다음 GDB 프롬프트에서 플래시의 처음 주소에 있는 R8의 마지막 인스트럭션부터 시작하십시오(`0x080`부터 시작하므로 처음 부분은 `0x0808439f`임). 실행은 왼쪽에서 오른쪽으로 진행합니다. 따라서 하드웨어 오류가 발생하기 전의 마지막 단계중 하나는, 다음 파일이 ```mavlink_log.c``` 무언가를 내보낼 때 나타납니다.

```gdb
(gdb) info line *0x0808439f
Line 77 of "../src/modules/systemlib/mavlink_log.c" starts at address 0x8084398 <mavlink_vasprintf+36>
   and ends at 0x80843a0 <mavlink_vasprintf+44>.
```

```gdb
(gdb) info line *0x08087c4e
Line 311 of "../src/modules/uORB/uORBDevices_nuttx.cpp"
   starts at address 0x8087c4e <uORB::DeviceNode::publish(orb_metadata const*, void*, void const*)+2>
   and ends at 0x8087c52 <uORB::DeviceNode::publish(orb_metadata const*, void*, void const*)+6>.
```