# 모의시험 디버깅

모의시험은 호스트 머신에서 동작하므로, 모든 데스크톱 환경에서 도구를 활용할 수 있습니다.

## CLANG Address Sanitizer (Mac OS, Linux)

Clang address sanitizer는 정렬(버스)오류 또는 세그먼테이션 폴트같은 기타 메모리 오류를 잡아내줄 수 있습니다. 아래 명령은 적절한 컴파일 옵션을 반영합니다.

```sh
make clean # only required on first address sanitizer run after a normal build
PX4_ASAN=1 make px4_sitl jmavsim
```

## Valgrind

```sh
brew install valgrind
```

또는

```sh
sudo apt-get install valgrind
```

명령으로 SITL 모의시험 환경을 실행하는 동안 valgrind를 사용할 수 있습니다:

```sh
make px4_sitl_default jmavsim___valgrind
```

## 시작 조합

SITL은 디버거를 붙이거나 그렇지 않은 상태에서 시작할 수 있으며 jMAVSim 또는 가제보를 모의시험 백엔드로 붙일 수 있습니다. 다음 명령을 내리면 준비할 수 있습니다:

```sh
make px4_sitl_default jmavsim
make px4_sitl_default jmavsim___gdb
make px4_sitl_default jmavsim___lldb

make px4_sitl_default gazebo
make px4_sitl_default gazebo___gdb
make px4_sitl_default gazebo___lldb
```

마지막 매개변수는 변수가 셋인 &lt;viewer\_model\_debugger&gt;입니다(밑줄 문자 셋을 활용하는 부분은 기본 'iris'모델임을 의미). 이 명령은 SITL 프로그램과 디버거를 시작합니다. 실행을 멈추고 디버거로 진입하려면 다음 키를 누르십시오 ```CTRL-C```:

```sh
Process 16529 stopped
* thread #1: tid = 0x114e6d, 0x00007fff90f4430a libsystem_kernel.dylib`__read_nocancel + 10, name = 'px4', queue = 'com.apple.main-thread', stop reason = signal SIGSTOP
    frame #0: 0x00007fff90f4430a libsystem_kernel.dylib`__read_nocancel + 10
libsystem_kernel.dylib`__read_nocancel:
->  0x7fff90f4430a <+10>: jae    0x7fff90f44314            ; <+20>
    0x7fff90f4430c <+12>: movq   %rax, %rdi
    0x7fff90f4430f <+15>: jmp    0x7fff90f3fc53            ; cerror_nocancel
    0x7fff90f44314 <+20>: retq
(lldb) 
```

디버깅 세션에서 DriverFrameworks 스케쥴링 혼동을 방지하려면 ```SIGCONT``` 를 LLDB나 GDB에서 마스킹 처리해야 합니다:

```bash
(lldb) process handle SIGCONT -n false -p false -s false
```

또는 GDB의 경우:

    (gdb) handle SIGCONT noprint nostop
    

이 설정이 끝나고 나면 lldb 또는 gdb 셸에서 일반 세션처럼 동작합니다. LLDB / GDB 문서를 참고하십시오.

마지막 매개변수는 변수가 셋인 &lt;viewer\_model\_debugger&gt; 입니다. 실제로 이 매개변수 값을 빌드 디렉터리 형태로 전달합니다. 따라서,

```sh
make px4_sitl_default jmavsim___gdb
```

명령은 다음 명령과 같습니다.

```sh
make px4_sitl_default   # Configure with cmake
make -C build/px4_sitl_default jmavsim___gdb
```

빌드 디렉터리의 빌드 대상 전체 목록을 활용하려면 다음 명령을 활용하십시오:

```sh
make help
```

편의상, 변수셋을 지닌 &lt;viewer\_model\_debugger&gt; 형식의 값은 다음 명령으로 출력합니다

```sh
make list_vmd_make_targets
```

## 컴파일러 최적화

`posix_sitl_*`을 설정할 때, 주어진 실행 파일 또는 모듈(cmake에 `add_executable` 또는 `add_library`로 추가)에 대해 컴파일러 최적화를 막을 수 있습니다. 디버거로 코드 실행을 진행할 때 또는 최적화 할 변수값을 출력할 때 이 조치가 상당히 유용할 수 있습니다.

최적화 방지를 진행하려면, 세미콜론으로 구분한 최적화 과정을 거치지 않고 컴파일해야 하는 다수의 대상을 아우르는 정규 표현식 값 목록 형식으로 `PX4_NO_OPTIMIZATION` 환경 변수를 설정하십시오. 이 환경 변수는 컴파일 대상이 `posix_sitl_*` 설정값에 해당하지 않으면 무시합니다.

예를 들어,

```sh
export PX4_NO_OPTIMIZATION='px4;^modules__uORB;^modules__systemlib$'
```

설정은 다음 대상의 최적화를 막아줍니다: platforms*\_posix**px4\_layer, modules**systemlib, modules**uORB, examples**px4\_simple\_app, modules**uORB*\_uORB\_tests 와 px4. 

다음 명령으로 정규 표현식에 일치하는 대상을 출력할 수 있습니다:

```sh
make -C build/posix_sitl_* list_cmake_targets
```