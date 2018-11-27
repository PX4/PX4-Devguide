# 시뮬레이션 디버깅

시뮬레이션을 호스트 머신에서 돌리는 경우, 데스크톱에 있는 모든 개발 도구를 활용할 수 있습니다.

## CLANG Address Sanitizer (Mac OS, Linux)

Clag address sanitizer는 세그멘테이션 폴트와 같이 alignment 에러나 다른 메모리 폴트를 찾는데 도움을 얻을 수 있습니다. 아래 명령과 같이 올바른 컴파일 옵션을 지정합니다.

```sh
make clean # only required on first address sanitizer run after a normal build
PX4_ASAN=1 make px4_sitl jmavsim
```

## Valgrind

```sh
brew install valgrind
```

or

```sh
sudo apt-get install valgrind
```

> **Todo** Valgrind 실행하는 방법에 대한 내용 추가

## combinations 시작하기

SITL은 디버거가 연결과 상관없이 jMAVSIm이나 Gazebo와 같은 백엔드 시뮬레이션과 함께 실행할 수 있습니다. 다음과 같은 시작 옵션들로 :

```sh
make px4_sitl_default jmavsim
make px4_sitl_default jmavsim___gdb
make px4_sitl_default jmavsim___lldb

make px4_sitl_default gazebo
make px4_sitl_default gazebo___gdb
make px4_sitl_default gazebo___lldb
```

마지막 parameter에 &lt;viewer\_model\_debugger&gt; 3개가 들어갑니다.(밑줄 3개는 기본으로 &#39;iris&#39; 모델을 의미)
이렇게 하면 디버거를 구동시키고 SITL application을 실행시킵니다. debugger shell로 들어가려면 실행을 중단시키기 위해 ```CTRL-C```을 누릅니다. :

```gdb
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

디버깅하는 동안 DriverFrameworks scheduling에 영향을 주지 않기 위해서는 LLDB와 GDB내에 ```SIGCONT``` 를 마스킹해야만 합니다. :

```bash
(lldb) process handle SIGCONT -n false -p false -s false
```

혹은 GDB의 경우 :

```
(gdb) handle SIGCONT noprint nostop
```

다음으로 lldb나 gdb shell은 일반 세션처럼 동작합니다. LLDB / GDB 문서를 참고하세요.

마지막 파라미터 &lt;viewer\_model\_debugger&gt; 3개는 실제로 빌드 디렉토리에 있는 make로 전달됩니다. 따라서

```sh
make px4_sitl_default jmavsim___gdb
```

는 다음과 동일합니다.

```sh
make px4_sitl_default	# Configure with cmake
make -C build/px4_sitl_default jmavsim___gdb
```

빌드 디렉토리에서 make target 가능한 모든 목록을 보고 싶다면 :

```sh
make help
```

하지만 편의를 위해서  &lt;viewer\_model\_debugger&gt; 3개 목록은 해당 명령과 함께 출력합니다.

```sh
make list_vmd_make_targets
```

## 컴파일러 최적화

`posix_sitl_*`을 설정할 때 실행자와 모듈(`add_executable`나 `add_library`를 cmake에 추가하는 것과 같이)에 대한 컴파일러 최적화를 숨기는 것이 가능하다. 이렇게 하면 디버거로 코드를 단계별로 실행해 나가거나 최적화시키지 않은 변수를 출력하는데 편리하다.

이렇게 하기 위해서는 `PX4_NO_OPTIMIZATION` 환경변수를 정규표현의 세미콜론으로 분리시킬 수 있다. 정규표현으로 컴파일에서 최적화가 필요없는 타겟을 매치시키기 편리합니다. 설정이 `px4_sitl*`이 아닌 경우에는 이 환경변수는 무시됩니다.

예로,

```sh
export PX4_NO_OPTIMIZATION='px4;^modules__uORB;^modules__systemlib$'
```

타겟의 최적화를 없애려면 : platforms\_\_posix\_\_px4\_layer, modules\_\_systemlib, modules\_\_uORB, examples\_\_px4\_simple\_app, modules\_\_uORB\_\_uORB\_tests and px4.

이 정규표현으로 매치시킬 수 있는 타겟을 아래 명령으로 출력 :

```sh
make -C build/posix_sitl_* list_cmake_targets
```
