# 모의시험 디버깅

모의시험은 호스트 머신에서 동작하므로, 모든 데스크톱 환경에서 도구를 활용할 수 있습니다.

## CLANG Address Sanitizer (Mac OS, Linux)

The Clang address sanitizer can help to find alignment (bus) errors and other memory faults like segmentation faults. The command below sets the right compile options.

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

디버깅 세션에서 DriverFrameworks 스케쥴링 혼동을 방지하려면 ```SIGCONT``` 를 LLDB나 GDB에서 마스킹 처리해야 합니다:

```bash
(lldb) process handle SIGCONT -n false -p false -s false
```

또는 GDB의 경우:

    (gdb) handle SIGCONT noprint nostop
    

After that the The lldb or gdb shells behave like normal sessions, please refer to the LLDB / GDB documentation.

The last parameter, the &lt;viewer\_model\_debugger&gt; triplet, is actually passed to make in the build directory, so

```sh
make px4_sitl_default jmavsim___gdb
```

명령은 다음 명령과 같습니다.

```sh
make px4_sitl_default   # Configure with cmake
make -C build/px4_sitl_default jmavsim___gdb
```

A full list of the available make targets in the build directory can be obtained with:

```sh
make help
```

but for your convenience, a list with just the &lt;viewer\_model\_debugger&gt; triplets is printed with the command

```sh
make list_vmd_make_targets
```

## Compiler optimization

It is possible to suppress compiler optimization for given executables and/or modules (as added by cmake with `add_executable` or `add_library`) when configuring for `posix_sitl_*`. This can be handy when it is necessary to step through code with a debugger or print variables that would otherwise be optimized out.

To do so, set the environment variable `PX4_NO_OPTIMIZATION` to be a semi-colon separated list of regular expressions that match the targets that need to be compiled without optimization. This environment variable is ignored when the configuration isn't `posix_sitl_*`.

For example,

```sh
export PX4_NO_OPTIMIZATION='px4;^modules__uORB;^modules__systemlib$'
```

would suppress optimization of the targets: platforms*\_posix**px4\_layer, modules**systemlib, modules**uORB, examples**px4\_simple\_app, modules**uORB*\_uORB\_tests and px4.

The targets that can be matched with these regular expressions can be printed with the command:

```sh
make -C build/posix_sitl_* list_cmake_targets
```