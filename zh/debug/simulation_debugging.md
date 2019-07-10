# 仿真调试

当模拟在主机上运行时，所有桌面开发工具都可用。

## CLANG Address Sanitizer (Mac OS, Linux)

CLANG Address Sanitizer 可以帮助查找对齐（总线）错误和其他内存错误，如段错误。 下面的命令设置了正确的编译选项。

```sh
make clean # 仅需在常规编译后，第一次运行 address sanitizer 时使用
PX4_ASAN=1 make px4_sitl jmavsim
```

## Valgrind

```sh
brew install valgrind
```

或

```sh
sudo apt-get install valgrind
```

To use valgrind during the SITL simulation:

```sh
make px4_sitl_default jmavsim___valgrind
```

## 开始组合

SITL can be launched with and without debugger attached and with either jMAVSim or Gazebo as simulation backend. This results in the start options below:

```sh
make px4_sitl_default jmavsim
make px4_sitl_default jmavsim___gdb
make px4_sitl_default jmavsim___lldb

make px4_sitl_default gazebo
make px4_sitl_default gazebo___gdb
make px4_sitl_default gazebo___lldb
```

where the last parameter is the &lt;viewer\_model\_debugger&gt; triplet (using three underscores implies the default 'iris' model). This will start the debugger and launch the SITL application. In order to break into the debugger shell and halt the execution, hit ```CTRL-C```:

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

In order to not have the DriverFrameworks scheduling interfere with the debugging session ```SIGCONT``` should be masked in LLDB and GDB:

```bash
(lldb) process handle SIGCONT -n false -p false -s false
```

Or in the case of GDB:

    (gdb) handle SIGCONT noprint nostop
    

After that the The lldb or gdb shells behave like normal sessions, please refer to the LLDB / GDB documentation.

The last parameter, the &lt;viewer\_model\_debugger&gt; triplet, is actually passed to make in the build directory, so

```sh
make px4_sitl_default jmavsim___gdb
```

is equivalent with

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

## 编译器优化

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