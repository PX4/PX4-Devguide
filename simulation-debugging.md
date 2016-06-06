# Simulation Debugging

As the simulation is running on the host machine, all the desktop development tools are available.

## CLANG Address Sanitizer (Mac OS, Linux)

The Clang address sanitizer can help to find alignment (bus) errors and other memory faults like segmentation fauls. The command below sets the right compile options.


<div class="host-code"></div>

```sh
make clean # only required on first address sanitizer run after a normal build
MEMORY_DEBUG=1 make posix jmavsim
```

## Valgrind

<div class="host-code"></div>

```sh
brew install valgrind
```

or

<div class="host-code"></div>

```sh
sudo apt-get install valgrind
```

<aside class="todo">
Add instructions how to run Valgrind
</aside>

## Start combinations

SITL can be launched with and without debugger attached and with either jMAVSim or Gazebo as simulation backend. This results in the start options below:

<div class="host-code"></div>

```sh
make posix_sitl_default jmavsim
make posix_sitl_default jmavsim_gdb
make posix_sitl_default jmavsim_lldb

make posix_sitl_default gazebo
make posix_sitl_default gazebo_gdb
make posix_sitl_default gazebo_lldb

make posix_sitl_lpe jmavsim
make posix_sitl_lpe jmavsim_gdb
make posix_sitl_lpe jmavsim_lldb

make posix_sitl_lpe gazebo
make posix_sitl_lpe gazebo_gdb
make posix_sitl_lpe gazebo_lldb
```
