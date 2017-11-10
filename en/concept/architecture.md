# Architectural Overview

PX4 consists of two main layers: the [flight stack](#flight-stack) is an autopilot software solution,
and the [middleware](#middleware) is a general robotics layer that can support any type of autonomous robot.

All PX4 [airframes](../airframes/README.md) share a single codebase (this includes other robotic systems like boats, rovers, submarines etc.). The complete system design is [reactive](http://www.reactivemanifesto.org), which means that:

- All functionality is divided into exchangeable components
- Communication is done by asynchronous message passing 
- The system can deal with varying workload

In addition to these runtime considerations, its modularity maximizes [reusability](https://en.wikipedia.org/wiki/Reusability).


## High Level Software Architecture

The diagram below provides an overview of the building blocks of PX4. 
The top part of the diagram contains middleware blocks, while the lower
section shows the components of the flight stack.

![PX4 Architecture](../../assets/diagrams/PX4_Architecture.svg)

<!-- This diagram can be updated from 
[here](https://drive.google.com/file/d/0B1TDW9ajamYkaGx3R0xGb1NaeU0/view?usp=sharing) 
and opened with draw.io Diagrams. -->

The source code is split into self-contained modules (shown in `monospace` in the
diagram). Usually a building block corresponds to exactly one module. At
runtime, you can inspect which modules are executed with the `top` command, and
each module can be started/stopped individually via `<module_name> start/stop`.

The arrows show the information flow for the *most important* connections between
the modules. In reality, there are many more connections than shown, and some messages 
(e.g. for parameters) are accessed by most of the modules.

Modules communicate with each other through a publish-subscribe message bus
named [uORB](../middleware/uorb.md). 
The use of the publish-subscribe scheme means that:

- The system is reactive — it will update instantly when new data is available
- All operations and communication are fully parallelized
- A system component can consume data from anywhere in a thread-safe fashion

> **Info** This architecture allows every single one of these
> blocks to be rapidly and easily replaced, even at runtime.

The controllers / mixers are specific to a particular airframe (e.g. a
multicopter, VTOL or plane), but the higher-level mission management blocks like
the `commander` and `navigator` are shared between platforms.

### Middleware {#middleware}

The middleware consists primarily of device drivers
for embedded sensors, communication with the external world (companion computer,
GCS, etc.) and a publish-subscribe message bus.

### Flight Stack {#flight-stack}

The flight stack is a collection of guidance, navigation and control algorithms 
for autonomous drones. 
It includes controllers for fixed wing, multirotor and VTOL airframes 
as well as estimators for attitude and position.

## Update Rates

Since the modules poll for message updates, typically the drivers define how
fast a module updates. Most of the IMU drivers sample the data at 1kHz,
integrate it and publish with 250Hz. Other parts of the system, such
as the `navigator`, don't need such a high update rate, and thus run
considerably slower.

The message update rates can be inspected in real-time on the system by running
`uorb top`.

## Runtime Environment

PX4 runs on various operating systems that provide a POSIX-API and execution
model (such as Linux, MacOS, NuttX or QuRT).

The inter-module communication (using uORB) is based on shared memory. The whole
PX4 middleware runs in a single address space, i.e. memory is shared between all
modules. 

> **Info** The system is designed such that with minimal effort it would
> be possible to run each module in separate address space (parts that would need
> to be changed include `uORB`, `parameter interface`, `dataman` and `perf`).

There are 2 different ways that a module can be executed:
- **Tasks**: this is the more common way. A module runs in its own task with its
  own stack and process priority.
- **Work queues**: the module runs on a shared task, meaning it does not own a
  stack. Multiple tasks run on the same stack with a single priority per work
  queue.
  
A task is scheduled by specifying a fixed time in the future.
The advantage is that it uses less RAM, but the task is not allowed to sleep
or poll on a message.

It is used for periodic tasks, such as sensor drivers or the land detector.

> **Note** Tasks running on a work queue do not show up in `top` 
> (only the work queues themselves can be seen - e.g. as `lpwork`).

The following sections provide additional OS-specific information.

### NuttX

[NuttX](http://nuttx.org/) is the primary RTOS for running PX4 on a flight-control
board. It is open source (BSD license), light-weight, efficient and very stable.

Modules are executed as tasks: they have their own file descriptor lists, but
they share a single address space. A task can still start one or more threads
that share the file descriptor list.

Each task/thread has a fixed-size stack, and there is a periodic task which
checks that all stacks have enough free space left (based on stack coloring).

### Linux/Mac

On Linux or macOS, PX4 runs in a single process, and the modules run in their own
threads (there is no distinction between tasks and threads as on NuttX).
