# uORB Messaging

## Introduction

The uORB is an asynchronous publish() / subscribe() messaging API used for
inter-thread/inter-process communication.

Look at the [tutorial](tutorial-hello-sky.md) to learn how to use it in C++.

uORB is automatically started early on bootup as many applications depend on it.
It is started with `uorb start`. Unit tests can be started with `uorb test`.

## Adding a new topic

To add a new topic, you need to create a new `.msg` file in the `msg/`
directory and add the file name to the `msg/CMakeLists.txt` list. From this,
there will automatically be C/C++ code generated.

Have a look at the existing `msg` files for supported types. A message can also
be used nested in other messages.
To each generated C/C++ struct, a field `uint64_t timestamp` will be added. This
is used for the logger, so make sure to fill it in when logging the message.

To use the topic in the code, include the header:
```
#include <uORB/topics/topic_name.h>
```

By adding a line like the following in the `.msg` file, a single message
definition can be used for multiple independent topic instances:
```
# TOPICS mission offboard_mission onboard_mission
```
Then in the code, use them as topic id: `ORB_ID(offboard_mission)`.


## Publishing

Publishing a topic can be done from anywhere in the system, including interrupt context (functions called by the `hrt_call` API). However, advertising a topic is only possible outside of interrupt context. A topic has to be advertised in the same process as its later published.

## Listing Topics and Listening in

<aside class="note">
The 'listener' command is only available on Pixracer (FMUv4) and Linux / OS X.
</aside>

To list all topics, list the file handles:

```sh
ls /obj
```

To listen to the content of one topic for 5 messages, run the listener:

```sh
listener sensor_accel 5
```

The output is n-times the content of the topic:

```sh
TOPIC: sensor_accel #3
timestamp: 84978861
integral_dt: 4044
error_count: 0
x: -1
y: 2
z: 100
x_integral: -0
y_integral: 0
z_integral: 0
temperature: 46
range_m_s2: 78
scaling: 0

TOPIC: sensor_accel #4
timestamp: 85010833
integral_dt: 3980
error_count: 0
x: -1
y: 2
z: 100
x_integral: -0
y_integral: 0
z_integral: 0
temperature: 46
range_m_s2: 78
scaling: 0
```
