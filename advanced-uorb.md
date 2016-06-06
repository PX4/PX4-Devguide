# uORB Messaging

## Introduction

The uORB is a publish() / subcribe() messaging API.

<aside class="todo">
The current documentation is still here for [uORB](https://pixhawk.org/dev/shared_object_communication) and this [page](https://pixhawk.org/dev/add_uorb_topic) covers how to add a new topic.
</aside>

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
