# Send Debug String / Float Pairs

It is often necessary during software development to output individual important numbers.
This is where the generic `NAMED_VALUE` packets of MAVLink come in.

## Files

The code for this tutorial is available here:

  * [Debug Tutorial Code](https://github.com/PX4/Firmware/blob/master/src/examples/px4_mavlink_debug/px4_mavlink_debug.c)
  * [Enable the tutorial app](https://github.com/PX4/Firmware/tree/master/cmake/configs) by uncommenting / enabling the mavlink debug app in the config of your board

All required to set up a debug publication is this code snippet. First add the header file:

<div class="host-code"></div>

```C
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
```

Then advertise the debug value topic (one advertisement for different published names is sufficient). Put this in front of your main loop:

<div class="host-code"></div>

```C
/* advertise debug value */
struct debug_key_value_s dbg = { .key = "velx", .value = 0.0f };
orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
```

And sending in the main loop is even simpler:

<div class="host-code"></div>

```C
dbg.value = position[0];
orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
```

> **Caution** Multiple debug messages must have enough time between their respective publishings for Mavlink to process them. This means that either the code must wait between publishing multiple debug messages, or alternate the messages on each function call iteration.

The result in QGroundControl then looks like this on the real-time plot:

![](../../assets/gcs/qgc-debugval-plot.jpg)
