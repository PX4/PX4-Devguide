# MAVLink Messaging
An overview of all messages can be found [here](http://mavlink.org/messages/common).
## Create Custom MAVLink Messages
This tutorial assumes you have a [custom uORB](advanced-uorb.md) `ca_trajectory`
message in `msg/ca_trajectory.msg` and a custom mavlink
`ca_trajectory` message in
`mavlink/include/mavlink/v1.0/custom_messages/mavlink_msg_ca_trajectory.h` (see
[here](http://qgroundcontrol.org/mavlink/create_new_mavlink_message) how to
create a custom mavlink message and header).

## Sending Custom MAVLink Messages
This section explains how to use a custom uORB message and send it as a mavlink
message.

Add the headers of the mavlink and uorb messages to
[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)

```C
#include <uORB/topics/ca_trajectory.h>
#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

Create a new class in [mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp#L2193)

```C
class MavlinkStreamCaTrajectory : public MavlinkStream
{
public:
    const char *get_name() const
	{
		return MavlinkStreamCaTrajectory::get_name_static();
	}
	static const char *get_name_static()
	{
		return "CA_TRAJECTORY";
	}
	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_CA_TRAJECTORY;
	}
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCaTrajectory(mavlink);
	}
	unsigned get_size()
	{
		return MAVLINK_MSG_ID_CA_TRAJECTORY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_sub;
	uint64_t _ca_traj_time;

	/* do not allow top copying this class */
	MavlinkStreamCaTrajectory(MavlinkStreamCaTrajectory &);
	MavlinkStreamCaTrajectory& operator = (const MavlinkStreamCaTrajectory &);

protected:
	explicit MavlinkStreamCaTrajectory(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sub(_mavlink->add_orb_subscription(ORB_ID(ca_trajectory))),  // make sure you enter the name of your uorb topic here
		_ca_traj_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct ca_traj_struct_s _ca_trajectory;    //make sure ca_traj_struct_s is the definition of your uorb topic

		if (_sub->update(&_ca_traj_time, &_ca_trajectory)) {
			mavlink_ca_trajectory_t _msg_ca_trajectory;  //make sure mavlink_ca_trajectory_t is the definition of your custom mavlink message

			_msg_ca_trajectory.timestamp = _ca_trajectory.timestamp;
			_msg_ca_trajectory.time_start_usec = _ca_trajectory.time_start_usec;
			_msg_ca_trajectory.time_stop_usec  = _ca_trajectory.time_stop_usec;
			_msg_ca_trajectory.coefficients =_ca_trajectory.coefficients;
			_msg_ca_trajectory.seq_id = _ca_trajectory.seq_id;

			_mavlink->send_message(MAVLINK_MSG_ID_CA_TRAJECTORY, &_msg_ca_trajectory);
		}
	}
};
```

Finally append the stream class to the `streams_list` at the bottom of
[mavlink_messages.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_messages.cpp)

```C
StreamListItem *streams_list[] = {
...
new StreamListItem(&MavlinkStreamCaTrajectory::new_instance, &MavlinkStreamCaTrajectory::get_name_static),
nullptr
};
```

Then make sure to enable the stream, for example by adding the following line to
the startup script (`-r` configures the streaming rate, `-u` identifies the
mavlink channel on UDP port 14556):

```
mavlink stream -r 50 -s CA_TRAJECTORY -u 14556
```


## Receiving Custom MAVLink Messages
This section explains how to receive a message over mavlink and publish it to
uORB.

Add a function that handles the incoming mavlink message in
[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L77)

```C
#include <uORB/topics/ca_trajectory.h>
#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>
```

Add a function that handles the incoming mavlink message in the
`MavlinkReceiver` class in
[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L140)

```C
void handle_message_ca_trajectory_msg(mavlink_message_t *msg);
```
Add an uORB publisher in the `MavlinkReceiver` class in
[mavlink_receiver.h](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.h#L195)

```C
orb_advert_t _ca_traj_msg_pub;
```

Implement the `handle_message_ca_trajectory_msg` function in [mavlink_receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp)

```C
void
MavlinkReceiver::handle_message_ca_trajectory_msg(mavlink_message_t *msg)
{
	mavlink_ca_trajectory_t traj;
	mavlink_msg_ca_trajectory_decode(msg, &traj);

	struct ca_traj_struct_s f;
	memset(&f, 0, sizeof(f));

	f.timestamp = hrt_absolute_time();
	f.seq_id = traj.seq_id;
	f.time_start_usec = traj.time_start_usec;
	f.time_stop_usec = traj.time_stop_usec;
	for(int i=0;i<28;i++)
		f.coefficients[i] = traj.coefficients[i];

	if (_ca_traj_msg_pub == nullptr) {
		_ca_traj_msg_pub = orb_advertise(ORB_ID(ca_trajectory), &f);

	} else {
		orb_publish(ORB_ID(ca_trajectory), _ca_traj_msg_pub, &f);
	}
}
```

and finally make sure it is called in [MavlinkReceiver::handle_message()](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp#L228)

```C
MavlinkReceiver::handle_message(mavlink_message_t *msg)
 {
 	switch (msg->msgid) {
        ...
	case MAVLINK_MSG_ID_CA_TRAJECTORY:
		handle_message_ca_trajectory_msg(msg);
		break;
		...
 	}
```
## General
### Set streaming rate
Sometimes it is useful to increase the streaming rate of individual topics (e.g. for inspection in QGC). This can be achieved by the following line
```sh
mavlink stream -u <port number> -s <mavlink topic name> -r <rate>
```
You can get the port number with ```mavlink status``` which will output (amongst others) ```transport protocol: UDP (<port number>)```. An example would be
```sh
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 300
```
