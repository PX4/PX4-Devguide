---
translated_page: https://github.com/PX4/Devguide/blob/master/en/ros/mavros_offboard.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# MAVROS机外（offboard）控制例程

> **注意：** 机外控制非常危险。如果在真机上操作，请确保可以在出错的时候切回手动控制。


下面的教程是一个基础的机外控制例子，通过MAVROS在Gazebo中应用于Iris四旋翼上。在教程最后，你应该会得到与下面视频相同的结果，即无人机缓慢起飞到高度2米。

<video width="100%" autoplay="true" controls="true">
	<source src="../assets/sim/gazebo_offboard.webm" type="video/webm">
</video>

## 代码

在ROS包中创建offb_node.cpp文件，并粘贴下面内容：

```C++
/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```

> **提示：** 本过程需要对ROS有一定的了解。
> 创建工作空间后需要`source devel/setup.bash`，否则会出现找不到package的情况，要想保证工作空间已配置正确需确保ROS_PACKAGE_PATH环境变量包含你的工作空间目录，采用`echo $ROS_PACKAGE_PATH`命令查看是否包含了你创建的package的路径，此操作也可以通过直接在.bashrc文件最后添加路径的方式解决。


## 代码解释

```C++
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
```

`mavros_msgs`包含MAVROS包中提供的服务（service）和话题（topic）所需的一切自定义消息。所有服务和话题以及相应的消息类型可参照文档[mavros wiki](http://wiki.ros.org/mavros)。

```C++
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
```

我们创建一个简单的回调函数来保存飞控的当前状态。我们可以用它检查连接状态，解锁状态以及外部控制标志。

```C++
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```

我们实例化一个用来发布被控制的本地位置的发布器，以及适当的客户端来请求解锁和模式更改。注意，对你自己的系统，"mavros"前缀部分会有所不同，它依赖于对应节点的launch文件中定义的名字。

```C++
//the setpoint publishing rate MUST be faster than 2Hz
ros::Rate rate(20.0);
```

px4飞行栈的两个机外（offboard）控制指令之间有500ms的时限。如果超过了时限，commander指令将会切换回进入机外控制模式前的上一个模式。这正是为什么发布频率**必须**高于2Hz的原因，并且还要考虑可能的延迟。这也是我们推荐从位置控制（POSCTL）模式进入机外控制模式的原因。这样一来，如果飞机意外脱离了机外控制模式，飞机将会停在当前轨道并悬停。

```C++
// wait for FCU connection
while(ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
```

在发布任何东西之前，我们需要等待MAVROS和飞控建立连接。一旦接收到心跳消息[heartbeat message](https://en.wikipedia.org/wiki/Heartbeat_message)，该循环就会立即退出。以上代码是以一定频率（20Hz）来执行ROS消息回调函数，即[ros::spinOnce()](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning).

```C++
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
```

即使px4飞行栈在航空[NED](https://en.wikipedia.org/wiki/North_east_down)坐标系中运行，MAVROS仍然会将这些坐标转换到标准的ENU坐标系，反之亦然。这是我们将Z设置为+2的原因。

```C++
//send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}
```

在进入机外控制模式之前，就必须开始发送设定值（这里是指pose），否则模式切换会被拒绝。这里的100是一个随意选取的值。

```C++
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
```

设置自定义模式为`OFFBOARD`。PX4飞行栈所支持的飞行模式可参考[这里](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack)

```C++
mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;

ros::Time last_request = ros::Time::now();

while(ros::ok()){
		if( current_state.mode != "OFFBOARD" &&
				(ros::Time::now() - last_request > ros::Duration(5.0))){
				if( set_mode_client.call(offb_set_mode) &&
						offb_set_mode.response.success){
						ROS_INFO("Offboard enabled");
				}
				last_request = ros::Time::now();
		} else {
				if( !current_state.armed &&
						(ros::Time::now() - last_request > ros::Duration(5.0))){
						if( arming_client.call(arm_cmd) &&
								arm_cmd.response.success){
								ROS_INFO("Vehicle armed");
						}
						last_request = ros::Time::now();
				}
		}

		local_pos_pub.publish(pose);

		ros::spinOnce();
		rate.sleep();
}
```

剩下的代码比较好理解。我们试图在解锁旋翼允许它起飞后，将它切换至机外控制模式。为了避免大量请求堵塞飞控，我们设置服务调用间隔时间为5秒。在同一个循环里，继续以合适的频率持续发布设定的pose。

> **提示:** 为了便于说明，该代码已被简化。在较大的系统中，往往会创建一个新的线程用来周期性地发布设定值。
