# Macros功能包的offboard模式控制例程

> ** 警告 **使用* Offboard * 模式来控制无人机有危险性的。 如果你是在一个真正的实物平台上进行试验，请保证你已经设置了切换回手动的开关来防止紧急情况的发生。

下面的教程将利用 * Offboard * 控制的基本操作, 通过 MAVROS 在Gazebo的仿真环境来控制一个Iris 无人机。 在教程的末尾, 你应该看到与下面的视频相同的行为, 即缓慢起飞到2米的高度。

<video width="100%" autoplay="true" controls="true">
    <source src="../../assets/simulation/gazebo_offboard.webm" type="video/webm">
</video>

## 代码

首先，在你的MAVROS包中创建`offb_node.cpp`文件（并且添加到对应的`CMakeList.txt`中去编译它），然后复制粘贴以下内容到文件中：

```cpp
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
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
    while(ros::ok() && !current_state.connected){
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
                offb_set_mode.response.mode_sent){
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

## 代码解释

```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
```

` mavros_msgs ` 功能包中包含操作 mavros 包中服务和主题所需的自定义消息文件。 所有服务和主题及其相应的消息类型都可以在 [ mavros wiki ](http://wiki.ros.org/mavros) 中找到。

```cpp
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
```

我们创建了一个简单的回调函数来储存飞控当前的状态。 这将使得我们可以检查飞机的各项状态，如是否连接上mavros功能包、是否解锁、当前飞行模式。

```cpp
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```

We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change. Note that for your own system, the "mavros" prefix might be different as it will depend on the name given to the node in it's launch file.

```cpp
//the setpoint publishing rate MUST be faster than 2Hz
ros::Rate rate(20.0);
```

The px4 flight stack has a timeout of 500ms between two *Offboard* commands. If this timeout is exceeded, the commander will fall back to the last mode the vehicle was in before entering *Offboard* mode. This is why the publishing rate **must** be faster than 2 Hz to also account for possible latencies. This is also the same reason why it is recommended to enter *Offboard* mode from *Position* mode, this way if the vehicle drops out of *Offboard* mode it will stop in its tracks and hover.

```cpp
// wait for FCU connection
while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
```

Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot. This loop should exit as soon as a heartbeat message is received.

```cpp
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
```

Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, MAVROS translates these coordinates to the standard ENU frame and vice-versa. This is why we set `z` to positive 2.

```cpp
//send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}
```

Before entering *Offboard* mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected. Here, `100` was chosen as an arbitrary amount.

```cpp
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
```

We set the custom mode to `OFFBOARD`. A list of [supported modes](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack) is available for reference.

```cpp
mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;

ros::Time last_request = ros::Time::now();

while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent){
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

The rest of the code is pretty self explanatory. We attempt to switch to *Offboard* mode, after which we arm the quad to allow it to fly. We space out the service calls by 5 seconds so to not flood the autopilot with the requests. In the same loop, we continue sending the requested pose at the appropriate rate.

> **Tip** This code has been simplified to the bare minimum for illustration purposes. In larger systems, it is often useful to create a new thread which will be in charge of periodically publishing the setpoints.