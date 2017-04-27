# MAVROS外部控制例程

> **注意：** 外部控制是危险的。如果在真机上操作，确保可以在出错的时候切换回手动控制。


下面的教程是基本的外部控制，通过MAVROS应用在Gazebo模拟的Iris四旋翼上。在教程最后，应该会得到与下面视频相同的结果，即缓慢起飞到高度2米。

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

`mavros_msgs`包含有所有用于MAVROS服务和主题的自定义消息。所有服务和主题以及它们所对应的消息类型参照文档[mavros wiki](http://wiki.ros.org/mavros)。

```C++
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
```

创建一个简单的回调函数，它可以保存飞控的当前状态。我们可以用它检查连接状态，解锁状态以及外部控制标志。

```C++
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```

我们实例化一个用来发布指令位置的发布器，一个请求解锁的客户端和一个请求改变模式的客户端。注意，对你自己的系统，根据启动文件中节点名字的不同，"mavros"前面的部分会有所不同。

```C++
//the setpoint publishing rate MUST be faster than 2Hz
ros::Rate rate(20.0);
```

px4飞行栈在外部控制指令之间有500ms的时限，如果超过了时限，那么飞控将会切换回进入外部控制模式之前的模式。这正是考虑可能的延迟，发布频率**必须**高于2Hz的原因。这同样也是推荐从位置控制模式进入外部控制模式的原因，如果外部控制模式发生故障，飞行器将会停止动作并处于盘旋状态。

```C++
// wait for FCU connection
while(ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
```

在发布之前，需要等待MAVROS和飞控建立连接。一旦接收到心跳包，该循环就会立即退出。

```C++
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
```

即使px4飞行栈工作在航空常用的NED坐标系，MAVROS仍然会将这些坐标转换到标准的ENU坐标系，反之亦然。这是我们将Z设置为+2的原因。

```C++
//send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}
```

在进入外部控制模式之前，就必须开始发布指令，否则模式切换会被拒绝。这里，100是个随意选取的值。

```C++
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
```

设置自定义模式为`OFFBOARD`，参考支持的[模式列表](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack)

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

剩下的代码比较好理解。在解锁并起飞后，不断地请求切换至外部控制模式。在请求之间间隔5秒，不至于让飞控响应不过来。在同样的循环里，以合适的频率持续发送位姿指令。

> **提示:** 出于解释的目的，这份代码经过了简化。在更大的系统中，创建一个新的负责周期性发送目标指令的线程往往更加有用。
