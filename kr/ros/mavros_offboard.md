# MAVROS 오프보드 제어 예제

> **Caution** 오프보드는 위험합니다. 실제 비행체에서 운영한다면 문제가 발생하는 경우에 대해서 수동 제어로 돌릴 수 있는 방법이 있는지를 확인하세요.

다음 튜토리얼은 Gazebo에서 Iris 쿼드로터를 시뮬레이션하는데 사용한 mavros를 통해 오프보드 제어의 기본 내용을 알아봅니다. 튜토리얼의 마지막에는 아래 비디오에서 보는 바와 같은 동작을 보게 됩니다. (2미터 고도로 천천히 이륙)

<video width="100%" autoplay="true" controls="true">
	<source src="../../assets/simulation/gazebo_offboard.webm" type="video/webm">
</video>

## Code
ros 패키지에서 offb_node.cpp 파일 생성하고 다음을 내부에 붙여넣기:

```cpp
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

## Code 설명

```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
```

`mavros_msgs` 패키지는 mavros 패키지가 제공하는 서비스와 topic을 운영하는데 필요한 모든 커스텀 메시지를 포함하고 있습니다. 관련된 메시지 타입뿐만 아니라 모든 서비스와 topic들에 관한 자료를 [mavros wiki](http://wiki.ros.org/mavros)에서 참고할 수 있습니다.

```cpp
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
```

autopilot의 현재 상태를 저장하기 위한 간단한 콜백을 생성합니다. 이것을 통해 연결, armimg, 오프보드 flag를 검사할 수 있습니다.

```cpp
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```

local position을 publish하기 위해서 publisher 인스턴스를 만들고 적합한 client에게 arming과 모드 변경을 요청합니다. 여러분의 시스템에서 "mavros" 접두사는 다른 의미일 수 있습니다. launch 파일내부에서 node에 주어진 이름에 의존하기 때문입니다.

```cpp
//the setpoint publishing rate MUST be faster than 2Hz
ros::Rate rate(20.0);
```
px4 flight stack은 2개 오프보드 명령 사이에 500ms 타임아웃이 있습니다. 타임아웃 시간을 초과하면, commander는 비행체가 오프보드 모드로 들어가기 전에 있던 마지막 모드로 되돌아갑니다. 지연시간을 고려해서 publishing rate는 **반드시** 2 Hz보다 빨라야만 합니다. POSCTL에서 오프보드 모드로 진입하는 것을 추천하는 것도 같은 이유입니다. 비행체가 오프보드 모드를 빠져나오면 멈춰서 hover 상태가 되기 떄문입니다.

```cpp
// wait for FCU connection
while(ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
```

어떤 것을 publishing하기 전에, mavros와 autopilot 사이에 연결이 되기까지 기다립니다. heartbeat 메시지를 받자마자 이 loop를 빠져나와야 합니다.

```cpp
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
```

비록 px4 flight stack이 대기 NED 좌표 프레임에서 운영되므로 mavros는 이 좌표변환을 표준 ENU 프레임으로 변환합니다. 반대로도 변환이 가능합니다. 이것이 z를 +2로 설정하는 이유입니다.

```cpp
//send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}
```
오프보드 모드로 진입하기 전에, 이미 스트리밍 setpoint를 구동시켰어야 합니다. 그렇지 않으면 모드 스위치가 동작하지 않습니다. 여기에서는 임의의 값으로 100을 취했습니다.
```cpp
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
```
커스텀 모드를 `OFFBOARD`로 설정. [지원 모드](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack) 목록을 참고하세요.

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
코드의 나머지 부분은 코드내부에 설명이 잘되어 있습니다. 비행을 위해 쿼드를 arm시킨 후에 오프보드 모드로 전환해 봅니다. 요청이 바로 autopilot에 먹지 않도록 서비스 호출까지 5초까지 여유를 둡니다. 동일 loop에서 적절한 rate로 요청한 pose를 보내기 시작합니다.

> **Tip** 이 코드는 이해를 목적으로 단순화 시켰습니다. 더 큰 시스템에서 주기적으로 setpoint를 publish하는 책임을 지는 새로운 thread를 생성하면 유용하다.
