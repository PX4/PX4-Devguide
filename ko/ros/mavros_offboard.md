!REDIRECT "https://docs.px4.io/master/ko/ros/mavros_offboard.html"

# MAVROS *보드 외부* 제어 예제

이 자습서에서는 가제보/SITL에서 모의시험을 진행할 쿼드콥터로 MAVROS의 *보드 외부* 제어 기본을 설명하도록 하겠습니다. 자습서의 마지막 부분에서는 아래 동영상에서 나타난 2미터 고도에서의 저속 이륙과 같은 동일한 동작이 나타나야 합니다.

> **Caution** *보드 외부* 제어는 위험합니다. 실제 기체를 운용할 때는 어떤 동작에 문제가 있을 경우를 감안하여 수동 조작으로 전환하는 방안을 강구하십시오.

<span></span>

> **Tip** 이 예제는 C++ 언어를 사용합니다. Similar examples in Python can be found here: [integrationtests/python_src/px4_it/mavros](https://github.com/PX4/PX4-Autopilot/tree/master/integrationtests/python_src/px4_it/mavros).

<video width="100%" autoplay="true" controls="true">
    <source src="../../assets/simulation/gazebo_offboard.webm" type="video/webm">
</video>

## 코드

ROS 패키지에 `offb_node.cpp` 파일을 만들고(컴파일 할 수 있게끔 `CMakeList.txt`에 파일을 추가), 다음 코드를 파일에 붙여넣으십시오:

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

## 코드 설명

```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
```

`mavros_msgs` 패키지에는 서비스와 MAVROS 패키지에서 제공하는 토픽 운용에 필요한 모든 개별 메세지가 들어있습니다. 모든 서비스와 토픽에 해당하는 메세지 형식은 [mavros 위키](http://wiki.ros.org/mavros)에 있습니다.

```cpp
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
```

오토파일럿의 현재 상태를 저장하는 간단한 콜백 메서드를 만들었습니다. 이 콜백 메서드로 연결, 이륙 준비 상태, *OFFBOARD* 플래그를 확인합니다.

```cpp
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```

명령에 따른 로컬 위치를 내보내는 전송자와, 이륙 준비와 모드 변경을 요청하는 적당한 클라이언트를 초기화합니다. 우리 시스템에서 "mavros" 접두부는 실행 파일에서 노드에 주어진 이름에 따라 달라질 수 있습니다.

```cpp
//the setpoint publishing rate MUST be faster than 2Hz
ros::Rate rate(20.0);
```

PX4는 *보드 외부*에 인가하는 명령 두개 사이에 500ms의 제한 시간을 둡니다. 제한 시간이 지나면, 통제 주체는 *보드 외부* 통제 모드로 들어가기 전 기체의 최근 상태로 복귀합니다. 이게 바로 가능한 지연 시간을 고려하여 메세지와 명령을 내보내는 속도가 2Hz보다 **빨라야 하는** 이유입니다. 또한 *위치* 통제 모드에서 *보드 외부* 통제 모드로의 진입을 추천하는 동일한 이유이기도 하며, *보드 외부* 통제 모드 진입을 기체에서 중단하면 추적을 멈추고 그 자리에서 떠 있습니다.

```cpp
// wait for FCU connection
while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
```

무언가를 전송하기 전 MAVROS와 오토파일럿의 연결을 기다립니다. 이 루프는 하트비트 메세지를 수신한 즉시 빠져나갑니다.

```cpp
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
```

PX4 프로 플라이트 스택이 항공 NED 좌표 영역에서 동작하긴 하지만, MAVROS는 이 좌표를 표준 ENU 프레임 또는 그 반대로 (자유 자재로) 변환합니다. 따라서 `z` 값을 양의 정수 2로 설정했습니다.

```cpp
//send a few setpoints before starting
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}
```

*보드 외부* 제어 모드로 들어가기 전에 실시간 데이터 전송 설정값을 둔 상태로 시작해야합니다. 그렇지 않으면 모드 전환이 불가능합니다. 여기서는 `100`을 임의의 값으로 선택했습니다.

```cpp
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
```

`OFFBOARD`를 custom_mode 값으로 설정했습니다. [지원 모드](http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack) 목록을 참고 목적으로 두었습니다.

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

나머지 코드는 굳이 설명하지 않아도 있는 그대로 동작합니다. 쿼드콥터 비행을 할 수 있도록 이륙 준비가 끝나고 나면, *OFFBOARD* 모드로 전환을 시도합니다. 5초간의 서비스 호출 여유 시간을 두어 오토 파일럿에 요청이 과도하게 들어가지 않게 합니다. 동일한 루프에서 적당한 속도로 요청한 자세를 계속 보냅니다.

> **Tip** 이 코드는 최소한의 묘사를 목적으로 단순화했습니다. 좀 더 규모가 큰 시스템에서는 설정값을 주기적으로 내보내도록 새 스레드를 만드는게 좋습니다.