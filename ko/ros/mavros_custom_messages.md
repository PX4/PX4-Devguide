# MAVROS에서 PX4로 커스텀 메시지 전송

> **경고** 본 문서는 아래 환경에서 테스트하였습니다:

- **Ubuntu:** 18.04
- **ROS:** Melodic
- **PX4 Firmware:** 1.9.0
    
    그렇지만 아래 절차는 아주 일반적이기 때문에 다른 배포판이나 버전에서 약간의 수정이나 수정 없이 동작할 것입니다. 

<!-- Content reproduced with permission from @JoonmoAhn in https://github.com/JoonmoAhn/Sending-Custom-Message-from-MAVROS-to-PX4/issues/1 -->

## MAVROS 설치

[mavlink/mavros](https://github.com/mavlink/mavros/blob/master/mavros/README.md) 의 *소프트웨어 설치* 안내에 따라 "ROS Kinetic" 설치를 따라하십시오.

## MAVROS

1. **keyboard_command.cpp** (**workspace/src/mavros/mavros_extras/src/plugins**에 있음) 예제에서, 우선 아래의 코드를 이용하여 신규 MAVROS 플러그인을 만듭니다:
    
    다음 코드는 ROS 토픽 `/mavros/keyboard_command/keyboard_sub`의 'char' 메시지를 구독하고 이를 MAVLink 메시지로 전송합니다.

   ```c
    #include <mavros/mavros_plugin.h>
    #include <pluginlib/class_list_macros.h>
    #include <iostream>
    #include <std_msgs/Char.h>

    namespace mavros {
    namespace extra_plugins{

    class KeyboardCommandPlugin : public plugin::PluginBase {
    public:
        KeyboardCommandPlugin() : PluginBase(),
            nh("~keyboard_command")

       { };

        void initialize(UAS &uas_)
        {
            PluginBase::initialize(uas_);
            keyboard_sub = nh.subscribe("keyboard_sub", 10, &KeyboardCommandPlugin::keyboard_cb, this);
        };

        Subscriptions get_subscriptions()
        {
            return {/* RX disabled */ };
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber keyboard_sub;

       void keyboard_cb(const std_msgs::Char::ConstPtr &req)
        {
            std::cout << "Got Char : " << req->data <<  std::endl;
            UAS_FCU(m_uas)->send_message_ignore_drop(req->data);
        }
    };
    }   // namespace extra_plugins
    }   // namespace mavros

   PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::KeyboardCommandPlugin, mavros::plugin::PluginBase)
   ```

1. **mavros_plugins.xml** (**workspace/src/mavros/mavros_extras**에 있음)에 다음 행을 추가합니다:

   ```xml
   <class name="keyboard_command" type="mavros::extra_plugins::KeyboardCommandPlugin" base_class_type="mavros::plugin::PluginBase">
        <description>Accepts keyboard command.</description>
   </class>
   ```

1. **CMakeLists.txt** (**workspace/src/mavros/mavros_extras**에 있음)에 아래 `add_library`에 아래 행을 추가합니다.

   ```cmake
   add_library( 
   ...
     src/plugins/keyboard_command.cpp 
   )
   ```

1. **common.xml** (**workspace/src/mavlink/message_definitions/v1.0**에 있음)에 다음 행을 복사하여 여러분의 MAVLink 메시지를 추가합니다:

   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description>Keyboard char command.</description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

## PX4 수정사항

1. **common.xml** (**Firmware/mavlink/include/mavlink/v2.0/message_definitions**에 있음)에 다음과 같이 여러분의 MAVLink 메시지를 추가합니다.(위의 MAVROS와 동일):

   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description>Keyboard char command.</description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

1. *common*, *standard* (**Firmware/mavlink/include/mavlink/v2.0**에 있음) 디렉토리 삭제.

   ```sh
   rm -r common
   rm -r standard
   ```

1. 원하는 디렉터리로 "mavlink_generator"를 git 클론 후 실행.

   ```sh
   git clone https://github.com/mavlink/mavlink mavlink-generator
   cd mavlink-generator
   python mavgenerate.py
   ```

1. "MAVLink Generator" 팝업이 나타납니다:
    
    - *XML*은 **/Firmware/mavlink/include/mavlink/v2.0/message_definitions/standard.xml**로 "Browse"합니다.
    - Out은 **/Firmware/mavlink/include/mavlink/v2.0/**로 "Browse"합니다.
    - **C** 언어를 선택
    - 프로토콜 **2.0**을 선택
    - *Validate* 체크
    
    **Generate**를 누릅니다. **/Firmware/mavlink/include/mavlink/v2.0/**아래에 *common* 및 *standard* 디렉토리가 생성됩니다.

2. (Firmware/msg)에 **key_command.msg**라는 uORB 메시지 파일을 작성합니다. 이번 예제에서는 "key_command.msg" 파일에는 아래의 코드만 있습니다:

   ```
   char cmd
   ```

다음으로 **CMakeLists.txt**(**Firmware/msg**에 있음)에 아래를 추가합니다.

   ```cmake
   set(
   ...
        key_command.msg
        )
   ```

1. **mavlink_receiver.h** (**Firmware/src/modules/mavlink**에 있음) 편집

   ```cpp
   ...
   #include <uORB/topics/key_command.h>
   ...
   class MavlinkReceiver
   {
   ...
   private:
       void handle_message_key_command(mavlink_message_t *msg);
   ...
       orb_advert_t _key_command_pub{nullptr};
   }
   ```

1. **mavlink_receiver.cpp** (**Firmware/src/modules/mavlink**에 있음) 편집. 여기에서 PX4는 ROS에서 전송된 MAVLink 메시지를 수신하고, 이를 uORB 토픽으로 발행합니다.

   ```cpp
   ...
   void MavlinkReceiver::handle_message(mavlink_message_t *msg)
   {
   ...
    case MAVLINK_MSG_ID_KEY_COMMAND:
           handle_message_key_command(msg);
           break;
   ...
   }
   ...
   void
   MavlinkReceiver::handle_message_key_command(mavlink_message_t *msg)
   {
       mavlink_key_command_t man;
       mavlink_msg_key_command_decode(msg, &man);

   struct key_command_s key = {};

       key.timestamp = hrt_absolute_time();
       key.cmd = man.command;

       if (_key_command_pub == nullptr) {
           _key_command_pub = orb_advertise(ORB_ID(key_command), &key);

       } else {
           orb_publish(ORB_ID(key_command), _key_command_pub, &key);
       }
   }
   ```

1. 다른 subscriber 모듈예저처럼 여러분의 uORB 토픽 subscriber를 작성합니다. (/Firmware/src/modules/key_receiver)에 모델을 생성합니다. 이 디렉토리에, **CMakeLists.txt** 및 **key_receiver.cpp** 두개의 파일을 생성합니다. 개별 파일은 다음과 같습니다.
    
    -CMakeLists.txt

   ```cmake
   px4_add_module(
       MODULE modules__key_receiver
       MAIN key_receiver
       STACK_MAIN 2500
       STACK_MAX 4000
       SRCS
           key_receiver.cpp
       DEPENDS
           platforms__common

       )
   ```

-key_receiver.cpp

   ```
   #include <px4_config.h>
   #include <px4_tasks.h>
   #include <px4_posix.h>
   #include <unistd.h>
   #include <stdio.h>
   #include <poll.h>
   #include <string.h>
   #include <math.h>

   #include <uORB/uORB.h>
   #include <uORB/topics/key_command.h>

   extern "C" __EXPORT int key_receiver_main(int argc, char **argv);

   int key_receiver_main(int argc, char **argv)
   {
       int key_sub_fd = orb_subscribe(ORB_ID(key_command));
       orb_set_interval(key_sub_fd, 200); // limit the update rate to 200ms

       px4_pollfd_struct_t fds[1];
       fds[0].fd = key_sub_fd, fds[0].events = POLLIN;

       int error_counter = 0;

       while(true)
       {
           int poll_ret = px4_poll(fds, 1, 1000);

           if (poll_ret == 0)
           {
               PX4_ERR("Got no data within a second");
           }

           else if (poll_ret < 0)
           {
               if (error_counter < 10 || error_counter % 50 == 0)
               {
                   PX4_ERR("ERROR return value from poll(): %d", poll_ret);
               }

               error_counter++;
           }

           else
           {
               if (fds[0].revents & POLLIN)
               {
                   struct key_command_s input;
                   orb_copy(ORB_ID(key_command), key_sub_fd, &input);
                   PX4_INFO("Recieved Char : %c", input.cmd);
                }
           }
       }
       return 0;
   }
   ```

상세한 설명은 [Writing your first application](https://dev.px4.io/en/apps/hello_sky.html) 문서를 참고하십시오.

1. 마지막으로 **Firmware/boards/**에서 여러분의 보드에 해댕하는 **default.cmake**파일에 여러분의 모듈을 추가합니다. 예를 들면, Pixhawk 4의 경우에는 **Firmware/boards/px4/fmu-v5/default.cmake**에 다음 코드를 추가합니다:

   ```cmake
    MODULES
        ...
        key_receiver
        ...
    ```

이제 여러분의 작업을 빌드할 준비가 되었습니다!

## Building

### Build for ROS

1. 워크스페이스에서 `catkin build` 입력.
1. Beforehand, you have to set your "px4.launch" in (/workspace/src/mavros/mavros/launch). 
   Edit "px4.launch" as below.
   If you are using USB to connect your computer with Pixhawk, you have to set "fcu_url" as shown below.
   But, if you are using CP2102 to connect your computer with Pixhawk, you have to replace "ttyACM0" with "ttyUSB0".
   Modifying "gcs_url" is to connect your Pixhawk with UDP, because serial communication cannot accept MAVROS, and your nutshell connection simultaneously.

1. Write your IP address at "xxx.xx.xxx.xxx"
   ```xml
   ...
     <arg name="fcu_url" default="/dev/ttyACM0:57600" />
     <arg name="gcs_url" default="udp://:14550@xxx.xx.xxx.xxx:14557" />
   ...
   ```

### Build for PX4

1. Build PX4 Firmware and upload [in the normal way](../setup/building_px4.md#nuttx).
    
    For example, to build for Pixhawk 4/FMUv5 execute the following command in the root of the Firmware directory:

   ```sh
    make px4_fmu-v5_default upload
    ```

## Running the Code

Next test if the MAVROS message is sent to PX4.

### Running ROS

1. In a terminal enter
   ```sh
   roslaunch mavros px4.launch
   ```

1. In a second terminal run:

   ```sh
   rostopic pub -r 10 /mavros/keyboard_command/keyboard_sub std_msgs/Char 97
   ```

This means, publish 97 ('a' in ASCII) to ROS topic "/mavros/keyboard_command/keyboard_sub" in message type "std_msgs/Char". "-r 10" means to publish continuously in "10Hz".

### Running PX4

1. Enter the Pixhawk nutshell through UDP. Replace xxx.xx.xxx.xxx with your IP.

   ```sh
   cd Firmware/Tools
   ./mavlink_shell.py xxx.xx.xxx.xxx:14557 --baudrate 57600
   ```

1. After few seconds, press **Enter** a couple of times. You should see a prompt in the terminal as below:

   ```sh
   nsh>
   nsh>
   ```

Type "key_receiver", to run your subscriber module.

   ```
   nsh> key_receiver
   ```

Check if it successfully receives `a` from your ROS topic.