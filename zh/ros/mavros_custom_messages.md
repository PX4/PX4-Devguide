# 将自定义消息从MOVROS发送到PX4

> **警告** 本文已经经过测试:

- **Ubuntu:** 18.04
- **ROS:** Melodic
- **PX4 Firmware:** 1.9.0
    
    这些步骤普遍适用于其他发行版，或者只需要稍微修改。

<!-- Content reproduced with permission from @JoonmoAhn in https://github.com/JoonmoAhn/Sending-Custom-Message-from-MAVROS-to-PX4/issues/1 -->

## MAVROS 安装

按照*Source Installation*中的指导，从[mavlink/mavros](https://github.com/mavlink/mavros/blob/master/mavros/README.md)安装"ROS Kinetic”版本。

## MAVROS

1. 首先，我们创建一个新的MAVROS 插件，在**keyboard_command.cpp**(**workspace/src/mavros/mavros_extras/src/plugins**)示例中添加以下代码：
    
    代码功能是从ROS消息主题`/mavros/keyboard_command/keyboard_sub`中订阅了一个字符消息，并且将其作为MAVLink 消息发送出去。

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

1. 编辑**mavros_plugins.xml**(**workspace/src/mavros/mavros_extras**）文件，并添加以下内容：

   ```xml
   <class name="keyboard_command" type="mavros::extra_plugins::KeyboardCommandPlugin" base_class_type="mavros::plugin::PluginBase">
        <description>Accepts keyboard command.</description>
   </class>
   ```

1. 编辑 **CMakeLists.txt**（**workspace/src/mavros/mavros_extras**）文件，并在`add_library`中添加以下内容：

   ```cmake
   add_library( 
   ...
     src/plugins/keyboard_command.cpp 
   )
   ```

1. 打开**common.xml**（**workspace/src/mavlink/message_definitions/v1.0**）文件，复制下面内容到你的MAVLink 消息中：

   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description>Keyboard char command.</description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

## PX4 更改

1. 打开 **common.xml** （**Firmware/mavlink/include/mavlink/v2.0/message_definitions**）文件，并添加你的MAVLink 消息（和前面MAVROS部分相同的操作）:

   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description>Keyboard char command.</description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

1. 删除*common*,*standard*文件夹（**Firmware/mavlink/include/mavlink/v2.0**）。

   ```sh
   rm -r common
   rm -r standard
   ```

1. git 克隆"mavlink_generator"到你想要的文件夹下并执行。

   ```sh
   git clone https://github.com/mavlink/mavlink mavlink-generator
   cd mavlink-generator
   python mavgenerate.py
   ```

1. 你会看到一个“MAVLink Generator”应用程序窗口：
    
    - *XML*一栏选择**/Firmware/mavlink/include/mavlink/v2.0/message_definitions/standard.xml**。
    - 输出一栏选择**/Firmware/mavlink/include/mavlink/v2.0/**
    - 语言一栏选择**C**
    - 选择**2.0**协议
    - 勾选*Validate*
    
    然后点击**Generate**按钮。 在**/Firmware/mavlink/include/mavlink/v2.0/**中会生成*common*和*standard*文件夹。

2. 添加你自己的uORB消息文件 **key_command.msg**到Firmware/msg目录下。 示例中的“key_command.msg”文件只包含以下代码：

   ```
   char cmd
   ```

然后，在 **CMakeLists.txt**（**Firmware/msg**）文件中包含你的消息文件。

   ```cmake
   set(
   ...
        key_command.msg
        )
   ```

1. 编辑**mavlink_receiver.h**（**Firmware/src/modules/mavlink**）文件。

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

1. 编辑 **mavlink_receiver.cpp**（**Firmware/src/modules/mavlink**）文件。 这是PX4接收ROS发送过来的MAVLink 消息的地方，并且将消息作为uORB主题发布。

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

1. 像其他示例一样订阅你自己的uORB主题。 让我们先在/Firmware/src/modules/文件夹下创建模块key_receiver。 在此模块目录下创建两个文件**CMakeLists.txt**，**key_receiver.cpp**。 每个文件如下所示：
    
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

For a more detailed explanation please see the documentation for [Writing your first application](https://dev.px4.io/en/apps/hello_sky.html).

1. Lastly add your module in the **default.cmake** file correspondent to your board in **Firmware/boards/**. For example for the Pixhawk 4 add the following code in **Firmware/boards/px4/fmu-v5/default.cmake**:

   ```cmake
    MODULES
        ...
        key_receiver
        ...
    ```

Now you are ready to build all your work!

## Building

### Build for ROS

1. In your workspace enter: `catkin build`.
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

### PX4 编译

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

### PX4 运行

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