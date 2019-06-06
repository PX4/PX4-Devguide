# Sending a Custom Message from MAVROS to PX4

> **Warning** This article has been tested against:
  - **Ubuntu:** 18.04
  - **ROS:** Melodic
  - **PX4 Firmware:** 1.9.0

  However these steps are fairly general and so it should other distros/versions with little to no modifications.

<!-- Content reproduced with permission from @JoonmoAhn in https://github.com/JoonmoAhn/Sending-Custom-Message-from-MAVROS-to-PX4/issues/1 -->

## MAVROS Installation

Follow *Source Installation* instructions from [mavlink/mavros](https://github.com/mavlink/mavros/blob/master/mavros/README.md) to install "ROS Kinetic".

## MAVROS

1. We start by creating a new MAVROS plugin, in this example named **keyboard_command.cpp** (in **workspace/src/mavros/mavros_extras/src/plugins**) by using the code below:

   The code subscribes a 'char' message from ROS topic `/mavros/keyboard_command/keyboard_sub` and sends it as a MAVLink message.
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

1. Edit **mavros_plugins.xml** (in **workspace/src/mavros/mavros_extras**) and add following lines:
   ```xml
   <class name="keyboard_command" type="mavros::extra_plugins::KeyboardCommandPlugin" base_class_type="mavros::plugin::PluginBase">
        <description>Accepts keyboard command.</description>
   </class>
   ```

1. Edit **CMakeLists.txt** (in **workspace/src/mavros/mavros_extras**) and add following line in `add_library`.
   ```cmake
   add_library( 
   ...
     src/plugins/keyboard_command.cpp 
   )
   ```

1. Inside **common.xml** in (**workspace/src/mavlink/message_definitions/v1.0**), add your MAVLink message:
   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description>Keyboard char command.</description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

## PX4 Changes

1. Inside **common.xml** (in **Firmware/mavlink/include/mavlink/v2.0/message_definitions**), add your MAVLink message as following (as for MAVROS section above):
   ```xml
   ...
     <message id="229" name="KEY_COMMAND">
        <description> mavlink message creating test </description>
        <field type="char" name="command"> </field>
      </message>
   ...
   ```

1. Remove *common*, *standard* directories in (**Firmware/mavlink/include/mavlink/v2.0**).
   ```sh
   rm -r common
   rm -r standard
   ```
1. Git clone "mavlink_generator" in your home directory (or any directory you want) and execute it.
   ```sh
   git clone https://github.com/mavlink/mavlink mavlink-generator
   cd mavlink-generator
   python mavgenerate.py
   ```

1. You will see a "MAVLink Generator" popup:
   - For *XML*, "Browse" to **/Firmware/mavlink/include/mavlink/v2.0/message_definitions/standard.xml**.
   - For Out, "Browse" to **/Firmware/mavlink/include/mavlink/v2.0/**.
   - Select Language **C**
   - Select Protocol **2.0**
   - Check *Validate*

   Then, press **Generate**.
   You will see *common*, and *standard* directories created in **/Firmware/mavlink/include/mavlink/v2.0/**.

1. Make your own uORB message file **key_command.msg** in (Firmware/msg). My "key_command.msg" has only the code:
   ```
   char cmd
   ```
   Then, in **CMakeLists.txt** (in **Firmware/msg**), include
   ```cmake
   set(
   ...
        key_command.msg
        )
   ```

1. Edit **mavlink_receiver.h** (in **Firmware/src/modules/mavlink**)

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
       orb_advert_t _key_command_pub;
   }
   ```

1. Edit **mavlink_receiver.cpp** (in **Firmware/src/modules/mavlink**).
   This is where PX4 receives the MAVLink message sent from ROS, and publishes it as a uORB topic.
   ```cpp
   ...
   MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
   ...
    _key_command_pub(nullptr)
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

1. Make your own uORB topic subscriber just like any example subscriber module. In my case, I made my own module in (/Firmware/src/inrol/key_receiver). 
   In this directory, I have two files **CMakeLists.txt**, **key_receiver.cpp**.
   Each one looks like following.

   -CMakeLists.txt

   ```cmake
   px4_add_module(
       MODULE inrol__key_receiver
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

   extern "C" {__EXPORT int key_receiver_main(int argc, char **argv);}

   int key_receiver_main(int argc, char **argv)
   {
       int key_sub_fd = orb_subscribe(ORB_ID(key_command));
       orb_set_interval(key_sub_fd, 200); // limit the update rate to 200ms

       px4_pollfd_struct_t fds[1];
       fds[0].fd = key_sub_fd, fds[0].events = POLLIN;

       int error_counter = 0;

       while(1)
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

1. Add your module in **nuttx_px4fmu-v2_default.cmake** (in **Firmware/cmake/configs**).
   ```cmake
   set(config_module_list
   ...
   inrol/key_receiver
   )
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

### Build for PX4

Build PX4 Firmware and upload [in the normal way](../setup/building_px4.md#nuttx--pixhawk-based-boards).

For example, to build for Pixhawk 4/FMUv5 execute the following command in the root of the Firmware directory:
```sh
make px4fmu-v5_default upload
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

1. Go into the Pixhawk nutshell through UDP.
   Write your IP at xxx.xx.xxx.xxx.
   ```sh
   cd Firmware/Tools
   ./mavlink_shell.py xxx.xx.xxx.xxx:14557 --baudrate 57600
   ```

1. After few seconds, press **Enter** several times.
   You should see a prompt in the terminal as below:
   ```sh
   nsh>
   ```
   Type "key_receiver", to run your subscriber module.
   ```
   nsh> key_receiver
   ```

Check if it successfully receives `a` from your ROS topic.
