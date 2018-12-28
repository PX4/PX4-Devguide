# 集成测试

这是关于端到端集成测试。 测试自动执行（[Jenkins CI](../test_and_ci/jenkins_ci.md)）

## ROS / MAVROS 测试

系统必备组件:

- [jMAVSim 仿真模拟](../simulation/jmavsim.md)
- [Gazebo 仿真模拟](../simulation/gazebo.md)
- [ROS 和 MAVROS](../simulation/ros_interface.md)

### Execute Tests

To run the complete MAVROS test suite:

```sh
cd <Firmware_clone>
source integrationtests/setup_gazebo_ros.bash $(pwd)
rostest px4 mavros_posix_tests_iris.launch
```

Or with GUI to see what's happening:

```sh
rostest px4 mavros_posix_tests_iris.launch gui:=true headless:=false
```

### Write a new MAVROS test (Python)

> **Note** Currently in early stages, more streamlined support for testing (helper classes/methods etc.) to come.

#### 1.) Create a new test script

Test scripts are located in `integrationtests/python_src/px4_it/mavros/`. See other existing scripts for examples. Also please consult the official ROS documentation on how to use [unittest](http://wiki.ros.org/unittest).

Empty test skeleton:

```python
#!/usr/bin/env python
# [... LICENSE ...]

#
# @author Example Author <author@example.com>
#
PKG = 'px4'

import unittest
import rospy
import rosbag

from sensor_msgs.msg import NavSatFix

class MavrosNewTest(unittest.TestCase):
    """
    Test description
    """

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.wait_for_service('mavros/cmd/arming', 30)

        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        self.rate = rospy.Rate(10) # 10hz
        self.has_global_pos = False

    def tearDown(self):
        pass

    #
    # General callback functions used in tests
    #
    def global_position_callback(self, data):
        self.has_global_pos = True

    def test_method(self):
        """Test method description"""

        # FIXME: hack to wait for simulation to be ready
        while not self.has_global_pos:
            self.rate.sleep()

        # TODO: execute test

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'mavros_new_test', MavrosNewTest)
```

#### 2.) Run the new test only

```sh
# 开始仿真
cd <Firmware_clone>
source integrationtests/setup_gazebo_ros.bash $(pwd)
roslaunch px4 mavros_posix_sitl.launch

# 运行测试（在新的 shell 中）：
cd <Firmware_clone>
source integrationtests/setup_gazebo_ros.bash $(pwd)
rosrun px4 mavros_new_test.py
```

#### 3.) Add new test node to launch file

In `launch/mavros_posix_tests_irisl.launch` add new entry in test group:

```xml
    <group ns="$(arg ns)">
        [...]
        <test test-name="mavros_new_test" pkg="px4" type="mavros_new_test.py" />
    </group>
```

Run the comlpete test suite as described above.