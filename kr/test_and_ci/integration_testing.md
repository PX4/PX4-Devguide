# 통합 테스팅

이것은 end to end 통합 테스팅에 관한 내용입니다. 테스트는 자동으로 실행됩니다. [Jenkins CI](../test_and_ci/jenkins_ci.md)

## ROS / MAVROS 테스트

전제 조건:

  * [SITL Simulation](../simulation/sitl.md)
  * [Gazebo](../simulation/gazebo.md)
  * [ROS and MAVROS](../simulation/ros_interface.md)

### 테스트 실행

전체 MAVROS 테스트를 실행하기 위해 :

```sh
cd <Firmware_clone>
source integrationtests/setup_gazebo_ros.bash $(pwd)
rostest px4 mavros_posix_tests_iris.launch
```

혹은 GUI로 일어나는 일 보기:

```sh
rostest px4 mavros_posix_tests_iris.launch gui:=true headless:=false
```

### 새로운 MAVROS 테스트 작성 (Python)

> **Note** 현재 초기 단계로 테스트를(helper classes/method 등) 위해 streamlined 지원

####1.) 새로운 테스트 스크립트 생성

테스트 스크립트는 `integrationtests/python_src/px4_it/mavros/`에 위치. 기존의 다른 스크립트 참조. [unittest](http://wiki.ros.org/unittest)를 어떻게 사용하는지는 공식 ROS 문서를 참고.

빈 테스트 skeleton:

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

####2.) 새로운 테스트만 실행

```sh
# Start simulation
cd <Firmware_clone>
source integrationtests/setup_gazebo_ros.bash $(pwd)
roslaunch px4 mavros_posix_sitl.launch

# Run test (in a new shell):
cd <Firmware_clone>
source integrationtests/setup_gazebo_ros.bash $(pwd)
rosrun px4 mavros_new_test.py
```

####3.) 파일을 런치하기 위해서 새로운 테스트 노드 추가

`launch/mavros_posix_tests_irisl.launch`에 테스트 그룹내에 새로운 엔트리 추가:

```xml
	<group ns="$(arg ns)">
		[...]
        <test test-name="mavros_new_test" pkg="px4" type="mavros_new_test.py" />
    </group>
```

위와 같이 전체 테스트 실행.
