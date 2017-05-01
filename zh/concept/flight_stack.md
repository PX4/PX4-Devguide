---
translated_page: https://github.com/PX4/Devguide/blob/master/en/concept/flight_stack.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

#  PX4飞行控制栈

PX4飞行控制栈集成了各种自主无人机的制导、导航以及控制算法。支持的机型包括固定翼，多旋翼以及垂直起降飞行器，算法包括姿态估计算法和姿态控制算法。

## 估计与控制结构

下图所示为一个典型框图（typical blocks）的实现示例。根据飞行器的不同，其中的一些框图（blocks）也可以组成一个单独的应用（例如，当我们需要一个特定飞行器的模型预测控制器时）

{% mermaid %}
graph TD;
  pos_ctrl-->att_ctrl;
  att_ctrl-->mixer;
  inertial_sensors-->attitude_estimator;
  inertial_sensors-->position_estimator;
  GPS-->position_estimator;
  computer_vision-->position_estimator;
  position_estimator-->navigator;
  position_estimator-->attitude_estimator;
  position_estimator-->pos_ctrl;
  attitude_estimator-->att_ctrl;
{% endmermaid %}
