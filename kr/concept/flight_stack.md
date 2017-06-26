# PX4 Flight Stack

PX4 flight stack은 가이던스, 네비게이션 그리고 제어 알고리즘을 합친 개념입니다. attitude와 position에 관한 estimator뿐만 아니라 고정익, 멀티로터, VTOL에 관한 제어기도 포함됩니다.

## Estimation 와 제어 구조(Control Architecture)

아래 그림은 일반 구현 블록을 보여주고 있습니다. 비행체에 따라서 일부는 하나의 application로 통합시킬 수도 있습니다.(예로 특정 비행체에서 모델 예측 제어기가 필요한 경우)

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
