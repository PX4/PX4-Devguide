# 착륙 감지기(Land Detector) 설정

착륙 감지기는 착륙과 지상 접촉과 같은 비행체의 상태를 알려주는 동작 비행체 모델입니다.

## Auto-Disarming 설정

디폴트로 착륙 감지기는 착륙을 감지합니다. 하지만 auto-disarm을 하지는 않습니다. 만약 [COM_DISARM_LAND](../advanced/parameter_reference.md#COM_DISARM_LAND)가 0이 아닌 값으로 설정되어 있다면 시스템은 N 초 후에\(설정되어 있는 값\) auto-disarm이 됩니다.

## 멀티콥터 착륙 감지기 설정

파라미터의 전체 집합은 `LNDMC` 접두어로 시작하는 QGroundControl 파라미터 에디터에서 설정할 수 있습니다. airframe마다 다른 핵심 파라미터는 다음과 같이 :

* [MPC_THR_HOVER](../advanced/parameter_reference.md#MPC_THR_HOVER) - 시스템의 hover 스로틀\(퍼센티지로 디폴트 50%\). altitude 제어를 정확하게 하는 목적이 아니라 정확히 착륙 감지를 하기 위해서는 이를 정확하게 설정하는 것이 중요하다. 페이로드가 없는 레이서와 큰 카메라 드론은 더 낮게 셋팅해야함\(예로 35%\)
* [MPC_THR_MIN](../advanced/parameter_reference.md#MPC_THR_MIN) - 시스템의 전체 최소 스로틀. 제어되면서 내려오도록 설정해야함.
* [LNDMC_THR_RANGE](../advanced/parameter_reference.md#LNDMC_THR_RANGE) - 착륙시 수용하는 최소화 hover 스로틀 사이에 범위를 정의하기 위해서 스케일링 팩터로 사용. Example: If the minimum throttle is 0.1, the hover throttle is 0.5 and the range is 0.2 \(20%\), then the highest throttle value that counts as landed is: `0.1 + (0.5 - 0.1) * 0.2 = 0.18`.


The throttle level to trigger takeoff is hard-coded to 62.5%. If the pilot raises above this threshold the system will attempt to take off. This value should be greater than the hover throttle.


## 고정익 착륙 감지기 설정

파라미터의 전체 집합은 `LNDFW` 접두어로 시작하는 QGroundControl 파라미터 에디터에서 설정할 수 있습니다. 2개 사용자 파라미터는 가끔씩 튜닝이 필요합니다. :

* [LNDFW_AIRSPD_MAX](../advanced/parameter_reference.md#LNDFW_AIRSPD_MAX) - 시스템이 허용하는 최대 airspeed는 착륙시 고려. 디폴트 값인 8 m/s는 airspeed 센싱 정확도와 빠른 트리거링을 고려했을 때 신뢰할 수 있음. 더 성능이 좋은 airspeed 센서를 사용하는 경우 이 파라미터 값을 더 낮게 설정 가능.
* [LNDFW_VELI_MAX](../advanced/parameter_reference.md#LNDFW_VELI_MAX) - 시스템에 대한 최대 속도에서 여전히 착륙시 고려. 이 파라미터는 손으로 던져서 airframe 런칭하는 경우 착륙 감지를 시작하는 때를 조정할 수 있습니다.
