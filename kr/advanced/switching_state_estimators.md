# 상태 Estimators 전환
이 페이지에서는 어떤 상태 estimator들이 있는지 그리고 이들 사이에서 전환하는 방법을 설명합니다.

## 사용가능한 estimators

**1. Q 자세 estimator**

자세 Q estimator는 매우 단순하며 attitude에 대해서 쿼터니언 기반 상보 필터입니다.

**2. INAV 위치 estimator**

INAV 위치 estimator는 3D 위치와 속도 상태에 대한 상보필터입니다.


**3. LPE 위치 estimator**

LPE 위치 estimator는 3D 위치과 속도 상태에 대한 extended kalman filter입니다.

**4. EKF2 자세, 위치와 바람상태 estimator**

EKF2는 자세, 3D 위치 / 속도와 바람 상태를 추정하는 extended kalman filter입니다.

**4. EKF 자세, 위치와 바람 상태 estimator (deprecated)**
EKF2와 유사한 extended kalam filter입니다. 하지만 조만간 EKF2로 완전히 대체될 예정입니다.
이 필터는 고정익에만 사용됩니다.

## 다른 estimators를 활성화시키는 방법
멀티로터와 VTOL에 대해 다음 설정 중에 하나를 선택하기 위해서 **SYS_MC_EST_GROUP** 파라미터를 사용합니다.


| SYS_MC_EST_GROUP | Q Estimator| INAV | LPE | EKF2 |
| --- | --- | --- | --- | --- |
| 0 | enabled | enabled | | |
| 1 | enabled |  | enabled | |
| 2 |  |  | | enabled |
