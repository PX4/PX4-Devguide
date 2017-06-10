# PWM_limit State Machine

[PWM_limit State Machine]은 pre-armed와 armed 입력의 작용으로 PWM 출력을 제어하는 것입니다. armed 신호를 받은 경우 "armed"와 ramp-up throttle 사이에 지연시간을 제공합니다.

## 빠른 요약
**Inputs**
* armed: 프로펠러 회전과 같이 위험한 동작도 가능하게 된다.
* pre-armed: 제어 가능한 부분을 움직이는 것과 같이 위험하지 않은 동작이 가능하게 된다.
 * 이 입력은 현재 상태를 덮어쓰게 된다.
 * 현재 어떤 상태인지 상관없이 pre-armed은 즉각 ON 상태의 동작을 하게된다.
 ** pre-armed의 해제시키면 현재 상태로 반전된다.

**States**
  * INIT 과 OFF
    * pwm 출력은 disarmed 값으로 설정
  * RAMP
    * pwm 출력 disarmed 값에서 최소값으로 서서히 변함
  * ON
    * pwm 출력은 제어 값에 따라 설정

## 상태 전이 다이어그램 (State Transition Diagram)
![](../../assets/diagrams/pwm_limit_state_diagram.png)
