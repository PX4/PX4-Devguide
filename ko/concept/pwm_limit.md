# PWM_limit 상태 머신

[PWM_limit 상태 머신] 은 PWM의 출력을 pre-armed 와 armed 입력의 함수로 제어합니다. "armed"의 어썰션과 armed 신호의 어썰션에서 병목을 증가시키의 어썰션과 armed 신호의 어썰션에서 병목을 증가시키것에는 딜레이가 있습니다.

## Quick Summary

**입력**

- armed: 프로펠러는 돌리는 것과 같이 위험한 행동을 하도록 활성화하는 것
- pre-armed: 제어 표면을 움직이기 시작하도록 활성화하는 것 
    - 이 입력은 현재의 상태보다 더 우선시 된다.
    - pre-armed의 주장은 현재의 상태와 상관없이 즉시 ON 상태를 강요한다.
    - pre-armed의 철회는 현재 상태로 되돌아가도록 한다.

**상태**

- INIT 과 OFF 
    - PWM의 출력을 disarmed 값으로 설정한다.
- RAMP 
    - PWM 출력을 disarmed 값에서 최소값으로 증가시킵니다.
- ON 
    - PWM 출력을 제어값에 맞게 설정합니다.

## 상태 전이 다이어그램

![](../../assets/diagrams/pwm_limit_state_diagram.png)