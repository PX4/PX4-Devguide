# PWM_limit 상태 머신

[PWM_limit 상태 머신]은 PWM의 출력을 pre-armed 와 armed 입력을 받는 함수로 제어합니다. "armed"의 실행 시점과 armed 시그널 발생후 스로틀 출력 증가 시점 사이에 지연 시간이 있습니다.

## 간단한 요약 설명

**입력**

- armed: 프로펠러를 돌리는 식의 위험한 동작을 활성화 합니다
- pre-armed: 제어 평면을 이동하는 식의 동작을 활성화 합니다 
    - 현재 상태로부터 입력 값의 상태로 바꿉니다.
    - 현재의 상태와 상관없이, pre-armed 상태가 되면 상태를 ON으로 강제로 바꿉니다.
    - pre-armed 상태를 취소하면 현재 상태로 복원합니다.

**상태**

- INIT와 OFF 
    - PWM의 출력을 disarmed 값으로 설정합니다.
- RAMP 
    - PWM 출력을 종전의 disarmed 값에서 최소값으로 조절합니다.
- ON 
    - PWM 출력을 제어값에 맞게 설정합니다.

## 상태 전이 다이어그램

![](../../assets/diagrams/pwm_limit_state_diagram.png)