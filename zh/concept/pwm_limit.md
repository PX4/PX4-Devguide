# PWM_limit状态机

PWM_limit 状态机根据解锁前和解锁后的输入控制PWM输出。并在”解锁“、油门加速和解锁信号的断言之间提供延迟。



## 快速概要

**输入**

* armed: 置1使能诸如旋转螺旋桨的危险行为。
* pre-armed: 置1使能诸如移动控制面的良性行为。
  * 这个输入覆盖当前状态。
  * pre-aremd置1无视当前状态，立即强制转移到状态ON，值0则回复到当前状态。


**状态**

* INIT和OFF
  * pwm输出值设定为未解锁值。

* RAMP
  * pwm输出值从未解锁值上升到最小值。

* ON
  * pwm输出值根据控制量设定。

## 状态转移图
![](../../images/diagrams/pwm_limit_state_diagram.png)
