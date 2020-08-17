# 시험 MC_04 - 안전 장치 시험

❏ Verify RC Loss action is Return to Land

❏ Verify Data Link Loss action is Return to Land and the timeout is 10 seconds

❏ 배터리 안전 장치 검증

&nbsp;&nbsp;&nbsp;&nbsp;❏ 안전 조치 동작은 착륙

&nbsp;&nbsp;&nbsp;&nbsp;❏ 배터리 경고 잔여량이 25%인가

&nbsp;&nbsp;&nbsp;&nbsp;❏ 배터리 안전 장치 가동 잔여량이 20%인가

&nbsp;&nbsp;&nbsp;&nbsp;❏ 배터리 응급 상태 잔여량이 15%인가 

❏ 고도 제어 모드에서 이륙

❏ 기준 위치에서 최소한 20미터 이상 이동

❏ RC loss

&nbsp;&nbsp;&nbsp;&nbsp;❏Turn off RC and check the vehicle returns to home position, wait for the descent and turn on the RC and take over.

## 데이터 연결 유실

❏ Disconnect telemetry, vehicle should return to home position after 10 seconds, wait for the descent and reconnect the telemetry radio

## 고도 제어 모드 전환

❏ Make sure roll, pitch and yaw sticks respond like in Stabilize mode

❏ Throttle should control altitude, and when the stick is centered it must maintain altitude

## 위치 제어 모드 전환

❏ When the sticks are centered, it must maintain position

❏ Move roll, pitch and yaw and check the vehicle is moving according to the inputs

❏ Center the sticks again and check the vehicle maintains position

## 배터리 안전 장치 가동 대기

❏ Confirm the warning message is received in QGC

❏ Confirm the vehicle returns to land

❏ Confirm the vehicle lands.