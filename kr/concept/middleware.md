# PX4 Middleware

PX4 미들웨어는 주로 임베디드 센서와 관련된 디바이스 드라이버와 [flight controls](../concept/flight_stack.md)를 실행하는 application과 센서를 연결을 위한 publish-subscribe 기반 미들웨어로 구성되어 있습니다.

publish-subscribe 개념을 사용한다는 의미는 :

  * 이 시스템은 리액티브(reactive)속성을 가진다 : 새로운 데이터가 들어오면 바로 업데이트된다.
  * 완전하게 병렬로 실행된다.
  * 시스템 컴포넌트는 어디서나 thread-safe 방식으로 데이터를 사용할 수 있다.
