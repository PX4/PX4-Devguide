# Jenkins CI

[SITL01](http://sitl01.dronetest.io/)주소의 Jenkins의 지속통합 서버는 PX4 SITL 대상으로 통합 테스트를 자동으로 수행합니다.

## 개요

  * 관련 컴포넌트: Jenkins, Docker, PX4 SITL
  * [Docker Containers](../test_and_ci/docker.md) 내부에서 테스트 수행
  * Jenkins가 수행하는 2개 작업 : master에 대해서 각 PR를 검사, master로 들어오는 모든 push를 검사

## Test 실행

Jenkins는 [run_container.bash](https://github.com/PX4/Firmware/blob/master/integrationtests/run_container.bash)를 사용해서 container를 시작하고 이 container는 테스트를 컴파일하고 실행하기 위해서 차례로 [run_tests.bash](https://github.com/PX4/Firmware/blob/master/integrationtests/run_tests.bash)를 실행합니다.

만약 Docker가 설치되어 있다면 동일한 방식으로 로컬에서 사용할 수 있습니다 :

```sh
cd <directory_where_firmware_is_cloned>
sudo WORKSPACE=$(pwd) ./Firmware/integrationtests/run_container.bash
```

## Server 셋업

### 설치

Jenkins의 설치와 유지보수하는 상세한 방법은 [script/log](https://github.com/PX4/containers/tree/master/scripts/jenkins) 셋업을 참고하세요.

### 설정

  * Jenkins 보안 기능 사용
  * 설치된 plugins
    * github
    * github pull request builder
    * embeddable build status plugin
    * s3 plugin
    * notification plugin
    * collapsing console sections
    * postbuildscript
