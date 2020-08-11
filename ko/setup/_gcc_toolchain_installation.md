<!-- NuttX를 빌드할 모든 리눅스 플랫폼에서 활용하는 GCC 툴체인 문서 -->

아래 스크립트를 실행하여 GCC 7-2017-q4를 설치하십시오:

```sh
pushd .
cd ~
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
popd
```

이제 시스템을 다시 시작하십시오.


**문제 해결**

다음 명령을 입력하여 버전을 확인하십시오:

```sh
arm-none-eabi-gcc --version
```

다음과 비슷한 출력 결과가 나와야 합니다:

```sh
arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
Copyright (C) 2017 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```
