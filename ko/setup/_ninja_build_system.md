!REDIRECT "https://docs.px4.io/master/ko/dev_setup/_ninja_build_system.html"

## 닌자 빌드 시스템

[Ninja](https://ninja-build.org/) 는 *make* 보다 빠른 빌드 시스템이며 PX4 *CMake* 제네레이서에서 이 빌드 시스템을 지원합니다. 

우분투 리눅스에서는 일반 저장소에서 자동으로 끌어와 설치할 수 있습니다.

```sh
sudo apt-get install ninja-build -y
```

다른 시스템에서는 꾸러미 관리자에 닌자가 들어있지 않을 수도 있습니다.
대신 이 경우에는,  바이너리를 다운로드하여 여러분의 지정 경로에 추가합니다:

```sh
mkdir -p $HOME/ninja
cd $HOME/ninja
wget https://github.com/martine/ninja/releases/download/v1.6.0/ninja-linux.zip
unzip ninja-linux.zip
rm ninja-linux.zip
exportline="export PATH=$HOME/ninja:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile
```
