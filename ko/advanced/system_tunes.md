# 시스템 알림음

PX4는 중요한 시스템 상태와 문제를 음성으로 알리는 여러가지 [표준 알림음](https://docs.px4.io/master/en/getting_started/tunes.html)을 지정해두었습니다(예시: 시스템 시작, 이륙 준비 완료, 배터리 경고 등)

알림음은 문자열([안시 악보 표기](http://artscene.textfiles.com/ansimusic/information/ansimtech.txt))로 정의하며 [튠즈](https://github.com/PX4/Firmware/tree/master/src/lib/tunes) 라이브러리를 통해 코드로 재생합니다. 튠즈 라이브러리에는 기본 시스템 음 목록이 들어있습니다. 해당 내용은 [lib/tunes/tune_definition.desc](https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc) 파일을 참고하십시오.

PX4에는 기본음 또는 사용자 지정음을 재생(시험)할 때 활용할 모듈이 있습니다.

이 주제에서는 알림음을 만들고 시스템 알림 음으로 추가하는 일반 과정을 안내해드리도록 하겠습니다.


## 알림음 만들기

음 문자열은 [안시 악보 표기 방식](http://artscene.textfiles.com/ansimusic/information/ansimtech.txt)으로 정의합니다.

> **Tip** [QBasic PLAY 구문](https://en.wikibooks.org/wiki/QBasic/Appendix#PLAY)(위키북스)에서 형식 정보를 찾아볼 수 있으며, [tune_definition.desc](https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc) 파일로 재산출했습니다.

새 알림음을 만드는 가장 쉬운 방법은 뮤직 편집기를 활용하는 방법입니다. 이 방법으로 컴퓨터에서 음악을 편집하고 재생해볼 수 있으며, PX4에서 재생할 수 있는 형식으로 내보낼 수 있습니다.

안시 뮤직은 안시 BBS 시스템을 사용하던 시절에 유명한 방식이었기에, 최고의 편집 도구는 DOS 유틸리티입니다. 윈도우에서는 *도스박스*에서 *멜로디 마스터*를 사용하는 선택지가 있습니다.

프로그램을 사용하는 단계는 다음과 같습니다:

1. [도스박스](http://www.dosbox.com/)를 다운로드하여 설치합니다
1. [멜로디 마스터](ftp://archives.thebbs.org/ansi_utilities/melody21.zip)를 다운로드하고 새 디렉터리로 압축을 해제합니다
1. *도스박스* 콘솔을 엽니다
1. 멜로디 마스터 디렉터리를 아래와 같이 도스박스에서 마운트하십시오:
   ```
   mount c C:\<path_to_directory\Melody21
   ```
1. 다음 명령으로 *Melody Master*를 시작하십시오
   ```
   c:
   start
   ```
1. 일부 화면을 통해 선택지를 누른 후, **1**을 눌러 *멜로디 마스터*를 띄우십시오: ![Melody Master 2.1](../../assets/tunes/tunes_melody_master_2_1.jpg)

   화면의 절반 하단부에서 도구 사용에 필요한 키보드 단축키를 안내해줍니다(악보를 움직이고 음표 길이를 선택할 수 있는 등의 작업 가능).
1. 음악을 저장할 준비가 끝나면:
   - Press **F2** to give the tune a name and save it in the */Music* sub folder of your Melody Master installation.
   - Press **F7**, the scroll down the list of output formats on the right to get to ANSI. The file will be exported to the *root* of the Melody Master directory (with the same name and a file-type specific extension).
1. Open the file. The output might look like this:

   ![ANSI Output from file](../../assets/tunes/tune_musicmaker_ansi_output.png)

1. The string that can be played in PX4 is the bit between `MNT` and `P64`: `150L1O3DL16CL32<B>C<AEL16A`


## Testing Tunes

When you're ready to try it out a new tune on PX4, use the [tune_control](../middleware/modules_system.md#tunecontrol) library. For example, to test the tune we "created" above you would enter the following command on a console or shell (e.g. the [MAVLink Shell](../debug/mavlink_shell.md)):
```sh
tune_control play -m "150L1O3DL16CL32<B>C<AEL16A"
```

> **Note** Out of the box, the tune_control is only present on real hardware (not the simulator).


## Replacing Existing Tunes

Tunes are defined within [tune_definition.desc](https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc).

If you just need to replace an existing tune, then you can replace the file in your own fork, and update the tune strings defined in `PX4_DEFINE_TUNE`.


## Adding a New Tune


TBD.


<!-- 

1. Assumption is that you need to define a new `PX4_DEFINE_TUNE` with its own number in the file.
2. Need to look at how tunes are played. Problem for another day.

-->