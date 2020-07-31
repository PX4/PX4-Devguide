# 번역

*QGroundControl* 프로그램과 PX4, *QGroundControl*, MAVLink 안내서 번역 도움을 간절히 바랍니다!

문서 (와 *QGroundControl* 프로그램)은 번역 도구로 [Crowdin](https://crowdin.com) 을 활용합니다. Crowdin은 Github에서 원본 주제를 자동으로 가져와서 번역하고 검토(승인)할 새 문자열 또는 바뀐 문자열을 보여줍니다.

Crowdin은 "풀 리퀘스트"(이 단계에서 개발팀이 주기적으로 검토하고 승인해줍니다) 방식으로 Github에 번역한 문서를 내보냅니다. 내보낸 출력물에는 번역한 원본 문서와 번역한 문자열로 바뀌어 승인한 텍스트가 들어있습니다(예: 문자열을 번역하지 않았거나 바꾸지 않았다면, 영문으로 그대로 나타냅니다).

> **Tip** 번역 팀에 참여하려면 (무료) [Crowdin 계정](https://crowdin.com/join) 이 필요합니다!

<span></span>

> **참고** 이 시스템의 잇점은 원본 문자열과 번역을 맞붙여 추적하는 기능입니다. 독자 입장에서는 오래되어 때가 지난 번역을 보고 오해할 일이 없습니다.

## 시작하기

번역 팀에 참여하는 방법은 다음과 같습니다:

1. Crowdin에 가입합니다: https://crowdin.com/join
2. 참여하고자 하는 번역 프로젝트를 엽니다: 
    - [QGroundControl](https://crowdin.com/project/qgroundcontrol)
    - [PX4 사용자 안내서](https://crowdin.com/project/px4-user-guide)
    - [PX4 개발자 안내서](https://crowdin.com/project/px4-developer-guide)
    - [QGroundControl 개발자 안내서](https://crowdin.com/project/qgroundcontrol-developer-guide)
    - [QGroundControl 사용자 안내서](https://crowdin.com/project/qgroundcontrol-user-guide)
    - [MAVLink 안내서](https://crowdin.com/project/mavlink)
3. 번역하려는 언어를 선택합니다
4. **Join** 버튼을 누릅니다(*You must join the translators team to be able to participate in this project* 텍스트 옆에 있음) > **Note** 참여 신청을 승인 받으면 알림이 옵니다.
5. 번역을 시작하세요!

## 별도 참고

### Note, Tip, Warning 텍스트를 수정하지 마십시오

Gitbook에서는 참고, 팁, 경고를 나타내는 특별한 접두 텍스트를 활용합니다(예: `> **Note**`, `> **Tip**`, `> **Warning**`). Crowdin에서는 다음과 같이 나타납니다:

```html
<0>Note</0> 이 텍스트는 참고 내용입니다.
```

`<0>Note</0>` 태그 안에 있는 텍스트를 번역하지 않는게 중요합니다. 만일 이를 번역하면 참고 내용을 제대로 표시하지 않고 멈춥니다.

## 새 언어 추가

번역하려는 언어가 프로젝트 홈페이지 옵션에 없으면 요청해야합니다.

프로젝트 소유자에게 연락하면 새 언어를 추가할 수 있습니다(각 프로젝트 홈페이지에 연락 링크가 있습니다).

## 도움 받기

The *Crowdin* interface is self explanatory, but there is plenty of additional information on the [knowledgeable](https://support.crowdin.com/) and [feedback tool](https://crowdin.uservoice.com/forums/31787-collaborative-translation-tool).

You can also ask for help from translators and developers in the Dronecode community using [our support channels](../README.md#support).