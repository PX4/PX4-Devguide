# 번역

*QGroundControl* 프로그램과 PX4, *QGroundControl*, MAVLink 안내서 번역 도움을 간절히 바랍니다!

문서 (와 *QGroundControl* 프로그램)은 번역 도구로 [Crowdin](https://crowdin.com) 을 활용합니다. Crowdin은 Github에서 원본 주제를 자동으로 가져와서 번역하고 검토(승인)할 새 문자열 또는 바뀐 문자열을 보여줍니다.

Crowdin은 "풀 리퀘스트"(이 단계에서 개발팀이 주기적으로 검토하고 승인해줍니다) 방식으로 Github에 번역한 문서를 내보냅니다. 내보낸 출력물에는 번역한 원본 문서와 번역한 문자열로 바뀌어 승인한 텍스트가 들어있습니다(예: 문자열을 번역하지 않았거나 바꾸지 않았다면, 영문으로 그대로 나타냅니다).

> **팁** 번역 팀에 참여하려면 (무료) [Crowdin 계정](https://crowdin.com/join) 이 필요합니다!

<span></span>

> **Note** The benefit of this system is that the translation closely tracks the source documents. Readers will not be mislead by old and out of date translations.

## Getting Started

The steps to join our translation tream are:

1. Join Crowdin: https://crowdin.com/join
2. Open the translation project you want to join: 
    - [QGroundControl](https://crowdin.com/project/qgroundcontrol)
    - [PX4 User Guide](https://crowdin.com/project/px4-user-guide)
    - [PX4 Developer Guide](https://crowdin.com/project/px4-developer-guide)
    - [QGroundControl Developer Guide](https://crowdin.com/project/qgroundcontrol-developer-guide)
    - [QGroundControl User Guide](https://crowdin.com/project/qgroundcontrol-user-guide)
    - [MAVLink Guide](https://crowdin.com/project/mavlink)
3. Select the language you want to translate
4. Click the **Join** button (next to the text *You must join the translators team to be able to participate in this project*) > **Note** You will be notified once your application to join is accepted.
5. Start translating!

## Special Notes

### Do not modify Note, Tip, Warning Text

Gitbook uses special prefix text to indicate Notes, Tips and Warnings (e.g. `> **Note**`, `> **Tip**`, `> **Warning**`). This is displayed in Crowdin as shown:

```html
<0>Note</0> The text for the note.
```

It is important that you do not translate the text inside the `<0>Note</0>` tags as this will stop the note from rendering properly.

## Adding a New Language

If the language you want to translate is not presented as an option on the project home page then you will need to request it.

You can do this by contacting the project owner (there is a contact link on each project's home page).

## Getting Help

The *Crowdin* interface is self explanatory, but there is plenty of additional information on the [knowledgeable](https://support.crowdin.com/) and [feedback tool](https://crowdin.uservoice.com/forums/31787-collaborative-translation-tool).

You can also ask for help from translators and developers in the Dronecode community using [our support channels](../README.md#support).