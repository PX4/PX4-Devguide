# Contributing

핵심 개발팀과 커뮤니티의 연락처 정보를 아래에서 찾을 수 있습니다. PX4 프로젝트의 3개 브랜치 Git :

* [master](https://github.com/px4/firmware/tree/master) 기본적으로 불안정버전으로 빠르게 개발 중인 버전
* [beta](https://github.com/px4/firmware/tree/beta) 철저히 테스트하고 있는 버전으로 비행 테스터를 위한 버전
* [stable](https://github.com/px4/firmware/tree/stable) 최신 릴리즈 버전

[rebase를 통한 발전과정](https://www.atlassian.com/git/tutorials/rewriting-history)을 보고 [Github flow](https://guides.github.com/introduction/flow/)을 피하고자 합니다. 그러나 글로벌 팀과 빠른 개발로 가끔은 merge하기 위해 다시 정렬해야하는 경우도 있습니다.

새로운 기능 개발에 기여하고자 한다면, [Github 계정만들고](https://help.github.com/articles/signing-up-for-a-new-github-account/) 저장소를 [fork](https://help.github.com/articles/fork-a-repo/)하고 [새로운 branch 만들고](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/) 변경 내용을 추가하고 마지막으로 [pull request 보내기](https://help.github.com/articles/using-pull-requests/)를 합니다. 변경 내용이 [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration)을 통과하면 merge 됩니다.

모든 contribution은 [BSD 3-clause 라이센스](https://opensource.org/licenses/BSD-3-Clause)를 따르며 모든 코드는 반드시 사용에 제약이 없어야 합니다.

## Code style formatting

PX4에서는 코드 포맷에 [astyle](http://astyle.sourceforge.net/)을 사용합니다. 유효한 버전은 다음과 같습니다.
* [astyle 2.05.1](https://sourceforge.net/projects/astyle/files/astyle/astyle%202.05.1/) (apt-get default)
* [astyle 2.06](https://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/) (recommended)
* [astyle 3.0](https://sourceforge.net/projects/astyle/files/astyle/astyle%203.0/)

일단 설치하면, 포맷은 `./Tools/astyle/check_code_style_all.sh`로 체크할 수 있습니다. 출력은 clean master에서 `Format checks passed`로 나와야 합니다. 이것이 제대로 작동했다면 향후에 모든 파일을 자동으로 검사하고 포맷팅하기 위해서 `make format`을 사용할 수 있습니다.

> **Info** [PX4](https://github.com/PX4/astyle)의 버전 2.05.1와 2.06 사이에 차이가 크다는 이슈가 있습니다. 이 경우 [sourceforge](http://astyle.sourceforge.net/)에서 버전 2.06이나 3.0을 설치하세요.

## 커밋과 커밋 메시지 (Commits and Commit Messages)

모든 변경에 대한 커밋 메시지는 커밋 내용을 설명하고 상세하게 기록하도록 합니다. 잘 구조화하면 한줄로 요약할 수 있겠지만 상세하게 설명을 제공하도록 합니다.

```
Component: Explain the change in one sentence. Fixes #1234

Prepend the software component to the start of the summary
line, either by the module name or a description of it.
(e.g. "mc_att_ctrl" or "multicopter attitude controller").

If the issue number is appended as <Fixes #1234>, Github
will automatically close the issue when the commit is
merged to the master branch.

The body of the message can contain several paragraphs.
Describe in detail what you changed. Link issues and flight
logs either related to this fix or to the testing results
of this commit.

Describe the change and why you changed it, avoid to
paraphrase the code change (Good: "Adds an additional
safety check for vehicles with low quality GPS reception".
Bad: "Add gps_reception_check() function").

Reported-by: Name <email@px4.io>
```

**`git commit -s`를 사용하면 모든 커밋에 사인을 남길 수 있습니다.** 마지막 라인에 `signed-off-by:`과 이름, 이메일을 추가합니다.

커밋 가이드는 Linux Kernel과 리누스 토발즈가 [관리하는 프로젝트](https://github.com/torvalds/subsurface/blob/a48494d2fbed58c751e9b7e8fbff88582f9b2d02/README#L88-L115) 의 좋은 예들을 기반으로 하고 있습니다.

## 비행 결과 테스트 (Tests Flight Results)

비행 테스트는 QA 관점에서 중요합니다. microSD 카드에서 log 파일을 [Log Muncher](http://logs.uaventure.com)에 업로드하고 비행 결과와 링크를 [PX4 Discuss](http://discuss.px4.io/)에 공유해 주세요.

## 포럼과 챗팅

* [Google+](https://plus.google.com/117509651030855307398)
* [Gitter](https://gitter.im/PX4/Firmware)
* [PX4 Discuss](http://discuss.px4.io/)

## 주단위 개발 콜미팅

PX4 개발 팀은 매주 개발관련 내용에 대해서 대화를 나눕니다.

* TIME: Wednesday 5PM CET, 11AM EST, 8AM PDT \([subscribe to calendar](https://calendar.google.com/calendar/ical/px4.io_fs35jm7ugmvahv5juhhr3tkkf0%40group.calendar.google.com/public/basic.ics)\)
* Uberconference: [www.uberconference.com/lf-dronecode](https://www.uberconference.com/lf-dronecode)
* Phone: +1 415-891-1494
* 아젠더는 미리 [PX4 Discuss](http://discuss.px4.io/c/weekly-dev-call)에 공지
* Issues와 PRs은 [devcall](https://github.com/PX4/Firmware/labels/devcall)로 레이블을 붙여서 토의
