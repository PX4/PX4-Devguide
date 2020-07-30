# 소스 코드 관리

## 분기 모델

PX4 프로젝트는 분기를 셋으로 나누는 git 분기 모델을 활용합니다:

- [master](https://github.com/px4/firmware/tree/master)는 기본 브랜치이며, 비교적 불안정하고 빠른 개발 속도를 보여줍니다.
- [beta](https://github.com/px4/firmware/tree/beta)는 철저하게 시험한 코드의 브랜치입니다. 비행체 시험자들이 활용하도록 만들었습니다.
- [stable](https://github.com/px4/firmware/tree/stable)은 최신 릴리스를 가리킵니다.

[리베이스를 통한 선형 기록](https://www.atlassian.com/git/tutorials/rewriting-history)을 유지하며 [Github 흐름](https://guides.github.com/introduction/flow/)을 배제합니다. 그러나, 전세계의 팀과 역동적인 개발로 인해 그때그때 최종적으로 병합을 진행합니다.

새 기능을 추가하려면, [Github에 가입](https://help.github.com/articles/signing-up-for-a-new-github-account/)하고, 저장소를 [포킹](https://help.github.com/articles/fork-a-repo/)한 후, [새 브랜치를 만들어](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/), 코드를 바꾸어넣은 다음, 마지막으로 [풀 리퀘스트를 보냅니다](https://help.github.com/articles/using-pull-requests/). [지속 통합](https://en.wikipedia.org/wiki/Continuous_integration) 테스트를 통과하고 나면, 바뀐 코드를 병합할 예정입니다.

모든 코드의 기여는 [BSD 3-clause 라이선스](https://opensource.org/licenses/BSD-3-Clause) 하에 진행하며, 사용상 모든 코드에 대해 더 이상의 제약 사항을 강요해서는 안됩니다.

## 코드 형식 정리

PX4는 [astyle](http://astyle.sourceforge.net/) 방식으로 코드 형식을 정리합니다. 유효한 버전은 다음과 같습니다:

- [astyle 2.06](https://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/) (추천)
- [astyle 3.0](https://sourceforge.net/projects/astyle/files/astyle/astyle%203.0/)
- [astyle 3.01](https://sourceforge.net/projects/astyle/files/)

설치하고 나면 `./Tools/astyle/check_code_style_all.sh` 명령으로 코드 형식을 점검할 수 있습니다. 정리한 master 브랜치에서의 출력은 `Format checks passed`로 나와야 합니다. 이 결과가 나왔다면, 나중에 모든 파일의 코드 형식을 검사할 때 `make format` 명령을 활용할 수 있습니다.

## 소스 코드 내 문서 정리

PX4 개발자 여러분들에게 목적에 부합하는 소스코드 내 문서 정리를 권장합니다.

> **참고** 소스 코드 문서 표준을 강제하지 않으며, 현재 코드는 불규칙하게 문서로 정리했습니다. 이보다 더 나아지길 바랍니다!

현재 두가지 방식의 소스코드 기반 문서가 있습니다:

- `PRINT_MODULE_*` 메서드는 이 안내서에서 실행 시간의 모듈 사용 방법과 [모듈 및 명령 참고](../middleware/modules_main.md)를 목적으로 활용합니다. 
  - API는 [이곳 소스 코드에](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381) 문서로 정리했습니다. 
  - Good examples of usage include the [Application/Module Template](../apps/module_template.md) and the files linked from the modules reference.

- We encourage other in-source documentation *where it adds value/is not redundant*.
  
  > **Tip** Developers should name C++ entities (classes, functions, variables etc.) such that their purpose can be inferred - reducing the need for explicit documentation.
  
  - Do not add documentation that can trivially be assumed from C++ entity names.
  - Commonly you may want to add information about corner cases and error handling.
  - [Doxgyen](http://www.doxygen.nl/) tags should be used if documentation is needed: `@class`, `@file`, `@param`, `@return`, `@brief`, `@var`, `@see`, `@note`. A good example of usage is [src/modules/events/send_event.h](https://github.com/PX4/Firmware/blob/master/src/modules/events/send_event.h).

## Commits and Commit Messages

Please use descriptive, multi-paragraph commit messages for all non-trivial changes. Structure them well so they make sense in the one-line summary but also provide full detail.

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
    

**Use **`git commit -s`** to sign off on all of your commits.** This will add `signed-off-by:` with your name and email as the last line.

This commit guide is based on best practices for the Linux Kernel and other [projects maintained](https://github.com/torvalds/subsurface/blob/a48494d2fbed58c751e9b7e8fbff88582f9b2d02/README#L88-L115) by Linus Torvalds.