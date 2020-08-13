# 문서 기여

드론코드 프로젝트의 모든 영역을 다루는 안내서에 기여를 해주심을 매우 좋게 생각합니다. 이 내용에는 PX4, QGroundControl 개발자 및 사용자 안내서, MAVLink 안내서 내용이 들어있습니다. 여기서는 문서의 내용을 바꾸고, 내용을 추가하며, 번역문을 만드는 방법에 대해 설명합니다.

> **Note** 안내서 발전에 기여하려면 (무료) [Github](http://github.com) 계정을 만드십시오.

## 간단한 수정

오타 수정 또는 *기존 페이지*의 텍스트 편집은 간단합니다:

1. 안내서 관련 페이지 상단에 있는 **편집** 도구모음 아이콘을 누릅니다.
    
    ![Gitbook: 페이지 편집 단추](../../assets/gitbook/gitbook_toolbar_icon_edit.png)
    
    이 동작을 통해 (Github의) 편집 페이지를 엽니다.

2. 원하는 내용으로 수정합니다.

3. 페이지 하단으로 내려가면 별도의 브랜치로 만들지를 물어보고, 이 과정을 넘어가면 *pull request*를 제출하도록 안내받습니다.

문서 팀은 제출한 pull request를 검토하고 여러분이 바꾼 내용을 병합하거나 추가 작업을 진행합니다.

> 새 페이지 또는 그림을 추가하려면, github가 아닌 git 도구로 작업해야합니다. 종종 바뀐 내용을 살펴보려 할 때 gitbook 툴체인으로 라이브러리를 빌드하고 싶을 때가 있습니다.

## 새 페이지 및 그림 추가 - 규모가 큰 수정 작업

Github 인터페이스는 쉽게 안되는 새 페이지 또는 그림의 추가 작업을 진행하고 싶을 때가 있습니다. 이 경우 *코드* 변경과 동일한 방식으로 내용을 바꿔야 합니다:

1. *git* 툴체인을 사용하여 문서 소스코드를 가져옵니다
2. 필요한 부분을 수정합니다(추가, 수정, 삭제).
3. Gitbook 클라이언트로 제대로 뜨는지 시험해봅니다.
4. 여러분이 바꾼 내용에 대한 별도의 브랜치를 만들고 pull request(PR)를 만들어 문서에 끌어다놓게 합니다. 

변경 요청은 [Gitbook 편집기](https://gitbookio.gitbooks.io/documentation/content/editor/index.html)로 Gitbook 웹사이트에서 또는 로컬(더 유연하지만, 사용자에게 친숙하지 않은 방법)에서 진행할 수 있습니다. 이들 방법 대부분은 로컬에서의 설정을 다룹니다.

### 개발자 안내서인가요? 사용자 안내서인가요?

PX4 사용자에는 여러가지 유형이 있고, 문서는 제위치에 있다는 점이 중요합니다.

*개발자 안내서*는 *소프트웨어 개발자*와 관련있는 내용들로 채운 문서입니다. 이 문서의 사용자는 다음을 필요로 합니다:

* 플랫폼 기능의 추가, 수정 - 모듈, 비행 모드, 등.
* 새 하드웨어 지원 추가/통합 - 비행 컨트롤러, 주변기기, 에어프레임 등.
* 외부 소스와의 플랫폼 통신 - 예: 보조 컴퓨터.
* 아키텍처의 이해

반면에 *사용자 안내서*의 사용자는 *기초적으로* 다음을 필요로 합니다:

* PX4로 비행체를 날리기
* 지원/기존 에어프레임에 PX4로 비행체를 구성하고, 수정하고, 설정합니다.

> **Tip** 예를 들어 기존 에어프레임을 빌드하고 설정하는 자세한 정보는 사용자 안내서에 있지만, *새* 에어프레임을 지정하는 방법은 개발자 안내서에 있습니다.

### 문서 소스 코드 가져오고(Get) 밀어올리기(Push)

라이브러리 원본 코드를 로컬 컴퓨터에 가져오려면 git 툴체인이 있어야합니다. 아래 절차는 로컬 컴퓨터에 git을 가져다 사용하는 방법을 설명합니다.

1. git 을 <https://git-scm.com/downloads>에서 컴퓨터로 다운로드하십시오.
2. 아직 Github에 계정이 없으면 [가입](https://github.com/join)하십시오.
3. Github에서 원하는 라이브러리의 사본을 여러분의 계정으로 복사하여 만(Fork)드십시오([절차는 여기에 있음](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository)). 저장소 라이브러리 URL은 다음과 같습니다: 
    * PX4 사용자 안내서: https://github.com/PX4/px4_user_guide
    * PX4 개발자 안내서: https://github.com/PX4/Devguide
    * QGroundControl 사용자 안내서: https://github.com/mavlink/qgc-user-guide
    * QGroundControl 개발자 안내서: https://github.com/mavlink/qgc-dev-guide
    * MAVLink 개발자 안내서: https://github.com/mavlink/mavlink-devguide
4. 복제한 저장소를 로컬 컴퓨터에 가져오십시오: 
        sh
        cd ~/wherever/
        git clone https://github.com/<your git name>/<repository_name>.git 예를 들어, PX4 사용자 안내서 복제본을 "john_citizen_smith" github 계정에서 가져오려면: 
    
        sh
        git clone https://github.com/john_citizen_smith/px4_user_guide.git

5. 로컬 저장소를 탐색합니다(아래에서는 px4_user_guide 디렉터리 활용): 
        sh
        cd ~/wherever/px4_user_guide

6. "업스트림"으로 부르는 *remote* 지점을 만들어 원본 라이브러리를 가리킵니다. 아래 예제에서는 사용자 안내서에 대해 수행하는 방법을 보여줍니다(URL 형식을 참고 - ".git" 확장자를 가지는 저장소 URL 입니다).
    
    ```sh
    git remote add upstream https://github.com/PX4/px4_user_guide.git
    ```
    
    > **Tip** "remote" 은 저장소 일부의 핸들입니다. *origin* remove는 저장소를 복제할 때 기본으로 만드는 항목이며, 여러분이 여러분의 계정으로 포크한 안내서 저장소를 가리킵니다. 문서의 공식 버전을 가리키는 새 원격 *업스트림* 을 만들고 싶을 것입니다.

7. 여러분이 바꾼 내용에 대한 브랜치를 새로 만드십시오:
    
    ```sh
    git checkout -b <your_feature_branch_name>
    ```
    
    이 명령으로 `your_feature_branch_name` 이름의 로컬 브랜치를 만듭니다.

8. 문서에서 필요한 부분을 바꾸십시오(이 방법에 대한 일반 지침은 다음 절에 있습니다)
9. 바꾼 내용에 만족스럽다면, 해당 내용을 "commit" 명령으로 로컬 브랜치에 추가할 수 있습니다: 
        sh
        git add <file name>
        git commit -m "<your commit message>" 바람직한 제출 메시지에 대해서는 
    
    [기여](../contribute/README.md) 절을 참고하십시오.
10. 로컬 브랜치(추가한 제출 사항)를 여러분의 github 계정으로 push 하십시오. 
        sh
        git push origin your_feature_branch_name

11. 웹 브라우저에서 `https://github.com/<your git name>/px4_user_guide.git`와 같이 Github의 복제 라이브러리로 이동하십시오. 이 과정을 통해 복제한 원격 저장소로 새 브랜치를 밀어 올렸다는 메시지가 나타나야합니다.
12. pull 요청(PR) 만들기: 
    * "new branch message" 우측을 보면(한단계 전), "Compare & Create Pull Request"가 적힌 녹색 단추를 볼 수 있습니다. 그 단추를누르십시오.
    * pull 요청 양식이 나타납니다. 제출한 변경 내역을 나타낼텐데, 의미있는 제목(커밋 하나를 가진 PR의 경우 커밋 메시지)과 메시지를 추가할 수 있습니다(<span style="color:orange">어떤 이유로 무엇을 했는지 설명하십시오.</span> [다른 pull 요청](https://github.com/PX4/px4_user_guide/pulls)을 보고 비교해보십시오)
13. 이제 다 끝났습니다. PX4 문서 담당자가 기여 상황을 살펴보고 병합을 할 지 말지를 결정하게 됩니다. 그동안 바뀐 내용에 대해 질문이 있을지 한번 정도는 확인해보십시오.

### 소스 코드 구성

이 안내서는 [예전 Gitbook 툴체인](https://legacy.gitbook.com/) 을 활용합니다. 툴체인 설치 활용 방법은 [툴체인 문서](https://github.com/GitbookIO/gitbook/blob/master/docs/setup.md)에서 찾을 수 있습니다.

개요에서는:

* 마크다운 문법을 활용하여 개별 파일로 페이지를 작성했습니다 \(Github 위키에서 활용하는 동일한 문법을 사용함\). 
* 책의 *구성* 내용은 **SUMMARY.md</0> 파일에 있습니다. 라이브러리에 새 페이지를 추가하려면 이 파일에 항목을 추가해야합니다. </li> 
    
    * 이 책은 [다국어](https://github.com/GitbookIO/gitbook/blob/master/docs/languages.md) 서적이기 때문에 루트 디렉터리의 **LANGS.md**에 어떤 언어를 지원하는지 명시합니다. 
        * 각 언어의 페이지는 관련 언어 코드로 이름이 붙은 폴더에 저장합니다 \(예: "zh"는 중문, "en"은 영문\).
        * 여러 파일 중 영문 버전만 편집해야합니다. 다른 트리를 관리할 때는 번역 프로그램을 활용합니다.
    * 그림은 **/assets** 의 하위 폴더에 저장해야합니다. 내용 폴더에 폴더가 두개가 있기 때문에, 그림을 추가하려면 다음과 같이 참조합니다: ```![그림이다](../../assets/path_to_file/filename.jpg)```
    * **book.json** 파일은 빌드 의존 요소를 지정합니다.
    * 웹 훅은 이 저장소의 마스터 브랜치로 파일을 병합할 경우 이를 추적하여 책을 다시 빌드하려고 활용합니다.</ul> 
    
    ### 방식 안내
    
    1. 파일 이름 
    
    * 적당한 하위 폴더에 새 파일을 추가하십시오
    * 이해할 수 있는 이름을 사용하십시오. 일례를 들어, 그림 파일 이름은 어떤 그림이 있는지 설명해야합니다.
    * 소문자를 사용하고 밑줄 문자 "\_"를 활용하여 단어를 구분하십시오
    
    2. 그림
    
    * 최대한 용량은 작게, 쓸만할 정도로 해상도를 낮춘 이미지를 사용하십시오.
    * 새 그림은 기본적으로 **/assets/** 하위 폴더에 만들어야합니다(그래야 번역에서도 쓸 수 있습니다).
    
    3. 내용:
    
    * "모양새" \(굵게, 강조(이탤릭) 등\)를 일관되게 활용하십시오. **굵게** 는 누르는 단추 텍스트와 메뉴 정의에 활용합니다. *강조*는 도구 이름에 사용합니다. 다른 경우에는 가급적 드물게 활용하십시오.
    * 소제목과 페이지 제목은 "첫글자를 대문자로" 해야 합니다.
    * 페이지 제목은 처음 수준 제목 \(\#\) 형식이어야 합니다. 다른 소제목은 h2 \(\#\#\) 또는 그 이하여야 합니다.
    * 제목에는 모양새를 덧붙이지 마십시오.
    * Note, Tip, Warning의 *첫 부분*은 번역하지 마십시오(예: `> **Note**`). 참고 내용을 제대로 표시하려면 있는 그대로의 텍스트가 필요합니다.
    
    ### Gitbook을 로컬에서 빌드하기
    
    Gitbook을 설치하고 빌드하는 모든 필요한 과정은 [툴체인 문서](https://github.com/GitbookIO/gitbook/blob/master/docs/setup.md)에도 있습니다.
    
    1. nodejs를 컴퓨터에 설치하십시오(4~6 버전 추천).
    2. gitbook을 NPM으로 설치하십시오. 터미널 프롬프트에서 간단하게 다음 명령을 내리면 GitBook을 설치합니다: 
            sh
            npm install gitbook-cli -g
    
    3. 로컬 저장소를 탐색하십시오:
        
        ```sh
        cd ~/wherever/px4_user_guide
        ```
        
        gitbook 의존 요소 하나를 설치하십시오:
        
        ```sh
        gitbook install
        ```
        
        > **Note** `/usr/bin/env: node: No such file or directory` 오류 상황에 빠졌다면 `ln -s /usr/bin/nodejs /usr/bin/node` 명령을 실행하십시오.
    
    4. 책을 빌드하십시오:
        
        ```sh
        gitbook build
        ```
    
    5. 책을 미리 보고 원격 공유하면 다음 명령을 실행하십시오: 
            sh
            gitbook serve
        
        * 이제 http://localhost:4000/ 에서 로컬 책을 탐색할 수 있습니다.
        * 원격 공개 상태에서 빠져나가려면 터미널 프롬프트에서 `CTRL+c` 키를 누르십시오.
    6. 4000번 대신 다른 포트 번호로 책을 원격 공개할 수 있습니다: 
            sh
            gitbook serve --port 4003
    
    7. html, pdf, epub, mobi로 내보낼 수도 있습니다: 
            sh
            gitbook help
    
    ## 번역 {#translation}
    
    *QGroundControl*과 PX4, *QGroundControl*, MAVLink 안내서 번역의 도움을 간절히 바랍니다. 자세한 정보는 [Translation](../contribute/translation.md)을 살펴보십시오.
    
    ## 라이선스
    
    모든 PX4/드론코드 문서는 [크리에이티브 커먼즈 저작자표시 4.0](https://creativecommons.org/licenses/by/4.0/) 라이선스의 허여 조항에 따라 자유롭게 활용하고 수정할 수 있습니다.