# 문서 기여

드론코드 프로젝트의 모든 영역을 다루는 안내서에 기여를 해주심을 매우 좋게 생각합니다. 이 내용에는 PX4, QGroundControl 개발자 및 사용자 안내서, MAVLink 안내서를 포함합니다. 여기서는 문서의 내용을 바꾸고, 내용을 추가하며, 번역문을 만드는 방법에 대해 설명합니다.

> **Note** 안내서 발전에 기여하려면 (무료) [Github](http://github.com) 계정을 만드십시오.

## 간단한 수정

오타 수정 또는 *기존 페이지*의 텍스트 편집은 간단합니다:

1. 안내서 관련 페이지 상단에 있는 **편집** 도구모음 아이콘을 누릅니다.
    
    ![Gitbook: Edit Page button](../../assets/gitbook/gitbook_toolbar_icon_edit.png)
    
    (Github의) 편집 페이지를 엽니다.

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
    
    This creates a local branch on your computer named `your_feature_branch_name`.

8. Make changes to the documentation as needed (general guidance on this in following sections)
9. Once you are satisfied with your changes, you can add them to your local branch using a "commit": 
        sh
        git add <file name>
        git commit -m "<your commit message>" For a good commit message, please refer to 
    
    [Contributing](../contribute/README.md) section.
10. Push your local branch (including commits added to it) to your forked repository on github 
        sh
        git push origin your_feature_branch_name

11. Go to your forked repository on Github in a web browser, e.g.: `https://github.com/<your git name>/px4_user_guide.git`. There you should see the message that a new branch has been pushed to your forked repository.
12. Create a pull request (PR): 
    * On the right hand side of the "new branch message" (see one step before), you should see a green button saying "Compare & Create Pull Request". Press it.
    * A pull request template will be created. It will list your commits and you can (must) add a meaningful title (in case of a one commit PR, it's usually the commit message) and message (<span style="color:orange">explain what you did for what reason</span>. Check [other pull requests](https://github.com/PX4/px4_user_guide/pulls) for comparison)
13. You're done! Responsible members of PX4 guides will now have a look at your contribution and decide if they want to integrate it. Check if they have questions on your changes every once in a while.

### Source Code Structure

The guide uses the [Legacy Gitbook Toolchain](https://legacy.gitbook.com/) toolchain. Instructions for how this toolchain is setup and used can be found in the [toolchain documentation](https://github.com/GitbookIO/gitbook/blob/master/docs/setup.md).

In overview:

* Pages are written in separate files using markdown \(almost the same syntax used by Github wiki\). 
* The *structure* of the book is defined in a file named **SUMMARY.md**. If you add a new page to the library you must add an entry to this file. 
* This is a [multilingual](https://github.com/GitbookIO/gitbook/blob/master/docs/languages.md) book, so there is a **LANGS.md** file in the root directory defining what languages are supported. 
    * Pages for each language are stored in the folder named for the associated language code \(e.g. "zh" for Chinese, "en" for English\).
    * You should only ever edit the ENGLISH version of files. We use translation software to manage the other trees.
* Images must be stored in a sub folder of **/assets**. This is two folders down from content folders, so if you add an image you will reference it like: ```![Image Description](../../assets/path_to_file/filename.jpg)```
* A file named **book.json** defines any dependencies of the build.
* A web hook is used to track whenever files are merged into the master branch on this repository, causing the book to rebuild.

### 방식 안내

1. 파일 이름 

* 적당한 하위 폴더에 새 파일을 추가하십시오
* 이해할 수 있는 이름을 사용하십시오. 일례를 들어, 그림 파일 이름은 어떤 그림이 있는지 설명해야합니다.
* 소문자를 사용하고 밑줄 문자 "\_"를 활용하여 단어를 구분하십시오

2. 그림

* 최대한 용량은 작게, 쓸만할 정도로 해상도를 낮춘 이미지를 사용하십시오.
* 새 그림은 기본적으로 **/assets/** 하위 폴더에 만들어야합니다(그래야 번역에서도 쓸 수 있습니다).

3. 내용:

* "모양새" \(굵게, 강조(이탤릭) 등\)를 일관되게 활용하십시오. **굵게** 는 누르는 단추 텍스트와 메뉴 정의에 활용합니다. *Emphasis* for tool names. Otherwise use as little as possible.
* Headings and page titles should use "First Letter Capitalisation"
* The page title should be a first level heading \(\#\). All other headings should be h2 \(\#\#\) or lower.
* Don't add any style to headings.
* Don't translate the *first part* of a note, tip or warning declaration (e.g. `> **Note**`) as this precise text is required to render the note properly.

### Building the Gitbook Locally

Everything you need to install and build Gitbook locally is also explained in the [toolchain documentation](https://github.com/GitbookIO/gitbook/blob/master/docs/setup.md).

1. Install nodejs on your computer (version 4-6 recommended).
2. Install gitbook via NPM. At the terminal prompt, simply run the following command to install GitBook: 
        sh
        npm install gitbook-cli -g

3. Navigate to your local repository:
    
    ```sh
    cd ~/wherever/px4_user_guide
    ```
    
    1 Install gitbook dependencies:
    
    ```sh
    gitbook install
    ```
    
    > **Note** If you run into an error: `/usr/bin/env: node: No such file or directory`, run `ln -s /usr/bin/nodejs /usr/bin/node`

4. Build your book:
    
    ```sh
    gitbook build
    ```

5. To preview and serve your book, run: 
        sh
        gitbook serve
    
    * Now you can browse your local book on http://localhost:4000/
    * Exit serving using `CTRL+c` in the terminal prompt.
6. You can also serve on a different port instead of 4000: 
        sh
        gitbook serve --port 4003

7. You can also output as html, pdf, epub or mobi: 
        sh
        gitbook help

## Translations {#translation}

We'd love your help to translate *QGroundControl* and our guides for PX4, *QGroundControl* and MAVLink. For more information see: [Translation](../contribute/translation.md).

## Licence

All PX4/Dronecode documentation is free to use and modify under terms of the permissive [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) licence.