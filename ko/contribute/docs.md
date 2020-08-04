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

Change requests can be either done on the Gitbook website using the [Gitbook editor](https://gitbookio.gitbooks.io/documentation/content/editor/index.html) or locally (more flexible, but less user-friendly). Most of these instructions cover the local setup.

### Developer or User Guide?

There are different types of PX4 users, and it is important that documentation goes into the right place.

The *Developer Guide* is for documentation that is relevant to *software developers*. This includes users who need to:

* Add or modify platform features - modules, flight modes, etc.
* Add support/integrate with new hardware - flight controllers, peripherals, airframes, etc.
* Communicate with the platform from an external source - e.g. a companion computer.
* Understand the architecture

The *User Guide*, by contrast, is *primarily* for users who want to:

* Fly a vehicle using PX4
* Build, modify, or configure a vehicle using PX4 on a supported/existing airframe.

> **Tip** For example, detailed information about how to build/configure an existing airframe are in the User Guide, while instructions for defining a *new* airframe are in the Developer Guide.

### Get/Push Documentation Source Code

To get the library(s) sources onto your local computer you will need to use the git toolchain. The instructions below explain how to get git and use it on your local computer.

1. Download git for your computer from <https://git-scm.com/downloads>
2. [Sign up](https://github.com/join) for github if you haven't already
3. Create a copy (Fork) of the desired library on Github ([instructions here](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository)). The library repo URLs are: 
    * PX4 User Guide: https://github.com/PX4/px4_user_guide
    * PX4 Developer Guide: https://github.com/PX4/Devguide
    * QGroundControl User Guide: https://github.com/mavlink/qgc-user-guide
    * QGroundControl Developer Guide: https://github.com/mavlink/qgc-dev-guide
    * MAVLink Developer Guide: https://github.com/mavlink/mavlink-devguide
4. Clone (copy) your forked repository to your local computer: 
        sh
        cd ~/wherever/
        git clone https://github.com/<your git name>/<repository_name>.git For example, to clone the PX4 userguide fork for a user with github account "john_citizen_smith": 
    
        sh
        git clone https://github.com/john_citizen_smith/px4_user_guide.git

5. Navigate to your local repository (px4_user_guide is used below): 
        sh
        cd ~/wherever/px4_user_guide

6. Add a *remote* called "upstream" to point to the original library. The exmaple below shows how to do this for the user guide (note the URL format - it is the repo URL with extension ".git").
    
    ```sh
    git remote add upstream https://github.com/PX4/px4_user_guide.git
    ```
    
    > **Tip** A "remote" is a handle to a particular repository. The remote named *origin* is created by default when you clone the repository, and points to your fork of the guide. You want to create a new remote *upstream* that points to the official version of the document.

7. Create a branch for your changes:
    
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

### Style guide

1. Files/file names

* Put new files in an appropriate sub-folder
* Use descriptive names. In particular, image filename should describe what they contain.
* Use lower case and separate words using underscores "\_"

2. Images

* Use the smallest size and lowest resolution that makes the image still useful.
* New images should be created in a sub-folder of **/assets/** by default (so they can be shared between translations).

3. Content:

* Use "style" \(bold, emphasis, etc\) consistently. **Bold** for button presses and menu definitions. *Emphasis* for tool names. Otherwise use as little as possible.
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