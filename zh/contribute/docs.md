# 投稿指南

非常欢迎大家给Dronecode项目的相关指南积极投稿。 内容包括PX4和QGroundControl 开发者和用户指南，以及MAVLink指南。 本篇章用来向投稿者们解释如何进行更改、添加内容和创建翻译。

> **注意**你需要一个(免费的) [Github](http://github.com)帐户来为指南作出贡献。

## 快速更改

如需修改错误或编辑一个*现有页面*，请按如下步骤操作：

1. 点击指南相关页面顶部的工具栏图标**编辑**。
    
    ![Gitbook: 编辑页面按钮](../../assets/gitbook/gitbook_toolbar_icon_edit.png)
    
    打开编辑页面(Github)。

2. 进行修改。

3. 页面底部会提示投稿者创建一个单独的分支，然后 引导其提交一个*拉取请求*。

文档团队通过审核已提交的合并请求，决定直接合并文档进行更新，或是与投稿者再次沟通进行更新。

## 添加新内容 - 大更改

### 内容分类

*开发者指南*是与*软件开发者相关的文档*。 使用者的需求为：

* 添加或修改软件平台功能，如模块、飞行模式等。
* 添加新硬件支持/集成，如飞行控制器、外围、机型等。
* 从外部源与平台通信，例如一个配套的计算机。
* 了解软件架构

*用户指南*的使用者，需求为：

* 使用 PX4 控制飞行器
* 使用 PX4，基于已支持/现存的机型，构建、修改或配置类似载具。

> **T提示** 例如，关于如何构建/配置现有机架的详细信息在用户指南中，而 定义*新*机架类型的说明在开发者指南中。

### Gitbook 文档工具

指南使用 [旧版Gitbook 工具链](https://legacy.gitbook.com/)

更改请求可以使用 [Gitbook 编辑器](https://gitbookio.gitbooks.io/documentation/content/editor/index.html) 在 Gitbook 网站上完成，也可以在本地完成(更灵活，但用户友好度欠佳)。

In order to contribute many changes to the documentation, it is recommended that you follow these steps to add the changes locally and then create a pull request:

* [Sign up](https://github.com/join) for github if you haven't already
* Fork the PX4 user guide from [here](https://github.com/PX4/px4_user_guide) or Dev guide from [here](https://github.com/PX4/Devguide). For instructions to fork a git repository, see [here](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository).
* Clone your forked repository to your local computer  
        sh
        cd ~/wherever/
        git clone https://github.com/<your git name>/px4_user_guide.git

* Install gitbook via NPM. At the terminal prompt, simply run the following command to install GitBook:
    
    ```sh
    npm install gitbook-cli -g
    ```
    
    > **Note** Everything you need to install and build Gitbook locally is also explained in the [toolchain documentation](https://github.com/GitbookIO/gitbook/blob/master/docs/setup.md).

* Navigate to your local repository and add original upstream:
    
    ```sh
    cd ~/wherever/px4_user_guide
    git remote add upstream https://github.com/PX4/px4_user_guide.git
    ```

* Now you can checkout a new branch and add your changes. To build your book, run:
    
    ```sh
    gitbook build
    ```
    
    > **Note** If you run into an error: `/usr/bin/env: node: No such file or directory`, run `ln -s /usr/bin/nodejs /usr/bin/node`

* To preview and serve your book, run:
    
    ```sh
    gitbook serve
    ```
    
    > **Note** run `gitbook install` to install missing plugins.

* Now you can browse your local book on http://localhost:4000/

* Exit serving using `CTRL+c` in the terminal prompt.

* You can also serve on a different port instead of 4000:
    
    ```sh
    gitbook serve --port 4003
    ```

* You can also output as html, pdf, epub or mobi: 
        sh
        gitbook help

* Once you are satisfied with your changes after previewing them, you can add and commit them:
    
    ```sh
    git add <file name>
    git commit -m "<your commit message>"
    ```
    
    For a good commit message, please refer to [Contributing](../contribute/README.md) section.

* Now you can push your local commits to your forked repository
    
    ```sh
    git push origin <your feature branch name>
    ```

* You can verify that the push was successful by going to your forked repository in your browser: ```https://github.com/<your git name>/px4_user_guide.git```  
    There you should see the message that a new branch has been pushed to your forked repository.
* Now it's time to create a pull request (PR). On the right hand side of the "new branch message" (see one step before), you should see a green button saying "Compare & Create Pull Request". Then it should list your changes and you can (must) add a meaningful title (in case of a one commit PR, it's usually the commit message) and message (<span style="color:orange">explain what you did for what reason</span>. Check [other pull requests](https://github.com/PX4/px4_user_guide/pulls) for comparison)
* You're done! Responsible members of PX4 guides will now have a look at your contribution and decide if they want to integrate it. Check if they have questions on your changes every once in a while.

In overview:

* Pages are written in separate files using markdown \(almost the same syntax used by Github wiki\). 
* The *structure* of the book is defined in a file named **SUMMARY.md**.
* This is a [multilingual](https://github.com/GitbookIO/gitbook/blob/master/docs/languages.md) book, so there is a **LANGS.md** file in the root directory defining what languages are supported. Pages for each language are stored in the folder named for the associated language code \(e.g. "zh" for Chinese, "en" for English\). 
* A file named **book.json** defines any dependencies of the build.
* A web hook is used to track whenever files are merged into the master branch on this repository, causing the book to rebuild.

## Style guide

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

## 翻译 {#translation}

We'd love your help to translate *QGroundControl* and our guides for PX4, *QGroundControl* and MAVLink. For more information see: [Translation](../contribute/translation.md).

## 许可证

All PX4/Dronecode documentation is free to use and modify under terms of the permissive [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) licence.