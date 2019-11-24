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

为了有助于对文档进行诸多改动， 请按照如下步骤添加本地更改，然后创建合并请求：

* 如果您还没有注册Github，请先[注册](https://github.com/join) Github 账户
* 点击[这里](https://github.com/PX4/px4_user_guide)查看PX4用户指南，点击[这里](https://github.com/PX4/Devguide)查看PX4开发者指南。 点击[这里](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository)，查看如何分支git repository。
* 将分支克隆到本地计算机  
        sh
        cd ~/wherever/
        git clone https://github.com/<your git name>/px4_user_guide.git

* 通过 NPM 安装 gitbook 在终端提示下，只需运行以下命令即可安装 GitBook：
    
    ```sh
    npm install gitbook-cli -g
    ```
    
    > **注意** 在本地安装和构建Gitbook 的所需一切也会在[工具链文档](https://github.com/GitbookIO/gitbook/blob/master/docs/setup.md)中说明。

* 导航到您的本地版本库并添加原始的上游版本：
    
    ```sh
    cd ~/wherever/px4_user_guide
    git remote add upstream https://github.com/PX4/px4_user_guide.git
    ```

* 现在您可以检出一个新分支，并向其中添加您的更改。 要构建您的卷册，请运行：
    
    ```sh
    gitbook build
    ```
    
    > **注意** 如果遇到报错: `/usr/bin/env: node：No such file or directory`, 请运行 `ln -s /usr/bin/nodejs /usr/bin/node`

* 要预览并服务您的卷册，请运行：
    
    ```sh
    gitbook serve
    ```
    
    > **注意** 运行 `gitbook install` 可安装缺失的插件。

* 现在您可以在 http://localhost:4000/ 上浏览您的本地卷册

* 在终端提示中使用`CTRL+c`退出。

* 除了端口4000，您也可以在另一个端口上作业：
    
    ```sh
    gitbook serve --port 4003
    ```

* 您也可以以 html 、pdf、epub 或 mobi 格式输出： 
        sh
        gitbook help

* 预览过您的更改后，只要您感觉满意，您便可以添加并提交这些更改：
    
    ```sh
    git add <file name>
    git commit -m "<your commit message>"
    ```
    
    需进一步了解提交信息, 请参阅 [Contributing](../contribute/README.md) 部分。

* 现在, 您可以将本地提交推送到分支版本库
    
    ```sh
    git push origin <your feature branch name>
    ```

* 您可以通过浏览器访问分支版本库来验证推送是否成功： ```https://github.com/<your git name>/px4_user_guide.git```  
    您应该会看到一条告知消息：一个新分支已被推送到您的分支版本库。
* 现在是时候创建一个拉取请求 (PR) 了。 在 "新分支消息" 的右侧 (请参阅前面的一个步骤), 您应该看到一个绿色按钮, 上面写着 "Compare & Create Pull Request"。 然后, 它应该列出你的更改，你必须添加一个有意义的标题 (在提交 PR 的情况下, 它通常是提交消息) 和消息 (<span style="color:orange">解释你做了这些更改的原因 </span>， 检查 [其他拉取请求 ](https://github.com/PX4/px4_user_guide/pulls) 进行比较)。
* 搞定！ PX4 指南项目的负责人现在将看到您的投稿, 并决定是否要整合稿件内容。 每过一段时间，他们会检查你的更改，以确保没有疑问。

概述：

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