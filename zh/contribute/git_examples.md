# GIT 示例

## 为 PX4 贡献代码

按照定义的工作流向 px4 添加功能。 为了在 px4 上分享您的贡献, 您可以遵循此示例。

* 如果您还没有注册，请先[Sign up](https://github.com/join) Github 账户
* 创建固件分支 (见 [here](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository))
* 将分支克隆到本地计算机  
    

```sh
cd ~/wherever/
git clone https://github.com/<your git name>/Firmware.git
```

* 进入新目录, 初始化和更新子模块, 并添加原始上游固件  
    

```sh
cd Firmware
git submodule update --init --recursive
git remote add upstream https://github.com/PX4/Firmware.git
```

* 现在, 您应该有两个远程存储库: 一个存储库被称为上游存储库, 指向 px4 固件, 另一个存储库指向您的 px4 存储库分支存储库。
* 这可以通过以下命令进行检查:

```sh
git remote -v
```

* 进行要添加到当前 master 的更改。
* 使用代表您的功能的有意义的名称创建一个新分支  
    

```sh
git checkout -b <your feature branch name>
```

您可以使用命令 ```git branch``` 以确保你在正确的分支上 * 通过添加相应的文件添加您希望成为提交的一部分的更改  


```sh
git add <file name>
```

如果您希望有一个 GUI 来添加您的文件, 请参阅 [Gitk](https://git-scm.com/book/en/v2/Git-in-Other-Environments-Graphical-Interfaces) 或 [

    git add -p](http://nuclearsquid.com/writings/git-add/) * 提交添加的文件, 并顺便记录一条有意义的消息, 解释您的更改

  


```sh
git commit -m "<your commit message>"
```

有关良好的提交消息, 请参阅 [Contributing](../contribute/README.md) 部分。 * 可能已经过去一段时间，[upstream master](https://github.com/PX4/Firmware.git) 已经改变。 PX4 更喜欢线性提交历史记录, 并使用 [git rebase](https://git-scm.com/book/de/v1/Git-Branching-Rebasing)。 要在本地分支中包含上游的最新更改, 请切换到主分支  


```sh
git checkout master
```

然后从上游 master 中提取最新的提交  


```sh
git pull upstream master
```

现在你本地的 master 是最新的。 切换回您的功能分支  


```sh
git checkout <your feature branch name>
```

并根据您更新的母版重新定位  


```sh
git rebase master
```

* 现在, 您可以将本地提交推送到分支存储库  
    

```sh
git push origin <your feature branch name>
```

* 您可以通过在浏览器中转到分叉存储库来验证推送是否成功： ```https://github.com/<your git name>/Firmware.git```  
    您应该会看到一条消息, 即一个新分支已被推送到分叉存储库。
* 现在是时候创建一个拉取请求 (PR) 了。 在 "新分支消息" 的右侧 (请参阅前面的一个步骤), 您应该看到一个绿色按钮, 上面写着 "比较 & amp; 创建拉取请求"。 然后, 它应该列出你的更改, 你必须添加一个有意义的标题 (在一个提交 PR 的情况下, 它通常是提交消息) 和消息 (<span style="color:orange">解释你做了这些的原因 </span>， 检查 [其他拉取请求 ](https://github.com/PX4/Firmware/pulls) 进行比较)。
* 搞定！ PX4 负责成员现在将看看你的贡献, 并决定他们是否要整合它。 每过一段时间检查你的更改看是否会有问题。

## 更新子模块

有几种方法可以更新子模块。 您可以克隆存储库, 也可以进入子模块目录, 并按照 [Contributing code to PX4](#Contributing-code-to-PX4) 中的步骤操作。

## 为子模块更新执行 PR

这是在您为子模块 x 存储库做了 PR 之后所必需的, 并且错误修复/功能添加在子模块 x 的当前主控件中。由于固件仍指向更新之前的提交, 因此需要一个子模块拉取请求, 以便固件使用的子模块指向最新提交。

```sh
cd Firmware
```

* 创建一个分支，描述子模块更新的 bug 修复/功能：

```sh
git checkout -b pr-some-fix
```

* 进入子模块的子目录

```sh
cd <path to submodule>
```

* PX4 子模块可能并不一定指向最新的更新。 因此，先 checkout master 并且拉取最新的上游代码。

```sh
git checkout master
git pull upstream master
```

* 回到 Firmware 目录，如往常一样添加、提交和上推更改。

```sh
cd -
git add <path to submodule>
git commit -m "Update submodule to include ..."
git push upstream pr-some-fix
```

## 查看拉取请求

你可以测试某人的拉取请求（更改尚未整合），即使要整合的分支只存在于他的拷贝中。 执行以下指令：

```sh
git fetch upstream  pull/<PR ID>/head:<branch name>
```

    PR ID 是 PR 标题旁边的数字 (不包括 #) 和 ```<branch name>``` 也可以在下面找到 ```PR ID```, 例如: ```<the other persons git name>:<branch name>```. 之后, 您可以看到新创建的分支在本地

```sh
git branch
```

然后切换到那个分支

```sh
git checkout <branch name>
```

## 常见错误

### 强制推送到分叉存储库

做完第一个 PR 后, 来自 PX4 社区的人将回顾你的更改。 在大多数情况下, 这意味着您必须根据评审来修复本地分支。 在本地更改文件后, 需要使用最新的 uperecle1 主服务器重新定位功能分支。 但是, 在重新建立基础后, 不再可能将特征分支直接推送到分叉存储库, 而是需要使用强制推送:

```sh
git push --force-with-lease origin <your feature branch name>
```

### 重新建立合并冲突

如果冲突发生在 ```git rebase```，请参阅 [this guide](https://help.github.com/articles/resolving-merge-conflicts-after-a-git-rebase/)。

### 拉取合并冲突

如果冲突发生在 ```git pull```，请参阅 [this guide](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/#competing-line-change-merge-conflicts)。