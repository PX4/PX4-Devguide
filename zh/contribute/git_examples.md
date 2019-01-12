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

For a good commit message, please refer to [Contributing](../contribute/README.md) section. * Some time might have passed and the [upstream master](https://github.com/PX4/Firmware.git) has changed. PX4 prefers a linear commit history and uses [git rebase](https://git-scm.com/book/de/v1/Git-Branching-Rebasing). To include the newest changes from upstream in your local branch, switch to your master branch  


```sh
git checkout master
```

Then pull the newest commits from upstream master  


```sh
git pull upstream master
```

Now your local master is up to date. Switch back to your feature branch  


```sh
git checkout <your feature branch name>
```

and rebase on your updated master  


```sh
git rebase master
```

* Now you can push your local commits to your forked repository  
    

```sh
git push origin <your feature branch name>
```

* You can verify that the push was successful by going to your forked repository in your browser: ```https://github.com/<your git name>/Firmware.git```  
    There you should see the message that a new branch has been pushed to your forked repository.
* Now it's time to create a pull request (PR). On the right hand side of the "new branch message" (see one step before), you should see a green button saying "Compare & Create Pull Request". Then it should list your changes and you can (must) add a meaningful title (in case of a one commit PR, it's usually the commit message) and message (<span style="color:orange">explain what you did for what reason</span>. Check [other pull requests](https://github.com/PX4/Firmware/pulls) for comparison)
* You're done! Responsible members of PX4 will now have a look at your contribution and decide if they want to integrate it. Check if they have questions on your changes every once in a while.

## 更新子模块

There are several ways to update a submodule. Either you clone the repository or you go in the submodule directory and follow the same procedure as in [Contributing code to PX4](#Contributing-code-to-PX4).

## Do a PR for a submodule update

This is required after you have done a PR for a submodule X repository and the bug-fix / feature-add is in the current master of submodule X. Since the Firmware still points to a commit before your update, a submodule pull request is required such that the submodule used by the Firmware points to the newest commit.

```sh
cd Firmware
```

* Make a new branch that describes the fix / feature for the submodule update:

```sh
git checkout -b pr-some-fix
```

* Go to submodule subdirectory

```sh
cd <path to submodule>
```

* PX4 submodule might not necessarily point to the newest commit. Therefore, first checkout master and pull the newest upstream code.

```sh
git checkout master
git pull upstream master
```

* Go back to Firmware directory, and as usual add, commit and push the changes.

```sh
cd -
git add <path to submodule>
git commit -m "Update submodule to include ..."
git push upstream pr-some-fix
```

## Checkout pull requests

You can test someone's pull request (changes are not yet merged) even if the branch to merge only exists on the fork from that person. Do the following

```sh
git fetch upstream  pull/<PR ID>/head:<branch name>
```

    PR ID is the number right next to the PR's title (without the #) and the ```<branch name>``` can also be found right below the ```PR ID```, e.g. ```<the other persons git name>:<branch name>```. After that you can see the newly created branch locally with

```sh
git branch
```

Then switch to that branch

```sh
git checkout <branch name>
```

## Common pitfalls

### Force push to forked repository

After having done the first PR, people from the PX4 community will review your changes. In most cases this means that you have to fix your local branch according to the review. After changing the files locally, the feature branch needs to be rebased again with the most recent upstream/master. However, after the rebase, it is no longer possible to push the feature branch to your forked repository directly, but instead you need to use a force push:

```sh
git push --force-with-lease origin <your feature branch name>
```

### Rebase merge conflicts

If a conflict occurs during a ```git rebase```, please refer to [this guide](https://help.github.com/articles/resolving-merge-conflicts-after-a-git-rebase/).

### Pull merge conflicts

If a conflict occurs during a ```git pull```, please refer to [this guide](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/#competing-line-change-merge-conflicts).