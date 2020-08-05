# git 예제

## PX4에 코드 기여하기 {#contributing_code}

PX4로의 기능 추가는 다음 절차를 따릅니다. 다음 예제를 따라 PX4에 기여 결과를 공유할 수 있습니다.

* 아직 github 계정이 없다면 [가입](https://github.com/join) 하십시오
* 펌웨어를 별도로 복제(fork)하십시오([이곳](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository) 참고)
* 여러분의 계정으로 복제(fork)한 저장소를 로컬 컴퓨터로 가져오십시오  
        sh
        cd ~/wherever/
        git clone https://github.com/<your git name>/Firmware.git

* 새 디렉터리로 이동, 초기화, 하위 모듈을 업데이트한 후, 원본 업스트림 펌웨어를 추가하십시오  
        sh
        cd Firmware
        git submodule update --init --recursive
        git remote add upstream https://github.com/PX4/Firmware.git

* 이제 원격 저장소가 둘이 됐습니다. 하나는 PX4 펌웨어를 가리키는 업스트림 저장소이며, 다른 하나는 PX4 저장소에서 복제(fork)한 저장소를 가리키는 저장소입니다.
* 다음 명령으로 이를 확인해볼 수 있습니다: 
        sh
        git remote -v

* 현재 마스터 브랜치에 추가하고자 하는 내용을 수정하십시오.
* 여러분이 구현한 기능을 설명하는 의미있는 이름으로 새 브랜치를 만드십시오.  
        sh
        git checkout -b <your feature branch name>
    
    `git branch` 명령으로 적절한 브랜치에 있는지 확인할 수 있습니다.
* 각 파일을 커밋의 일부로 추가하는 방식으로 바뀐 내용을 추가하십시오.  
        sh
        git add <file name> 파일을 추가하는 적절한 GUI 프로그램을 원한다면 
    
    [Gitk](https://git-scm.com/book/en/v2/Git-in-Other-Environments-Graphical-Interfaces) 또는 [`git add -p`](http://nuclearsquid.com/writings/git-add/) 를 참고하십시오.
* 추가한 파일을 바뀐 내용을 설명한 명료한 메시지를 넣어 제출하십시오  
        sh
        git commit -m "<your commit message>" 바람직한 제출 메시지 내용은 
    
    [기여](../contribute/README.md)절을 참조하십시오.
* 때로는 [업스트림 마스터](https://github.com/PX4/Firmware.git) 변경으로 인해 건너뛰는 경우가 있습니다. PX4 prefers a linear commit history and uses [git rebase](https://git-scm.com/book/de/v1/Git-Branching-Rebasing). To include the newest changes from upstream in your local branch, switch to your master branch  
        sh
        git checkout master 그리고 새 커밋을 업스트림 마스터에서 가져오십시오
    
      
        sh
        git pull upstream master 이제 로컬 마스터 브랜치는 최신입니다. 여러분이 기능을 추가하는 브랜치로 돌아가십시오
    
      
        sh
        git checkout <your feature branch name> and rebase on your updated master
    
      
        sh
        git rebase master

* 이제 로컬 커밋을 복제(fork)한 저장소로 밀어 올릴 수 있습니다  
        sh
        git push origin <your feature branch name>

* 복제(fork)한 저장소로 이동하여 밀어올리기(push)를 제대로 수행했는지 확인할 수 있습니다: `https://github.com/<your git name>/Firmware.git`  
    새 브랜치를 복제 저장소로 밀어올렸음을 알리는 메시지를 볼 수 있어야합니다.
* 이제 pull 요청(PR)을 할 시간입니다. "new branch message" 우측을 보면(한단계 전), "Compare & Create Pull Request"가 적힌 녹색 단추를 볼 수 있습니다. Then it should list your changes and you can (must) add a meaningful title (in case of a one commit PR, it's usually the commit message) and message (<span style="color:orange">explain what you did for what reason</span>. Check [other pull requests](https://github.com/PX4/Firmware/pulls) for comparison)
* You're done! Responsible members of PX4 will now have a look at your contribution and decide if they want to integrate it. Check if they have questions on your changes every once in a while.

## Get a Specific Release

To get the source code for a *specific older release*:

* Clone the Firmware repo and navigate into Firmware directory: 
        sh
        git clone https://github.com/PX4/Firmware.git
        cd Firmware

* List all releases (tags) 
        sh
        git tag -l

* Checkout code for particular tag (e.g. for tag 1.7.4beta) 
        sh
        git checkout v1.7.4beta

## Update Submodule

There are several ways to update a submodule. Either you clone the repository or you go in the submodule directory and follow the same procedure as in [Contributing code to PX4](#contributing_code).

## Do a PR for a submodule update

This is required after you have done a PR for a submodule X repository and the bug-fix / feature-add is in the current master of submodule X. Since the Firmware still points to a commit before your update, a submodule pull request is required such that the submodule used by the Firmware points to the newest commit.

```sh
cd Firmware
```

* Make a new branch that describes the fix / feature for the submodule update: 
        sh
        git checkout -b pr-some-fix

* Go to submodule subdirectory 
        sh
        cd <path to submodule>

* PX4 submodule might not necessarily point to the newest commit. Therefore, first checkout master and pull the newest upstream code. 
        sh
        git checkout master
        git pull upstream master

* Go back to Firmware directory, and as usual add, commit and push the changes. 
        sh
        cd -
        git add <path to submodule>
        git commit -m "Update submodule to include ..."
        git push upstream pr-some-fix

## Checkout pull requests

You can test someone's pull request (changes are not yet merged) even if the branch to merge only exists on the fork from that person. Do the following:

```sh
git fetch upstream  pull/<PR ID>/head:<branch name>
```

`PR ID` is the number right next to the PR's title (without the #) and the `<branch name>` can also be found right below the `PR ID`, e.g. `<the other persons git name>:<branch name>`. After that you can see the newly created branch locally with

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

If a conflict occurs during a `git rebase`, please refer to [this guide](https://help.github.com/articles/resolving-merge-conflicts-after-a-git-rebase/).

### Pull merge conflicts

If a conflict occurs during a `git pull`, please refer to [this guide](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/#competing-line-change-merge-conflicts).

### Build error due to git tags out of date

The build error `Error: PX4 version too low, expected at least vx.x.x` occurs if git tags are out of date.

This can be solved by fetching the upstream repository tags:

```sh
git fetch upstream --tags
```