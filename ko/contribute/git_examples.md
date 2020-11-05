# git 예제

<a id="contributing_code"></a>

## Contributing Code to PX4

Adding a feature to PX4 follows a defined workflow. In order to share your contributions on PX4, you can follow this example.

* 아직 github 계정이 없다면 [가입](https://github.com/join) 하십시오
* Fork the PX4-Autopilot re[p (see [here](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository))
* 여러분의 계정으로 복제(fork)한 저장소를 로컬 컴퓨터로 가져오십시오  
        sh
        cd ~/wherever/
        git clone https://github.com/<your git name>/PX4-Autopilot.git

* Go into the new directory, initialize and update the submodules, and add the original upstream PX4-Autopilot  
        sh
        cd PX4-Autopilot
        git submodule update --init --recursive
        git remote add upstream https://github.com/PX4/PX4-Autopilot.git

* You should have now two remote repositories: One repository is called upstream that points to PX4/PX4-Autopilot, and one repository that points to your forked repository of the PX4 repository.
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
* Some time might have passed and the [upstream master](https://github.com/PX4/PX4-Autopilot.git) has changed. PX4는 선형 제출 기록 유지를 선호하며 [git rebase](https://git-scm.com/book/de/v1/Git-Branching-Rebasing) 명령을 활용합니다. 업스트림의 새로 바뀐 내용을 로컬 브랜치에 반영하려면 마스터 브랜치로 전환하십시오.  
        sh
        git checkout master 그리고 새 커밋을 업스트림 마스터에서 가져오십시오
    
      
        sh
        git pull upstream master 이제 로컬 마스터 브랜치는 최신입니다. 여러분이 기능을 추가하는 브랜치로 돌아가십시오
    
      
        sh
        git checkout <your feature branch name> 그리고 master 브랜치를 재편성(rebase)하십시오
    
      
        sh
        git rebase master

* 이제 로컬 커밋을 복제(fork)한 저장소로 밀어 올릴 수 있습니다  
        sh
        git push origin <your feature branch name>

* You can verify that the push was successful by going to your forked repository in your browser: `https://github.com/<your git name>/PX4-Autopilot.git`  
    There you should see the message that a new branch has been pushed to your forked repository.
* 이제 pull 요청(PR)을 할 시간입니다. "new branch message" 우측을 보면(한단계 전), "Compare & Create Pull Request"가 적힌 녹색 단추를 볼 수 있습니다. 그 다음 바뀐 내용을 나열하고 의미있는 제목(PR 제출 건이 하나인 경우, 보통 제출 메시지)과 메시지(<span style="color:orange">어떤 이유로 뭘 했는가를 설명</span>)를 추가할 수 있습니다(추가해야 합니다). Check [other pull requests](https://github.com/PX4/PX4-Autopilot/pulls) for comparison)
* 이제 다 끝났습니다. PX4 담당자가 기여 내용을 살펴보고 병합을 할 지 말지를 결정합니다. 그동안 바뀐 내용에 대해 질문이 있을지 한번 정도는 확인해보십시오.

## 특정 릴리스 가져오기

To get the source code for a *specific older release*:

* Clone the PX4-Autopilot repo and navigate into PX4-Autopilot directory: 
        sh
        git clone https://github.com/PX4/PX4-Autopilot.git
        cd PX4-Autopilot

* 모든 릴리스(태그)를 조회하십시오 
        sh
        git tag -l

* 해당 태그의 코드를 체크아웃하십시오(예: 태그 1.7.4beta) 
        sh
        git checkout v1.7.4beta

## 하위 모듈 업데이트 

There are several ways to update a submodule. Either you clone the repository or you go in the submodule directory and follow the same procedure as in [Contributing code to PX4](#contributing_code).

## 하위 모듈 업데이트 PR 진행

This is required after you have done a PR for a submodule X repository and the bug-fix / feature-add is in the current master of submodule X. Since the Firmware still points to a commit before your update, a submodule pull request is required such that the submodule used by the Firmware points to the newest commit.

```sh
cd Firmware
```

* 수정 내용 / 하위모듈 업데이트 기능을 설명하는 새 브랜치를 만드십시오: 
        sh
        git checkout -b pr-some-fix

* 하위 모듈 하위 디렉터리로 이동하십시오 
        sh
        cd <path to submodule>

* PX4 하위 모듈은 아마도 새 제출 내용을 가리킬 필요는 없을수도 있습니다. 그러니, 마스터 브랜치를 우선 체크아웃한 후 새 업스트림 코드를 가져오십시오. 
        sh
        git checkout master
        git pull upstream master

* 펌웨어 디렉터리로 돌아가서 마찬가지로 바뀐 내용의 추가, 제출, 밀어올리기를 진행하십시오. 
        sh
        cd -
        git add <path to submodule>
        git commit -m "Update submodule to include ..."
        git push upstream pr-some-fix

## pull 요청 진입

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

## 일반적인 실수

### 복제한 저장소로 강제로 밀어올리기(push)

After having done the first PR, people from the PX4 community will review your changes. In most cases this means that you have to fix your local branch according to the review. After changing the files locally, the feature branch needs to be rebased again with the most recent upstream/master. However, after the rebase, it is no longer possible to push the feature branch to your forked repository directly, but instead you need to use a force push:

```sh
git push --force-with-lease origin <your feature branch name>
```

### 동시 병합 문제 재편성 해결

If a conflict occurs during a `git rebase`, please refer to [this guide](https://help.github.com/articles/resolving-merge-conflicts-after-a-git-rebase/).

### pull 동시 병합 문제

If a conflict occurs during a `git pull`, please refer to [this guide](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/#competing-line-change-merge-conflicts).

### 오래된 git 태그로 인한 빌드 오류

The build error `Error: PX4 version too low, expected at least vx.x.x` occurs if git tags are out of date.

This can be solved by fetching the upstream repository tags:

```sh
git fetch upstream --tags
```