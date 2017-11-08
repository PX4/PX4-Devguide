# GIT 예제

## PX4 code에 기여하기 {#contributing-code-to-px4}

PX4에 새로운 기능을 추가하기 위해서는 정해놓은 워크플로우를 따라야 합니다. PX4에 여러분이 기여한 코드를 공유하기 위해서 아래와 같은 예제를 참고하세요.

* [Sign up](https://github.com/join) github 계정이 없는 경우 계정을 생성합니다.
* Firmware를 Fork합니다. (참고 [여기](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository))
* 여러분 컴퓨터에 fork한 repository를 clone 합니다.<br>
```sh
cd ~/wherever/
git clone https://github.com/<your git name>/Firmware.git
```
* 새로 생성한 디렉토리로 가서 submodue의 초기화 업데이트를 수행하고 original upstream Firmware를 추가합니다.<br>
```sh
cd Firmware
git submodule update --init --recursive
git remote add upstream https://github.com/PX4/Firmware.git
```
* 이제 2개 remote repository를 가지게 됩니다. : 하나는 PX4 Firmware를 가리키며 이를 upstream이라 부릅니다. 다른 하나는 fork한 repository를 가리킵니다.
* 다음과 같은 명령으로 체크할 수 있습니다 :
```sh
git remote -v
```
* 현재 master에 추가하기 위해 변경합니다.
* 여러분의 기능을 대표할 수 있는 의미있는 이름으로 새로운 branch를 생성합니다.<br>
```sh
git checkout -b <your feature branch name>
```
이제 ```git branch``` 명령을 사용해서 현재 올바른 branch에 위치하고 있는지 확인합니다.
* commit할 각 변경된 파일을 추가하기 위해서 변경사항을 추가합니다.<br>
```sh
git add <file name>
```
GUI로 파일을 추가하기 위한다면 다음을 참고하세요. [Gitk](https://git-scm.com/book/en/v2/Git-in-Other-Environments-Graphical-Interfaces) 나 [```git add -p```](http://nuclearsquid.com/writings/git-add/)
* 변경내용을 설정하는 의미있는 메시지를 추가하여 추가한 파일들을 commit 합니다.<br>
```sh
git commit -m "<your commit message>"
```
좋은 commit 메시지에 대한 내용은 [Contributing](../contribute/README.md) 를 참고하세요.
* 시간이 지나면 [upstream master](https://github.com/PX4/Firmware.git)가 변경됩니다. PX4는 점진적인 commit history를 선호하며 [git rebase](https://git-scm.com/book/de/v1/Git-Branching-Rebasing)를 사용합니다. upstream에서 변경사항을 나의 local branch에 포함시키기 위해서 master branch로 전환시킵니다.<br>
```sh
git checkout master
```
다음으로 upstream에서 새로 commit된 것들을 pull합니다.<br>
```sh
git pull upstream master
```
이제 local master는 최신상태가 됩니다. 이제 다시 여러분의 feature branch로 전환합니다.<br>
```sh
git checkout <your feature branch name>
```
다음으로 업데이트한 master를 rebase합니다.<br>
```sh
git rebase master
```
* 이제 local commit을 fork한 repository에 push합니다.<br>
```sh
git push origin <your feature branch name>
```
* push가 성공적으로 수행되었는지 확인하기 위해 fork한 repository를 브라우저로 확인합니다. ```https://github.com/<your git name>/Firmware.git```<br>
여기서 새로 생성한 branch가 fork한 repository에 push되었다는 메시지를 확인합니다.
* 이제 PR(pull request)를 생성할 차례입니다. 이전 단계에서 봤던 "new branch message"의 오른쪽에, "Compare & Create Pull Request" 녹색 버튼이 있을 것입니다. 변경 내용을 보여주고 의미있는 title과 메시지를 추가합니다. title은 commit PR이 하나인 경우 보통 commit 메시지가 됩니다. 메시지는 <span style="color:orange">무슨 이유로 이렇게 했는지 설명</span> 합니다. [other pull requests](https://github.com/PX4/Firmware/pulls)를 보고 비교해 보세요.
* 자 이제 끝났습니다! PX4의 멤버가 여러분의 작성한 코드르르 살펴보고 통합시킬지 여부를 결정하게 됩니다. 만약에 여러분이 변경한 부분에 대해서 질문이 있을 수 있으니 확인해 주세요.

## submodule 업데이트

submodule 업데이트하는 여러가지 방식이 있습니다. repository를 clone하거나 submodule 디렉토리로 가서 [Contributing code to PX4](#contributing-code-to-px4)와 동일한 절차를 따릅니다.

## submodule 업데이트를 위한 PR

이 과정은 submodle X repository에 대해서 PR를 한 후에 필요하며 buf-fix / feature-add는 submodule X의 현재 master에 있습니다. Firmware는 여러분이 업데이트하기 전의 commit를 가리키므로, Firmware가 사용한 submodule이 가리키는 새로운 commit처럼 submodule pull request가 필요합니다.
```sh
cd Firmware
```
* submodule 업데이트를 위해 fix / feature를 설명하는 새로운 branch를 만듭니다. :
```sh
git checkout -b pr-some-fix
```
* submodule 하부 디렉토리로 이동합니다.
```sh
cd <path to submodule>
```
* PX4 submodule는 가장 최신 commit를 가리키고 있지 않을 수도 있습니다. 따라서 먼저 checkout master를 하고 최신 upstream 코드를 pull합니다.
```sh
git checkout master
git pull origin master
```
* Firmware 디렉토리로 돌아가서 보통처럼 변경에 대해서 add, commit, push를 수행합니다.
```sh
cd -
git add <path to submodule>
git commit -m "Update submodule to include ..."
git push upstream pr-some-fix
```

## Checkout pull requests

아직 merge하지 않은 변경에 대해서도 다른 사람의 pull request를 테스트할 수 있습니다. 특정 사람의 fork에만 존재하더라도 가능합니다. 다음과 같이 따라합니다.
```sh
git fetch upstream  pull/<PR ID>/head:<branch name>
```
```PR ID```은 PR의 title 바로 옆에 있는 숫자이고 ```<branch name>```은 ```PR ID```의 오른쪽 아래에서 볼수 있습니다. 예로 ```<the other persons git name>:<branch name>```. 이제 다음과 같은 명령을 실행해서 새로운 생성한 branch를 볼 수 있습니다.
```sh
git branch
```
다음으로 해당 branch로 전환합니다.
```sh
git checkout <branch name>
```

## 흔히 하는 실수

### fork한 repository에 force push 하기

처음 PR을 하고나서, PX4 커뮤니티에서 여러분이 변경한 내용을 리뷰합니다. 대부분 경우 리뷰를 참고하여 여러분의 local branch를 수정해야한다는 것을 의미합니다. 파일을 변경한 후에, 가장 최신의 upstream/master으로 feature branch를 다시 rebase해야 합니다. 하지만 rebase 후에, 직접 여러분이 fork한 repository로 feature branch를 push할 수 없으므로 대신 force push를 사용해야 합니다. :
```sh
git push --force-with-lease origin <your feature branch name>
```

### Rebase merge 충돌

```git rebase``` 할때 conflict가 발생하면 다음을 참고하세요. [this guide](https://help.github.com/articles/resolving-merge-conflicts-after-a-git-rebase/).

### Pull merge conflicts

```git pull``` 할때 conflict가 발생하면 다음을 참고하세요. [this guide](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/#competing-line-change-merge-conflicts).
