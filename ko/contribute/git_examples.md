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
* 때로는 [업스트림 마스터](https://github.com/PX4/Firmware.git) 변경으로 인해 건너뛰는 경우가 있습니다. PX4는 선형 제출 기록 유지를 선호하며 [git rebase](https://git-scm.com/book/de/v1/Git-Branching-Rebasing) 명령을 활용합니다. 업스트림의 새로 바뀐 내용을 로컬 브랜치에 반영하려면 마스터 브랜치로 전환하십시오.  
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

* 복제(fork)한 저장소로 이동하여 밀어올리기(push)를 제대로 수행했는지 확인할 수 있습니다: `https://github.com/<your git name>/Firmware.git`  
    새 브랜치를 복제 저장소로 밀어올렸음을 알리는 메시지를 볼 수 있어야합니다.
* 이제 pull 요청(PR)을 할 시간입니다. "new branch message" 우측을 보면(한단계 전), "Compare & Create Pull Request"가 적힌 녹색 단추를 볼 수 있습니다. 그 다음 바뀐 내용을 나열하고 의미있는 제목(PR 제출 건이 하나인 경우, 보통 제출 메시지)과 메시지(<span style="color:orange">어떤 이유로 뭘 했는가를 설명</span>)를 추가할 수 있습니다(추가해야 합니다). [다른 pull 요청](https://github.com/PX4/Firmware/pulls)을 보고 비교해보십시오)
* 이제 다 끝났습니다. PX4 담당자가 기여 내용을 살펴보고 병합을 할 지 말지를 결정합니다. 그동안 바뀐 내용에 대해 질문이 있을지 한번 정도는 확인해보십시오.

## 특정 릴리스 가져오기

*오래된 특정 릴리스*의 소스코드를 가져오려면:

* 펌웨어 저장소를 다운로드한 후 펌웨어 디렉터리를 찾아보십시오: 
        sh
        git clone https://github.com/PX4/Firmware.git
        cd Firmware

* 모든 릴리스(태그)를 조회하십시오 
        sh
        git tag -l

* 해당 태그의 코드를 체크아웃하십시오(예: 태그 1.7.4beta) 
        sh
        git checkout v1.7.4beta

## 하위 모듈 업데이트 

하위모듈을 업데이트하는 방법에는 여러가지가 있습니다. 저장소를 복제하거나 하위모듈 디렉터리로 이동하여 [PX4에 코드 기여하기](#contributing_code)의 동일한 절차를 따르는 것입니다.

## 하위 모듈 업데이트 PR 진행

하위모듈 X 저장소와 버그 수정/기능 추가는 하위 모듈 X 의 현재 마스터 브랜치의 PR을 완료하고 나면 이 과정이 필요합니다. 펌웨어에서는 업데이트 이전 제출 사항을 가리키고 있으니 하위 모듈 pull 요청시 펌웨어에서 활용하는 하위모듈이 새 제출 내용을 가리키도록 해야 합니다.

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

브랜치를 만든 사람의 기존 사본에만 병합 대상 브랜치가 있을 경우, 누군가의 pull 요청을 시험(바뀐 내용을 마스터 브랜치에 아직 병합하지 않음)할 수 있습니다. 다음 명령을 실행하십시오:

```sh
git fetch upstream  pull/<PR ID>/head:<branch name>
```

`PR ID`는 PR 제목 다음 옆에 있는 (# 을 뺀) 숫자이며 `<branch name>`은 아래의 `PR ID` 바로 옆에서 찾을 수 있습니다. 예를 들면 `<the other persons git name>:<branch name>` 같은 식입니다. 이 과정을 진행하고 나면 다음 명령으로 로컬에서 새로 만든 브랜치를 볼 수 있습니다

```sh
git branch
```

그러면 브랜치를 전환하겠습니다.

```sh
git checkout <branch name>
```

## 일반적인 실수

### 복제한 저장소로 강제로 밀어올리기(push)

처음 PR을 끝내고나면 PX4 커뮤니티에서 바뀐 내용을 살펴봅니다. 대부분의 경우는 검토 후 로컬 브랜치에서 무언가를 수정해야 합니다. 파일을 바꾼 다음에는 기능 브랜치를 가장 최근의 업스트림/마스터로 다시 재편성해야합니다. 그러나 재편성(rebase)후, 복제(fork)한 저장소에의 기능 단위 브랜치에 더이상 직접 밀어올릴 수 없는 상황이 옵니다만, 강제로 진행해야 합니다.

```sh
git push --force-with-lease origin <your feature branch name>
```

### 동시 병합 문제 재편성 해결

`git rebase` 명령 진행 중 문제가 발생했을 때는 [이 안내서](https://help.github.com/articles/resolving-merge-conflicts-after-a-git-rebase/)를 참고하십시오.

### pull 동시 병합 문제

`git pull` 명령 실행 중 동시 병합 문제가 발생할 경우 [이 안내서](https://help.github.com/articles/resolving-a-merge-conflict-using-the-command-line/#competing-line-change-merge-conflicts)를 참고하십시오.

### 오래된 git 태그로 인한 빌드 오류

`Error: PX4 version too low, expected at least vx.x.x` 빌드 오류는 git 태그가 너무 오래됐을 경우 나타납니다.

이 문제는 업스트림 저장소 태그를 가져오면 해결할 수 있습니다:

```sh
git fetch upstream --tags
```