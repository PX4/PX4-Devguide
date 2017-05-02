---
translated_page: https://github.com/PX4/Devguide/blob/master/en/contribute/README.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 贡献代码

官网英文原文地址：http://dev.px4.io/starting-contributing.html

核心开发团队和社区的联系信息可以在下面找到。PX4项目使用了三个分支Git branching model:

* [master](https://github.com/px4/firmware/tree/master) 默认情况下不稳定，可以看到快速的开发。  
* [beta](https://github.com/px4/firmware/tree/beta) 已充分测试，面向飞行测试者。  
* [stable](https://github.com/px4/firmware/tree/stable) 指向最新的发布分支。  

我们尝试通过[rebases保持一个线性的历史](https://www.atlassian.com/git/tutorials/rewriting-history)，避免[Github flow](https://guides.github.com/introduction/flow/)。但是由于全球的开发队伍和快速的开发转移，我们会定期分类合并。

为了贡献新的功能，首先[注册Github账户](https://help.github.com/articles/signing-up-for-a-new-github-account/)，然后[fork](https://help.github.com/articles/fork-a-repo/)仓库，[创建新分支](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/)，加入你的改变，最后发送\[pull request\]。当它们通过我们的持续的[综合测试](https://en.wikipedia.org/wiki/Continuous_integration)，更新就会被合并。

所有的贡献必须在 [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause)许可下进行,并且所有的代码在使用上不能提出任何的，进一步的限制。


## Commits and Commit Messages

Please use descriptive, multi-paragraph commit messages for all non-trivial changes. Structure them well so they make sense in the one-line summary but also provide full detail.

```
Component: Explain the change in one sentence. Fixes #1234

Prepend the software component to the start of the summary
line, either by the module name or a description of it.
(e.g. "mc_att_ctrl" or "multicopter attitude controller").

If the issue number is appended as <Fixes #1234>, Github
will automatically close the issue when the commit is
merged to the master branch.

The body of the message can contain several paragraphs.
Describe in detail what you changed. Link issues and flight
logs either related to this fix or to the testing results
of this commit.

Describe the change and why you changed it, avoid to
paraphrase the code change (Good: "Adds an additional
safety check for vehicles with low quality GPS reception".
Bad: "Add gps_reception_check() function").

Reported-by: Name <email@px4.io>
```

**Use ```git commit -s``` to sign off on all of your commits.** This will add ```signed-off-by:``` with your name and email as the last line.

This commit guide is based on best practices for the Linux Kernel and other [projects maintained](https://github.com/torvalds/subsurface/blob/a48494d2fbed58c751e9b7e8fbff88582f9b2d02/README#L88-L115) by Linus Torvalds.

## 测试飞行结果

飞行测试对于保证质量非常重要，请从microSD卡上传飞行日志到 [Log Muncher](http://logs.uaventure.com)，并在[论坛](http://groups.google.com/group/px4users)分享连接，附带有书面飞行报告。


## 论坛和聊天

* [Google+](https://plus.google.com/117509651030855307398)
* [Gitter](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
* [PX4 Users Forum](http://groups.google.com/group/px4users)

## 每周开发电话

PX4团队在每周同时进行电话联系（通过[Mumble](http://mumble.info) client\)客户端连接）。

* TIME: 19:00h Zurich time, 1 p.m. Eastern Time, 10 a.m. Pacific Standard Time
* Server: mumble.dronecode.org
  * Port: 10028
  * Channel: PX4 Channel
  * The agenda is announced the same day on the [px4users forum](http://groups.google.com/group/px4users)