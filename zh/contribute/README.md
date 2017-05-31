---
translated_page: https://github.com/PX4/Devguide/blob/master/en/contribute/README.md
translated_sha: 8d53a0221388930d07f44e470505abcd94437835
---

# 贡献代码

官网英文原文地址：http://dev.px4.io/starting-contributing.html

核心开发团队和社区的联系信息可以在下面找到。PX4项目使用了三个分支Git branching model:

* [master](https://github.com/px4/firmware/tree/master) 默认情况下不稳定，可以看到快速的开发。  
* [beta](https://github.com/px4/firmware/tree/beta) 已充分测试，面向飞行测试者。  
* [stable](https://github.com/px4/firmware/tree/stable) 指向最新的发布分支。  

我们尝试通过[rebases保持一个线性的历史](https://www.atlassian.com/git/tutorials/rewriting-history)，避免[Github flow](https://guides.github.com/introduction/flow/)。但是由于全球的开发队伍和快速的开发转移，我们会定期分类合并。

为了贡献新的功能，首先[注册Github账户](https://help.github.com/articles/signing-up-for-a-new-github-account/)，然后[fork](https://help.github.com/articles/fork-a-repo/)仓库，[创建新分支](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/)，加入你的改变，最后发送[pull request](https://help.github.com/articles/using-pull-requests/)。当它们通过我们的持续的[综合测试](https://en.wikipedia.org/wiki/Continuous_integration)，更新就会被合并。

所有的贡献必须在 [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause)许可下进行,并且所有的代码在使用上不能提出任何的，进一步的限制。


## 提交与提交消息

请使用描述性的、多段落提交消息进行所有非平凡的更改。 也可以好好组织语言，使其能够在一行的总结中有意义，并且依然提供全面的细节。

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

**使用 ```git commit -s```来结束你的所有提交。** 这将会添加签名 ```signed-off-by:``` 以您的姓名和电子邮件作为最后一行。

此提交指南是基于Linux内核和Linus Torvalds[维护的其他项目](https://github.com/torvalds/subsurface/blob/a48494d2fbed58c751e9b7e8fbff88582f9b2d02/README#L88-L115)的最佳做法。

## 测试飞行结果

飞行测试对于保证质量非常重要，请从microSD卡上传飞行日志到 [Log Muncher](http://logs.uaventure.com)，并在[PX4 Discuss](http://discuss.px4.io/)分享链接，附带书面飞行报告。


## 论坛和聊天

* [Google+](https://plus.google.com/117509651030855307398)
* [Gitter](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
* [PX4 Users Forum](http://groups.google.com/group/px4users)

## 每周开发电话

PX4开发团队每周都会进行电话会议，同步进度。

* 时间： 周三 5PM CET, 11AM EST, 8AM PDT \([subscribe to calendar](https://calendar.google.com/calendar/ical/px4.io_fs35jm7ugmvahv5juhhr3tkkf0%40group.calendar.google.com/public/basic.ics)\)
* Uberconference: [www.uberconference.com/lf-dronecode](http://www.uberconference.com/lf-dronecode)
* 电话： +1 415-891-1494
* 议程事先在[PX4 Discuss](http://discuss.px4.io/c/weekly-dev-call)公布
* 问题和PRs可能被标记为[devcall](https://github.com/PX4/Firmware/labels/devcall)来进行讨论

