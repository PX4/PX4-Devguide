# 源代码管理

## 分支模型

PX4 项目使用三分支 Git 模型：

- [master](https://github.com/px4/firmware/tree/master) 默认是不稳定版本， 用于快速开发。
- [beta](https://github.com/px4/firmware/tree/beta) 经过全面测试。 它是供飞行测试人员使用的。
- [stable](https://github.com/px4/firmware/tree/stable) 是最新发行版本。

我们试着 [通过重置保留线性历史](https://www.atlassian.com/git/tutorials/rewriting-history)，并且避免 [Github flow](https://guides.github.com/introduction/flow/)。 然而，由于全球团队和快速的发展，我们可能有时会进行合并。

要贡献新功能，[注册Github](https://help.github.com/articles/signing-up-for-a-new-github-account/)，然后 [fork](https://help.github.com/articles/fork-a-repo/) 仓库，[创建一个新分支](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/)，添加您的更改，最后 [发送拉取请求](https://help.github.com/articles/using-pull-requests/)。 更改将在通过我们的 [持续整合](https://en.wikipedia.org/wiki/Continuous_integration) 测试时合并。

所有代码贡献都必须在许可的 [BSD 3 条款的许可证 ](https://opensource.org/licenses/BSD-3-Clause) 下进行，不得对其使用施加任何进一步的限制。

## 代码样式格式

PX4 使用 [asty](http://astyle.sourceforge.net/) 进行代码格式化。 有效版本是

- [astyle 2.06](https://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/)（推荐）
- [astyle 3.0](https://sourceforge.net/projects/astyle/files/astyle/astyle%203.0/)
- [astyle 3.01](https://sourceforge.net/projects/astyle/files/)

一旦安装完毕，格式化可以使用 `.工具/astyle/check_code_style_all.sh`。 一个干净的 master 分支的输出应该是 `格式检查通过`。 如果做到这一点，`制作格式`将来可以自动检查和格式化所有文件。

## 源码文档

鼓励PX4开发者创建适当的源文档。

> **Note** 源代码文件标准没有得到执行，目前该代码的文件记录不一致。 我们想做得更好！

Currently we have two types of source-based documentation:

- `PRINT_MODULE_*` methods are used for both module run time usage instructions and for the [Modules & Commands Reference](../middleware/modules_main.md) in this guide. 
  - The API is documented [in the source code here](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381). 
  - Good examples of usage include the [Application/Module Template](../apps/module_template.md) and the files linked from the modules reference.

- We encourage other in-source documentation *where it adds value/is not redundant*.
  
  > **Tip** Developers should name C++ entities (classes, functions, variables etc.) such that their purpose can be inferred - reducing the need for explicit documentation.
  
  - Do not add documentation that can trivially be assumed from C++ entity names.
  - Commonly you may want to add information about corner cases and error handling.
  - [Doxgyen](http://www.doxygen.nl/) tags should be used if documentation is needed: `@class`, `@file`, `@param`, `@return`, `@brief`, `@var`, `@see`, `@note`. A good example of usage is [src/modules/events/send_event.h](https://github.com/PX4/Firmware/blob/master/src/modules/events/send_event.h).

## Commits and Commit Messages

Please use descriptive, multi-paragraph commit messages for all non-trivial changes. Structure them well so they make sense in the one-line summary but also provide full detail.

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
    

**Use **`git commit -s`** to sign off on all of your commits.** This will add `signed-off-by:` with your name and email as the last line.

This commit guide is based on best practices for the Linux Kernel and other [projects maintained](https://github.com/torvalds/subsurface/blob/a48494d2fbed58c751e9b7e8fbff88582f9b2d02/README#L88-L115) by Linus Torvalds.