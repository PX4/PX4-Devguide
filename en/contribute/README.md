# Contributing

Contact information for the core dev team and community can be [found here](../README.md#support). 
Developers are also most welcome to attend the [weekly dev call](#dev_call) and other [developer events](#calendar).

## Support

### Weekly Dev Call {#dev_call}

The PX4 dev team syncs up on platform technical details and in-depth analysis. 
There is also space in the agenda to discuss pull requests, major impacting issues and Q&A.

> **Tip** Dev Call schedule, agenda, and call in details, [can be found here](https://dev.px4.io/master/en/contribute/dev_call.html)!


### Tests Flights

The Dronecode test team can help review (test flight) your pull requests and provide feedback and logs.

See [Test Flights](../test_and_ci/test_flights.md) for information about available test vehicles/autopilots, how to request flights, and response times.


### Have a problem?

#### Help diagnosing problems

If you are unsure what the problem is and you need help diagnosing

* Upload logs to [Flight Log Review](http://logs.px4.io/)
* Open a discussion on [PX4 Discuss](http://discuss.px4.io/) with a flight report and links to logs.
* If you find an issue or bug with PX4 [open a Github Issue](https://github.com/PX4/Devguide/issues)

#### Issue & Bug reporting

* Upload logs to [Flight Log Review](http://logs.px4.io/)
* [Open a Github Issue](https://github.com/PX4/Devguide/issues) with a flight report with as much detail as possible and links to logs.

#### General support
* [Join our Slack community](http://slack.px4.io/)
* [Open a discussion](http://discuss.px4.io)
* [Open Github Issue](https://github.com/PX4/Devguide/issues)


### Calendar & Events {#calendar}

The *Dronecode Calendar* shows important events for platform developers and users. 
Select the links below to display the calendar in your timezone (and to add it to your own calendar):
* [Switzerland – Zurich](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Europe%2FZurich)
* [Pacific Time – Tijuana](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=America%2FTijuana)
* [Australia – Melbourne/Sydney/Hobart](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Australia%2FSydney)

**Note:** calendar defaults to CET.


{% raw %}
<iframe src="https://calendar.google.com/calendar/embed?title=Dronecode%20Calendar&amp;mode=WEEK&amp;height=600&amp;wkst=1&amp;bgcolor=%23FFFFFF&amp;src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&amp;color=%23691426&amp;ctz=Europe%2FZurich" style="border-width:0" width="800" height="600" frameborder="0" scrolling="no"></iframe>
{% endraw %}

## Contribution

### Code of Conduct

We pledge to adhere to the [PX4 code of conduct](https://github.com/PX4/Firmware/blob/master/CODE_OF_CONDUCT.md). This code aims to foster an open and welcoming environment.


### Source Code Management

The PX4 project uses a three-branch Git branching model:

* [master](https://github.com/px4/firmware/tree/master) is by default unstable and sees rapid development.
* [beta](https://github.com/px4/firmware/tree/beta) has been thoroughly tested. It's intended for flight testers.
* [stable](https://github.com/px4/firmware/tree/stable) points to the last release.

We try to retain a [linear history through rebases](https://www.atlassian.com/git/tutorials/rewriting-history) and avoid the [Github flow](https://guides.github.com/introduction/flow/). However, due to the global team and fast moving development we might resort to merges at times.

To contribute new functionality, [sign up for Github](https://help.github.com/articles/signing-up-for-a-new-github-account/), then [fork](https://help.github.com/articles/fork-a-repo/) the repository, [create a new branch](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/), add your changes, and finally [send a pull request](https://help.github.com/articles/using-pull-requests/). Changes will be merged when they pass our [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration) tests.

All code contributions have to be under the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause) and all code must not impose any further constraints on the use.

### Code Style Formatting

PX4 uses [astyle](http://astyle.sourceforge.net/) for code formatting. Valid versions are
* [astyle 2.06](https://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/) (recommended)
* [astyle 3.0](https://sourceforge.net/projects/astyle/files/astyle/astyle%203.0/)
* [astyle 3.01](https://sourceforge.net/projects/astyle/files/)

Once installed, formatting can be checked with `./Tools/astyle/check_code_style_all.sh`. The output should be `Format checks passed` on a clean master. If that worked, `make format` can be used in the future to check and format all files automatically.

### In-Source Documentation

PX4 developers are encouraged to create appropriate in-source documentation.

> **Note** Source-code documentation standards are not enforced, and the code is currently inconsistently documented.
  We'd like to do better!
  
Currently we have two types of source-based documentation:
- `PRINT_MODULE_*` methods are used for both module run time usage instructions and for the [Modules & Commands Reference](../middleware/modules_main.md) in this guide.
  - The API is documented [in the source code here](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381). 
  - Good examples of usage include the [Application/Module Template](../apps/module_template.md) and the files linked from the modules reference.
* We encourage other in-source documentation *where it adds value/is not redundant*. 

  > **Tip** Developers should name C++ entities (classes, functions, variables etc.) such that their purpose can be inferred - reducing the need for explicit documentation.
  
  - Do not add documentation that can trivially be assumed from C++ entity names.
  - Commonly you may want to add information about corner cases and error handling.
  - [Doxgyen](http://www.doxygen.nl/) tags should be used if documentation is needed: `@class`, `@file`, `@param`, `@return`, `@brief`, `@var`, `@see`, `@note`. A good example of usage is [src/modules/events/send_event.h](https://github.com/PX4/Firmware/blob/master/src/modules/events/send_event.h).

### Commits and Commit Messages

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

**Use **`git commit -s`** to sign off on all of your commits.** This will add `signed-off-by:` with your name and email as the last line.

This commit guide is based on best practices for the Linux Kernel and other [projects maintained](https://github.com/torvalds/subsurface/blob/a48494d2fbed58c751e9b7e8fbff88582f9b2d02/README#L88-L115) by Linus Torvalds.
