# Contributing

Contact information for the core dev team and community can be [found here](../README.md#support). 
Developers are also most welcome to attend the [weekly dev call](#dev_call) and other [developer events](#calendar).

## Support

### Weekly Dev Call {#dev_call}

The PX4 dev team syncs up on platform technical details and in-depth analysis. There is also space in the agenda to discuss pull requests, major impacting issues and Q&A.

#### Who should attend:

* Core project maintainers
* Component maintainers
* Test team lead
* Dronecode members
* Community members

> **Tip** The dev call is open to all interested developers (not just the core dev team). 
> This is a great opportunity to meet the team and contribute to the ongoing development of the platform.

#### Schedule

* TIME: Wednesday 5PM CET, 11AM EST, 8AM PST \([subscribe to calendar](https://calendar.google.com/calendar/ical/px4.io_fs35jm7ugmvahv5juhhr3tkkf0%40group.calendar.google.com/public/basic.ics)\)
* **Join the call**: https://zoom.us/j/625711763
* **Meeting ID**: 625 711 763
* **Dial(for higher quality, dial a number based on your current location)**:
  * **Switzerland**: +41 (0) 31 528 0988 
  * **US**: +1 646 876 9923  or +1 669 900 6833  or +1 408 740 3766 
  * **Germany**: +49 (0) 30 3080 6188 
  * **Mexico**: +52 554 161 4288 
  * **Australia**: +61 (0) 2 8015 2088 
  * **United Kingdom**: +44 (0) 20 3695 0088 
  * **South Korea**: +82 (0) 2 6022 2322 
  * **Spain**: +34 91 198 0188 
  * [**International numbers available**](https://zoom.us/zoomconference?m=bMNcJolpnXMOL_qSf2svkR7Yow6FqceS)

* Agenda is published before the call on [PX4 Discuss - weekly-dev-call](http://discuss.px4.io/c/weekly-dev-call)
* To nominate Issues and PRs for the call you can use the [devcall](https://github.com/PX4/Firmware/labels/devcall) label to flag them for discussion.

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

### Coding Standards {#code_standards}

PX4 recommends the following coding standards:
- In-source documentation [as below](#source_docs)
- Formatting/layout using *astyle*, [as below](#code_formatting)

Other conventions:
- All files should have standard copyright header, with date range reflecting creation to last modification date
  (note that the comment markup may change for different types of files).
  ```
  /****************************************************************************
  *
  *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions
  * are met:
  *
  * 1. Redistributions of source code must retain the above copyright
  *    notice, this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright
  *    notice, this list of conditions and the following disclaimer in
  *    the documentation and/or other materials provided with the
  *    distribution.
  * 3. Neither the name PX4 nor the names of its contributors may be
  *    used to endorse or promote products derived from this software
  *    without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *
  ****************************************************************************/
  ```
- Naming conventions:
  - Classes
    - Use CamelCase
    - Start with a capitalized letter and are nouns.
    - One class per file, with filename prefix matching classname.
  - Methods/functions and variables
    - Use snake case (words start with lower case letters and are separated by underscores).
    - Files containing just functions may be named following pattern of main function.
  - Member variables
    - Start with an underscore and otherwise use snake case.
- Prefer `#pragma once` over include guards


<!-- 

Include Headers
A good practice is to include headers in the order most local to least local and alphabetized to whatever degree possible to avoid duplications. The purpose for this is to ensure that a proper dependency chain is maintained, because as projects grow, dependency driven compilation failures sometimes can be difficult to identify and resolve.

This means that header files have includes (alphabetized) in the order:
project includes
project dependency includes
system includes

Source files have the includes (alphabetized) in the order:
definition include
project includes
project dependency includes
system includes
-->


### Code Formatting {#code_formatting}

PX4 uses [astyle](http://astyle.sourceforge.net/) for code formatting. Valid versions are
* [astyle 2.06](https://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/) (recommended)
* [astyle 3.0](https://sourceforge.net/projects/astyle/files/astyle/astyle%203.0/)
* [astyle 3.01](https://sourceforge.net/projects/astyle/files/)

Once installed, formatting can be checked with `./Tools/astyle/check_code_style_all.sh`. The output should be `Format checks passed` on a clean master. If that worked, `make format` can be used in the future to check and format all files automatically.



### In-Source Documentation {#source_docs}

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
