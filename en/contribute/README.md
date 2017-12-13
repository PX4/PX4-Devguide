# Contributing

Contact information for the core dev team and community can be [found here](../README.md#support). 
Developers are also most welcome to attend the [weekly dev call](#dev_call) and other [developer events](#calendar).

## Source Code Management

The PX4 project uses a three-branch Git branching model:

* [master](https://github.com/px4/firmware/tree/master) is by default unstable and sees rapid development.
* [beta](https://github.com/px4/firmware/tree/beta) has been thoroughly tested. It's intended for flight testers.
* [stable](https://github.com/px4/firmware/tree/stable) points to the last release.

We try to retain a [linear history through rebases](https://www.atlassian.com/git/tutorials/rewriting-history) and avoid the [Github flow](https://guides.github.com/introduction/flow/). However, due to the global team and fast moving development we might resort to merges at times.

To contribute new functionality, [sign up for Github](https://help.github.com/articles/signing-up-for-a-new-github-account/), then [fork](https://help.github.com/articles/fork-a-repo/) the repository, [create a new branch](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/), add your changes, and finally [send a pull request](https://help.github.com/articles/using-pull-requests/). Changes will be merged when they pass our [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration) tests.

All code contributions have to be under the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause) and all code must not impose any further constraints on the use.

## Code Style Formatting

PX4 uses [astyle](http://astyle.sourceforge.net/) for code formatting. Valid versions are
* [astyle 2.06](https://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/) (recommended)
* [astyle 3.0](https://sourceforge.net/projects/astyle/files/astyle/astyle%203.0/)
* [astyle 3.01](https://sourceforge.net/projects/astyle/files/)

Once installed, formatting can be checked with `./Tools/astyle/check_code_style_all.sh`. The output should be `Format checks passed` on a clean master. If that worked, `make format` can be used in the future to check and format all files automatically.


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

**Use **`git commit -s`** to sign off on all of your commits.** This will add `signed-off-by:` with your name and email as the last line.

This commit guide is based on best practices for the Linux Kernel and other [projects maintained](https://github.com/torvalds/subsurface/blob/a48494d2fbed58c751e9b7e8fbff88582f9b2d02/README#L88-L115) by Linus Torvalds.

## Tests Flight Results

Test flights are important for quality assurance. Please upload the logs from the microSD card to the [Flight Log Review site](http://logs.px4.io/) and share the link on the [PX4 Discuss](http://discuss.px4.io/) along with a verbal flight report.


## Weekly Dev Call {#dev_call}

The PX4 Dev Team syncs up on its weekly dev call.

> **Tip** The dev call is open to all interested developers (not just the core dev team). 
> This is a great opportunity to meet the team and contribute to the ongoing development of the platform.

* TIME: Wednesday 5PM CET, 11AM EST, 8AM PDT \([subscribe to calendar](https://calendar.google.com/calendar/ical/px4.io_fs35jm7ugmvahv5juhhr3tkkf0%40group.calendar.google.com/public/basic.ics)\)
* Uberconference: [www.uberconference.com/lf-dronecode](http://www.uberconference.com/lf-dronecode)
* Phone: +1 415-891-1494
* The agenda is announced in advance on the [PX4 Discuss](http://discuss.px4.io/c/weekly-dev-call)
* Issues and PRs may be labeled [devcall](https://github.com/PX4/Firmware/labels/devcall) to flag them for discussion


## Calendar & Events {#calendar}

The *Dronecode Calendar* shows important events for platform developers and users. 
Select the links below to display the calendar in your timezone (and to add it to your own calendar):
* [Switzerland – Zurich](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Europe%2FZurich)
* [Pacific Time – Tijuana](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=America%2FTijuana)
* [Australia – Melbourne/Sydney/Hobart](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Australia%2FSydney)

The calendar below shows events for the Zurich timezone.


{% raw %}
<iframe src="https://calendar.google.com/calendar/embed?title=Dronecode%20Calendar&amp;mode=WEEK&amp;height=600&amp;wkst=1&amp;bgcolor=%23FFFFFF&amp;src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&amp;color=%23691426&amp;ctz=Europe%2FZurich" style="border-width:0" width="1800" height="600" frameborder="0" scrolling="no"></iframe>
{% endraw %}
