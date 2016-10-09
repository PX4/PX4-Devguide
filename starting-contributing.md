# Contributing

Contact information for the core dev team and community can be found below. The PX4 project uses a three-branch Git branching model:

  * [master](https://github.com/px4/firmware/tree/master) is by default unstable and sees rapid development.
  * [beta](https://github.com/px4/firmware/tree/beta) has been thoroughly tested. Its intended for flight testers.
  * [stable](https://github.com/px4/firmware/tree/stable) points to the last release branch.

We try to retain a [linear history through rebases](https://www.atlassian.com/git/tutorials/rewriting-history) and avoid the [Github flow](https://guides.github.com/introduction/flow/). However, due to the global team and fast moving development we might resort to merges at times.

To contribute new functionality, [sign up for Github](https://help.github.com/articles/signing-up-for-a-new-github-account/), then [fork](https://help.github.com/articles/fork-a-repo/) the repository, [create a new branch](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/), add your changes, and finally [send a pull request](https://help.github.com/articles/using-pull-requests/). Changes will be merged when they pass our [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration) tests.

All contributions have to be under the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause) and all code must not impose any further constraints on the use.

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

## Tests Flight Results

Test flights are important for quality assurance. Please upload the logs from the microSD card to [Log Muncher](http://logs.uaventure.com) and share the link on the [forum](http://groups.google.com/group/px4users) along with a verbal flight report.

## Forums and Chat

  * [Google+](https://plus.google.com/117509651030855307398)
  * [Gitter](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
  * [PX4 Users Forum](http://groups.google.com/group/px4users)

## Weekly Dev Call

The PX4 Dev Team syncs up on its weekly dev call (connect via [Mumble](http://mumble.info) client).

  * TIME: 19:00h Zurich time, 1 p.m. Eastern Time, 10 a.m. Pacific Standard Time
  * Server: mumble.dronecode.org
  * Port: 10028
  * Channel: PX4 Channel
  * The agenda is announced the same day on the [px4users forum](http://groups.google.com/group/px4users)
