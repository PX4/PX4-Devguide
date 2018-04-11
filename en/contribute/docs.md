# Contributing to Documentation

Contributions to the Dronecode guides, including the PX4 developer and user guides are very welcome!
This article explains how you can make changes, add content, and create translations.

> **Note** You will need a (free) [Github](http://github.com) account to contribute to the guide.

## Quick Changes

Fixing typos or editing an *existing page* is easy:
1. Click the **Edit** toolbar icon at the top of the relevant page in the guide.

   ![Gitbook: Edit Page button](../../assets/gitbook/gitbook_toolbar_icon_edit.png)
   
   This will open the page for editing (in Github).
1. Make the desired change.
1. At the bottom of the page you'll be prompted to create a separate branch and then 
   guided to submit a *pull request*.
   
The documentation team reviews submitted pull requests and will either merge
it or work with you to update it.

## Adding New Content - Big Changes

### What Goes Where?

The *Developer Guide* is for documentation that is relevant to *software developers*.
This includes users who need to: 
* Add or modify platform features - modules, flight modes, etc.
* Add support/integrate with new hardware - flight controllers, peripherals, airframes, etc.
* Communicate with the platform from an external source - e.g. a companion computer.
* Understand the architecture

The *User Guide*, by contrast, is *primarily* for users who want to:
* Fly a vehicle using PX4
* Build, modify, or configure a vehicle using PX4 on a supported/existing airframe.

> **Tip** For example, detailed information about how to build/configure an existing airframe are in the User Guide, 
> while instructions for defining a *new* airframe are in the Developer Guide.


### Gitbook Documentation Toolchain

The guide uses the [Gitbook](https://www.gitbook.com/about) toolchain. 
Change requests can be either done on the Gitbook website using the [Gitbook editor](https://gitbookio.gitbooks.io/documentation/content/editor/index.html) or locally (more flexible, but less user-friendly). 

In order to contribute many changes to the documentation, it is recommended that you follow these steps to add the changes locally and then create a pull request:

* [Sign up](https://github.com/join) for github if you haven't already
* Fork the PX4 user guide from [here](https://github.com/PX4/px4_user_guide) or Dev guide from [here](https://github.com/PX4/Devguide). For instructions to fork a git repository, see [here](https://help.github.com/articles/fork-a-repo/#fork-an-example-repository).
* Clone your forked repository to your local computer<br>
```sh
cd ~/wherever/
git clone https://github.com/<your git name>/px4_user_guide.git
```
* Install gitbook via NPM. At the terminal prompt, simply run the following command to install GitBook:
```sh
npm install gitbook-cli -g
```
* Navigate to your local repository:
```sh
cd ~/wherever/px4_user_guide
```
* To build your book, run:
```sh
gitbook build
```
> **Note** If you run into an error: `/usr/bin/env: node: No such file or directory`, run `ln -s /usr/bin/nodejs /usr/bin/node`

* To preview and serve your book, run:
```sh
gitbook serve
```
> **Note** run `gitbook install` to install missing plugins.

* Now you can browse your local book on http://localhost:4000/
* Exit serving using `CTRL+c` in the terminal prompt.

* You can also serve on a different port instead of 4000:
```sh
gitbook serve --port 4003
```
* You can output as html, pdf, epub or mobi:
```sh
gitbook help
```

Everything you need to install and build Gitbook locally is explained in the 
[toolchain documentation](https://toolchain.gitbook.com/setup.html). 


In overview:

* Pages are written in separate files using markdown \(almost the same syntax used by Github wiki\). 
* The _structure_ of the book is defined in a file named **SUMMARY.md**.
* This is a [multilingual](https://toolchain.gitbook.com/languages.html) book, 
  so there is a **LANGS.md** file in the root directory defining what languages are supported. 
  Pages for each language are stored in the folder named for the associated language code \(e.g. "zh" for Chinese, "en" for English\). 
* A file named **book.json** defines any dependencies of the build.
* A web hook is used to track whenever files are merged into the master branch on this repository, causing the book to rebuild.



## Style guide

1. Files/file names

   * Put new files in an appropriate sub-folder
   * Use descriptive names. In particular, image filename should describe what they contain.
   * Use lower case and separate words using underscores "\_"

2. Images

   * Use the smallest size and lowest resolution that makes the image still useful.
   * New images should be created in a sub-folder of **/assets/** by default 
     (so they can be shared between translations).

3. Content:

   * Use "style" \(bold, emphasis, etc\) consistently. **Bold** for button presses and menu definitions. 
     _Emphasis_ for tool names. Otherwise use as little as possible.
   * Headings and page titles should use "First Letter Capitalisation"
   * The page title should be a first level heading \(\#\). All other headings should be h2 \(\#\#\) or lower.
   * Don't add any style to headings.
   * Don't translate the *first part* of a note, tip or warning declaration (e.g. `> **Note**`) 
     as this precise text is required to render the note properly.

## Translations {#translation}

We have recently started adding translated versions of all the PX4/Dronecode guides! 
If you would like to help, contact us on [our support channels](../README.md#support). 

Gitbook supports translation [as described here](https://toolchain.gitbook.com/languages.html):
* Each language is independent and keeps all its documents in its own directory \(named using it's international code - "en" for English, "es" for Spanish, etc.\) 
* The **LANGS.md** file in the root directory lists the language folders that Gitbook must build.

In order to keep all language-versions of the guide up to date and synchronised, we have the following policy/guidelines:

* This is an **English-first** book.

  * Any _technical_ changes should be made in the English tree _first_ \(including both updates and new pages\). After the English change has been accepted in the main repo then submit the translation. This approach ensures that changes will propagate through to all the other translations!
  * Improvements to translations that don't change "technical information" won't need to be made in the English tree.

* The structure and documents should be the same for all languages \(i.e. based on the English version\).
* All languages share the same images by default \(do not duplicate the _/image_ folder, unless you're changing/translating the image\).
* Translation changes are submitted to the repo in the same way as any other changes \(fork the repo, make a branch for your changes, and create PRs of the branches to submit them into this repo\).
* Translation teams can organise themselves however they like as long as PRs are submitted using the above approach.


### Starting a new language translation

The process straightforward:

1. Fork the documentation repo.
1. Create and checkout a new branch for your language.
   ```
   checkout -b add_translation_language_yourlanguagename
   ```
1. Copy the whole English folder \(/en\) and rename it to the appropriate language code \(e.g. "es" for Spanish\).

   > **Note** This ensures that you keep the same structure and documents as the original version.

1. Update **\LANGS.md** with your language.
1. Translate the content in your language tree.

   > **Tip** Minimally complete the home page and the **SUMMARY.md** before submitting any PR request. Ideally do more!

1. Commit the changes and push them back to your own fork repo.
   ```
   git add *
   git commit -m "Created a your_new_language translation"
   git push origin add_translation_language_yourlanguagename
   ```

1. On the Github interface, create a PR to submit your branch back to the master repo \(a banner appears on Github that you can click when you visit the repo\).

### Updating translations

Translations can be updated like any other change to documentation: fork the repo, create a branch for your changes in your fork, then submit them back to the main repo as PRs.

### Tracking changes

We hope that translation owners will track changes in the English version and propagate them through to their translations.

Git/Github have excellent mechanisms for tracking changes. We recommend that when you add [document front matter](https://toolchain.gitbook.com/pages.html#front-matter) to your translation with the commit information for the page you translated. This allows anyone to go back later and find out whether the text has changed since it was last translated. For example:

```md
---
translated_page: https://github.com/PX4/Devguide/blob/master/en/setup/config_initial.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
translated: false
---
```

> **Note** The *translated_sha* is the full SHA of the commit that you translated. Find this by opening the source page on github, press the **History** button. Find the commit of the document you are translating from (ideally the most recent) and press the "Copy the full SHA" icon associated with that commit.


## Licence

All PX4/Dronecode documentation is free to use and modify under terms of the permissive 
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) licence.
