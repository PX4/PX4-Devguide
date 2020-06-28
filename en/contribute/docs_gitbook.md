# Building the Gitbook Locally

Everything you need to install and build Gitbook locally is also explained in the [toolchain documentation](https://github.com/GitbookIO/gitbook/blob/master/docs/setup.md). 

1. Install nodejs on your computer (version 4-6 recommended).
1. Install gitbook via NPM. 
   At the terminal prompt, simply run the following command to install GitBook:
   ```sh
   npm install gitbook-cli -g
   ```
1. Navigate to your local repository:
   ```sh
   cd ~/wherever/px4_user_guide
   ```
1 Install gitbook dependencies:
  ```sh
  gitbook install
  ```
  > **Note** If you run into an error: `/usr/bin/env: node: No such file or directory`, run `ln -s /usr/bin/nodejs /usr/bin/node`

1. Build your book:
   ```sh
   gitbook build
   ```
1. To preview and serve your book, run:
  ```sh
  gitbook serve
  ```
  * Now you can browse your local book on http://localhost:4000/
  * Exit serving using `CTRL+c` in the terminal prompt.
1. You can also serve on a different port instead of 4000:
   ```sh
   gitbook serve --port 4003
   ```
1. You can also output as html, pdf, epub or mobi:
   ```sh
   gitbook help
   ```