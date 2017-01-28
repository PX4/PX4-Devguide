# Installing Files and Code

The first step is to install Xcode from the Mac app store. Once its installed, open a new terminal and install the command line tools:

<div class="host-code"></div>

```bash
xcode-select --install
```

## Homebrew Installation

Usage of the [Homebrew package manager](http://mxcl.github.com/homebrew/) for Mac OS X is recommended. The installation of Homebrew is quick and easy: [installation instructions](http://mxcl.github.com/homebrew/).

After installing Homebrew, copy these commands to your shell:

<div class="host-code"></div>

```sh
brew tap PX4/homebrew-px4
brew tap osrf/simulation
brew update
brew install git bash-completion genromfs kconfig-frontends gcc-arm-none-eabi
brew install astyle cmake ninja
# simulation tools
brew install ant graphviz sdformat3 eigen protobuf
brew install homebrew/science/opencv
```

Then install the required python packages:

<div class="host-code"></div>

```sh
sudo easy_install pip
sudo pip install pyserial empy pandas jinja2
```

### Java for jMAVSim

If you're intending to use jMAVSim, you need to install [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html).

## Snapdragon Flight

Developers working on Snapdragon Flight should use an Ubuntu VM for the time being and follow the Linux instructions. Qualcomm provides reliable tooling for Ubuntu exclusively. The PX4 dev team had the most consistent experience with VMWare, in particular when it comes to USB stability.

## Simulation

OS X comes with CLANG pre-installed. No further installation steps are required.

## Editor / IDE

And finally download and install the Qt Creator app: [Download](http://www.qt.io/download-open-source/#section-6)

Now continue to run the [first build](starting-building.md)!
