# Development Environment on Mac

## Command Line Tools

The first step is to install Xcode from the Mac app store. Once its installed, open a new terminal and install the command line tools:

```sh
xcode-select --install
```

## Homebrew Installation

Usage of the [Homebrew package manager](http://mxcl.github.com/homebrew/) for Mac OS X is recommended. The installation of Homebrew is quick and easy: [installation instructions](http://mxcl.github.com/homebrew/).

## Common Tools Needed

After installing Homebrew, copy these commands to your shell:

```sh
brew tap PX4/px4
brew update
brew install git bash-completion genromfs kconfig-frontends gcc-arm-none-eabi
brew install astyle cmake ninja
```

Install pip if you don't already have it:

```sh
sudo easy_install pip
```

Then use it to install the required packages:

```sh
sudo pip install pyserial empy pandas jinja2
```

## Gazebo Simulation

```
brew cask install xquartz
brew tap PX4/simulation
brew install graphviz protobuf
brew install homebrew/science/opencv
```


### jMAVSim simulation

If you're intending to use jMAVSim, you need to install [Java JDK 8](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html), as well as
ant:

```sh
brew install ant
```

## Snapdragon Flight

Developers working on Snapdragon Flight should use an Ubuntu VM (or a docker setup) for the time being and follow the Linux instructions. Qualcomm provides reliable tooling for Ubuntu exclusively. The PX4 dev team had the most consistent experience with VMWare, in particular when it comes to USB stability.

## Editor / IDE

If you want to use an IDE, Qt Creator is an option: [Download](http://www.qt.io/download-open-source/#section-6)

Now continue to run the [first build](../setup/building_px4.md)!
