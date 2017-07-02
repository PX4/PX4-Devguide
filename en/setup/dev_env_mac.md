# Development Environment on Mac

## Homebrew Installation

The installation of Homebrew is quick and easy: [installation instructions](https://brew.sh).

## Common Tools Needed

After installing Homebrew, run these commands in your shell:

```sh
brew tap PX4/px4
brew install px4-dev
# Optional, but recommended additional simulation tools:
brew install px4-sim

```

If the installation outputs an error message about missing requirements follow the instructions. Your system will be missing Java and Quartz:

```sh
brew cask install xquartz java
```

Install pip if you don't already have it and use it to install the required packages:

```sh
sudo easy_install pip
sudo -H pip install pyserial empy pandas jinja2

```

## Editor / IDE

[Sublime Text](https://www.sublimetext.com) is a fast and lean text editor. If you want to use an integrated development environment, Qt Creator is an option: [Download](http://www.qt.io/download-open-source/#section-6)

## Build and Fly in the Simulator!

Now continue to run the [first build](../setup/building_px4.md)!
