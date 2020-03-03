# 系统通知声音

PX4 定义了一些用于为系统状态和问题提供音频通知的 [standard tones/tunes](https://docs.px4.io/master/en/getting_started/tunes.html) (比如系统启动，解锁成功，电池警告等）

使用字符串指定音乐 ([ANSI Music notation](http://artscene.textfiles.com/ansimusic/information/ansimtech.txt))，并使用 [tunes](https://github.com/PX4/Firmware/tree/master/src/lib/tunes) 库播放。 声音库也包含默认系统调节列表——见 [lib/tunes/tune_definition.desc](https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc)。

PX4还有一个模块可以用于播放 (测试) 默认或用户自定义声音。

本主题提供了如何创建您自己的声音并添加/替换系统通知音调/乐曲的通用指导。


## 创建曲子

乐曲字符串使用 [ANSI Music notation](http://artscene.textfiles.com/ansimusic/information/ansimtech.txt) 定义。

> **Tip** 关于格式的更多信息见 [QBasic PLAY statement](https://en.wikibooks.org/wiki/QBasic/Appendix#PLAY) (Wikibooks) 并已转载于 [tune_definition.des](https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc)。

创建新乐曲的最简单方式是使用音乐编辑器。 这允许您编辑乐曲并在您的电脑上播放， 然后导出为 PX4 可以播放的格式。

ANSI 音乐在 ANSI BBS 系统中很受欢迎，因此最好的编辑工具是 DOS 实用工具。 在 Windows 上，一个选项是在 *Dosbox* 内使用 *Melody Master*。

使用软件的步骤是：

1. Download [DosBox](http://www.dosbox.com/) and install the app
1. Download [Melody Master](ftp://archives.thebbs.org/ansi_utilities/melody21.zip) and unzip into a new directory
1. Open the *Dosbox* console
1. Mount the melody master directory in Dosbox as shown below:
   ```
   mount c C:\<path_to_directory\Melody21
   ```
1. Start *Melody Master* with the following commands
   ```
   c:
   start
   ```
1. You will then have the option to click through a few screens, then press **1** to display *Melody Master*: ![Melody Master 2.1](../../assets/tunes/tunes_melody_master_2_1.jpg)

   The lower half of the screen provides helpful advice on keyboard shortcuts for using the tool (arrows for moving in stave, and numbers for selecting the note length, etc.).
1. When you're ready to save the music:
   - Press **F2** to give the tune a name and save it in the */Music* sub folder of your Melody Master installation.
   - Press **F7**, the scroll down the list of output formats on the right to get to ANSI. The file will be exported to the *root* of the Melody Master directory (with the same name and a file-type specific extension).
1. Open the file. The output might look like this:

   ![ANSI Output from file](../../assets/tunes/tune_musicmaker_ansi_output.png)

1. The string that can be played in PX4 is the bit between `MNT` and `P64`: `150L1O3DL16CL32<B>C<AEL16A`


## Testing Tunes

When you're ready to try it out a new tune on PX4, use the [tune_control](../middleware/modules_system.md#tunecontrol) library. For example, to test the tune we "created" above you would enter the following command on a console or shell (e.g. the [MAVLink Shell](../debug/system_console.md#mavlink_shell)):
```sh
tune_control play -m "150L1O3DL16CL32<B>C<AEL16A"
```

> **Note** Out of the box, the tune_control is only present on real hardware (not the simulator).


## Replacing Existing Tunes

Tunes are defined within [tune_definition.desc](https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc).

If you just need to replace an existing tune, then you can replace the file in your own fork, and update the tune strings defined in `PX4_DEFINE_TUNE`.


## Adding a New Tune


TBD.


<!-- 

1. Assumption is that you need to define a new `PX4_DEFINE_TUNE` with its own number in the file.
2. Need to look at how tunes are played. Problem for another day.

-->