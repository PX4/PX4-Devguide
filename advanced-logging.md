# Logging

This describes the new logger, `SYS_LOGGER` set to 1.

The logger is able to log any ORB topic with all included fields. Everything
necessary is generated from the `.msg` file, so that only the topic name needs
to be specified. An optional interval parameter specifies the maximum logging
rate of a certain topic. All existing instances of a topic are logged.

The output log format is [ULog](advanced-ulog-file-format.md).


## Configuration
The list of logged topics can be customized with a file on the SD card. Create a
file `etc/logging/logger_topics.txt` on the card with a list of topics:
```
<topic_name>, <interval>
```
The `<interval>` is optional, and if specified, defines the minimum interval in
ms between two logged messages of this topic. If not specified, the topic is
logged at full rate.

The topics in this file replace all of the default logged topics.


## Scripts
There are several scripts to analyze and convert logging files in the
[pyulog](https://github.com/PX4/pyulog) repository.


## Dropouts
Logging dropouts are undesired and there are a few factors that influence the
amount of dropouts:
- SD Card: usually the more expensive cards from well-known manufacturers are
  better. All SD cards we tested exhibit multiple pauses per minute. This shows
  itself as a several 100 ms delay during a write command. It causes a dropout
  if the write buffer fills up during this time.
- Formatting an SD card can help to prevent dropouts.
- Increasing the log buffer helps.
- Decrease the logging rate of selected topics or remove unneeded topics from
  being logged (`info.py -v <file>` is useful for this).


