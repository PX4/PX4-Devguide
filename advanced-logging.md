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
- Most SD cards we tested exhibit multiple pauses per minute. This shows
  itself as a several 100 ms delay during a write command. It causes a dropout
  if the write buffer fills up during this time. This effect depends on the SD
  card (see below).
- Formatting an SD card can help to prevent dropouts.
- Increasing the log buffer helps.
- Decrease the logging rate of selected topics or remove unneeded topics from
  being logged (`info.py <file>` is useful for this).

## SD Cards
The following provides performance results for different SD cards.
Tests were done on a Pixracer; the results are applicable to Pixhawk as well.

| SD Card | Mean Seq. Write Speed [KB/s] | Max Write Time / Block (average) [ms] |
| -- | -- | -- |
| SanDisk Extreme U3 32GB | 461 | **15** |
| Sandisk Ultra Class 10 8GB | 348 | 40 |
| Sandisk Class 4 8GB | 212 | 60 |
| SanDisk Class 10 32 GB (High Endurance Video Monitoring Card) | 331 | 220 |
| Lexar U1 (Class 10), 16GB High-Performance | 209 | 150 |
| Sandisk Ultra PLUS Class 10 16GB | 196 | 500 |
| Sandisk Pixtor Class 10 16GB | 334 | 250 |
| Sandisk Extreme PLUS Class 10 32GB | 332 | 150 |

More important than the mean write speed is the maximum write time per block (of
4 KB). This defines the minimum buffer size: the larger this maximum, the larger
the log buffer needs to be to avoid dropouts. Logging bandwidth with the default
topics is around 50 KB/s, which all of the SD cards satisfy.

By far the best card we know so far is the **SanDisk Extreme U3 32GB**. This
card is recommended, because it does not exhibit write time spikes (and thus
virtually no dropouts). Different card sizes might work equally well, but the
performance is usually different.

You can test your own SD card with `sd_bench -r 50`, and report the results to
https://github.com/PX4/Firmware/issues/4634.
