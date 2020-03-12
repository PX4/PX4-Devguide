# PX4 SD Card Layout

The PX4 SD Card is used for storing configuration files, flight logs, mission information etc. 

> **Note** The file system is FAT32.

The directory structure/layout is shown below.

Directory/File(s) | Description
--- | ---
/etc/ | Extra config (+ mixers)
/etc/mixers/ | [Mixers](../concept/mixing.md)
/log/ | Full [flight logs](../log/logging.md)
/mission_log | Reduced flight logs
/fw/ | [UAVCAN](../uavcan/README.md) firmware
/uavcan.db/ | UAVCAN DB + logs
/params | Parameters (if not in FRAM/FLASH)
/dataman | Mission storage file
/fault_<datetime>.txt | Hardfault files
/bootlog.txt | Boot log file
