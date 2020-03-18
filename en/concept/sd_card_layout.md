# PX4 SD Card Layout

The PX4 SD Card is used for storing configuration files, flight logs, mission information etc. 

> **Tip** We recommend that you format the SD Card as FAT32 before use.

The directory structure/layout is shown below.

Directory/File(s) | Description
--- | ---
/etc/ | Extra config (+ mixers). See [System Startup > Replacing the System Startup](../concept/system_startup.md#replacing-the-system-startup).
/etc/mixers/ | [Mixers](../concept/mixing.md)
/log/ | Full [flight logs](../log/logging.md)
/mission_log/ | Reduced flight logs
/fw/ | [UAVCAN](../uavcan/README.md) firmware
/uavcan.db/ | UAVCAN DB + logs
/params | Parameters (if not in FRAM/FLASH)
/dataman | Mission storage file
/fault_<datetime>.txt | Hardfault files
/bootlog.txt | Boot log file
