---
translated_page: https://github.com/PX4/Devguide/blob/master/en/advanced/telemetry_wifi.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# Wifi Telemetry

PX4 supports telemetry via UDP and Wifi. It broadcasts a heartbeat to port 14550 on 255.255.255.255 until it receives the first heartbeat from a ground control station, at which point it will only send data to this ground control station.
