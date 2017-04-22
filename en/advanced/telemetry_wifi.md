# Wifi Telemetry

PX4 supports telemetry via UDP and Wifi. It broadcasts a heartbeat to port 14550 on 255.255.255.255 until it receives the first heartbeat from a ground control station, at which point it will only send data to this ground control station.
