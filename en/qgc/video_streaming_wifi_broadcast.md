# Long-distance video streaming in QGroundControl

This page shows how to set up a a companion computer with a camera (Logitech C920 or RaspberryPI camera) such that the video stream is transferred from UAV to a ground computer and displayed in the application QGroundControl that runs on this computer. This setup uses WiFi in unconnected (broadcast) mode.

The whole hardware setup consists of the following parts:

On TX (UAV) side:
* NanoPI NEO2 (and/or Raspberry PI if use PI camera)
* Logitech camera C920 or RaspberryPI camera
* WiFi module  ALPHA AWUS051NH v2.

On RX (group station side):
* Any computer with Linux.
* WiFi module with Ralink RT5572 chipset (CSL 300Mbit Stick or GWF-4M02)


## Wifibroadcast

 - 1:1 map RTP to IEEE80211 packets for minimum latency (doesn't serialize to byte steam)
 - Smart FEC support (immediately yeild packet to video decoder if FEC pipeline without gaps)
 - Stream encryption and authentication ([libsodium](https://download.libsodium.org/doc/))
 - Distributed operation. It can gather data from cards on different hosts. So you don't limited to bandwidth of single USB bus.
 - Aggreagation of mavlink packets. Doesn't send wifi packet for every mavlink packet.
 - Enhanced OSD https://github.com/svpcom/wifibroadcast_osd for Raspberry PI (consume 10% CPU on PI Zero)


## Theory:

Wifibroadcast puts the wifi cards into monitor mode. This mode allows to send and receive arbitrary packets without association.
This way a true unidirectional connection is established which mimics the advantageous properties of an analog link. Those are:

 - The transmitter sends its data regardless of any associated receivers. Thus there is no risk of sudden video stall due to the loss of association

 - The receiver receives video as long as it is in range of the transmitter. If it gets slowly out of range the video quality degrades but does not stall.

 - The traditional scheme “single broadcaster – multiple receivers” works out of the box. If bystanders want to watch the video stream with their devices they just have to “switch to the right channel”

 - Wifibroadcast allows you to use several low cost receivers in parallel and combine their data to increase probability of correct data reception. This so-called software diversity allows you to use identical receivers to improve relieability as well as complementary receivers (think of one receiver with an omnidirectional antenna covering 360° and several directional antennas for high distance all working in parallel)

 - Wifibroadcast uses Forward Error Correction to archive a high reliability at low bandwidth requirements. It is able to repair lost or corrupted packets at the receiver.



## Hardware modification.
Alpha WUS051NH is a high power card and eats too much current while TX. If you power it from USB will reset port on most ARM boards.
So you need to connect it to 5V BEC directly. You can do this two ways:

 1. Make a custom usb cable.
 2. Cut a 5V wire on PCB near usb port and wire it to BEC.
    Also I suggest to add 470uF low ESR capacitor (like ESC has) between power and ground to filter voltage spikes.

## Software setup

1. Install libpcap and libsodium development libs.
2. Download wifibroadcast [sources](https://github.com/svpcom/wifibroadcast).
3. [Patch](https://github.com/svpcom/wifibroadcast/wiki/Kernel-patches) your kernel. You only need to patch kernel on TX (except if you want to use WiFi channel which is disabled in your region by CRDA).

### Generate encryption keys

```
make
keygen
```
Copy ``rx.key`` to RX host and ``tx.key`` to TX host.

### On TX side you need:

1a. Setup Logitech camera C920 camera to output RTP stream:
```
gst-launch-1.0 uvch264src device=/dev/video0 initial-bitrate=6000000 average-bitrate=6000000 iframe-period=1000 name=src auto-start=true \
               src.vidsrc ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5600
```
1b. Setup RaspberryPi camera to output RTP stream:
```
raspivid --nopreview --awb auto -ih -t 0 -w 1920 -h 1080 -fps 30 -b 4000000 -g 30 -pf high -o - | gst-launch-1.0 fdsrc ! h264parse !  rtph264pay !  udpsink host=127.0.0.1 port=5600
```

2. Setup wifibroadcast in TX mode:

```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
./scripts/tx_standalone.sh wlan1   # where wlan1 is your wifi TX interface
```
This will setup wifibroadcast using ``MCS #1: QPSK 1/2 40MHz Short GI`` modulation (30 Mbit/s) on 149 WiFi channel (in 5GHz band) and listening on UDP port 5600 for incoming data.

### On RX side you need:

1. Setup wifibroadcast in RX mode:
```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
./scripts/rx_standalone.sh wlan1  # your wifi interface for RX
```

2. Run qgroundcontrol or
```
gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
             ! rtph264depay ! avdec_h264 ! clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false
```
to decode video.

## Enhanced setup with RX antenna array, FPV goggles and OSD
See [wiki](https://github.com/svpcom/wifibroadcast/wiki/enhanced-setup) article

## TODO
1. Write user docs and make prebuild packages. Pull requests are welcome.
2. Do a flight test with different cards/antennas.
3. Investigate how to set TX power without CRDA hacks.
4. Tune FEC for optimal latency/redundancy.
5. Inject packets with radio link RSSI to mavlink stream


## FAQ
Q: What is a difference from original wifibroadcast?

A: Original version of wifibroadcast use a byte-stream as input and splits it to packets of fixed size (1024 by default). If radio
packet was lost and this is not corrected by FEC you'll got a hole at random (unexpected) place of stream. This is especially bad if
data protocol is not resistent to (was not desired for) such random erasures. So i've rewrite it to use UDP as data source and pack one
source UDP packet into one radio packet. Radio packets now have variable size depends on payload size. This is reduces a video latency a lot.

Q: What type of data can be transmitted using wifibroadcast?

A: Any UDP with packet size <= 1466. For example x264 inside RTP or Mavlink.

Q: What are transmission guarancies?

A: Wifibrodcast use FEC (forward error correction) which can recover 4 lost packets from 12 packets block with default settings.
   You can tune it (both TX and RX simultaniuosly!) to fit your needs.

Q: I have a lot of frame drops and messages ``XX packets lost``. What is this?

A: This is can be due to:
   1. Signal power is too low. Use high-power card or annennas with more gain. Use directed antenna on RX side. Use additional RX card for diversity (add wlan2, wlan3, ... to rx program)
   2. Signal power is too high. Especially if you use 30dBm TX indoors. Try to reduce TX power (for example hack CRDA database inside kernel and make
      several regions with power limit 10dBm and 20dBm).
   3. Interference with other WiFi. Try to change WIFI channel and/or WIFI band. CAUTION: Don't use band where you RC TX operates on! Or setup RTL properly to avoid model loss.
      You can increase FEC block size (by default it is 8/12 (8 data blocks and 4 fec blocks), but it will also increase latency. Use additional RX card for diversity (add wlan2, wlan3, ... to rx program)

## Wifi Cards:

The following chipset was tested with wifibroadcast (used by author):
  - Ralink RT3572 (ALFA AWUS051NH v2). Use it for TX
  - Ralink RT5572 (CSL 300Mbit Stick, GWF-4M02 OEM modules). Use it for RX

See [wiki](https://github.com/svpcom/wifibroadcast/wiki/WiFi-hardware) article.

## ARM boards for UAV
 - [Raspberry Pi Zero](https://www.raspberrypi.org/products/raspberry-pi-zero/)

   Pros:
     * Huge community
     * Camera support
     * HW video encoder/decoder with OMX API.

   Cons:
     * Hard to buy outside US (shipping costs >> its price)
     * Slow CPU
     * Only one USB bus
     * 512MB SDRAM

 - [Odroid C0](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G145326484280)

   Pros:
     * Fast CPU
     * EMMC
     * 1GB SDRAM

   Cons:
     * Very sensitive to radio interference
     * Doesn't supported by mainline kernel
     * High cost
     * HW video encoder is broken
     * Bad PCB quality (too thin, ground pins without solder bridges)

 - [NanoPI NEO2](http://www.friendlyarm.com/index.php?route=product/product&product_id=180)

   Pros:
     * ARM 64-bit CPU
     * Very cheap
     * Supported by mainline kernel
     * 3 independent USB busses
     * 1Gbps ethernet port
     * 3 UARTs
     * Very small form-factor
     * Resistent to radio interference

   Cons:
     * Small community
     * 512MB SDRAM
     * No camera interface

 My choice is to use Pi Zero as camera board (encode video) and NEO2 as main UAV board (wifibroadcast, mavlink telemetry, etc)

## Links:
 -  [Original version](https://befinitiv.wordpress.com/wifibroadcast-analog-like-transmission-of-live-video-data/) of wifibroadcast
