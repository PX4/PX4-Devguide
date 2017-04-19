# Long-distance video streaming in QGroundControl
This page shows how to set up a a companion computer (Odroid C1 or C0) with a camera (Logitech C920) such that the video stream is transferred via the Odroid C1 to a network computer and displayed in the application QGroundControl that runs on this computer. This setup uses WiFi in unconnected (broadcast) mode.

The whole hardware setup consists of the following parts:

On TX (copter) side:
* Odroid C1
* Logitech camera C920
* WiFi module  ALPHA AWUS051NH v2.

On RX (group station side):
* Any computer with Linux.
* WiFi module  ALPHA AWUS051NH v2.


## Why normal wifi is a bad choice for long-distance video transfer
 - Association: Video transmitter and receiver need to be associated. If one device looses association (for example due to too weak signal strength) the video transmission stops instantly.
 - Error-free transmission: Wifi transmits either data that is correct or no data. In an FPV scenario this means that even if you received data with just small errors it would be rejected completely. This could result in stalling video although you have received useful data.
 - Two-way communication: Even if you are sending data only from source to sink a bi-directional data flow is required using wifi. The reason for this is that a wifi receiver needs to acknowledge the received packets. If the transmitter receives no acknowledgements it will drop the association. Therefore, you would need equally strong transmitters and antennas both on the aircraft and on the ground station. A setup with a strong transmitter in the air using an omnidirectional antenna and a weak device on the ground using a high-gain antenna is not possible with normal wifi.
 - Rate control: Normal wifi connections switch automatically to a lower transmission rate if signal strength is too weak. Due to this it is possible that the (automatically) selected rate is too low to transfer the video data. This way the data would queue up and introduce an unpredictable latency that can be up to several seconds.
 - One to one transfers: Unless you use broadcast frames or similar techniques a normal wifi data flow is a one to one connection. A scenario where a bystander just locks onto your “channel” as in analog video transmission to watch your stream is not easy to accomplish using traditional wifi.
 - Limited diversity: Normal wifi limits you to the number of diversity streams that your wifi card offers.

## What wifibroadcast makes different
Wifibroadcast puts the wifi cards into monitor mode. This mode allows to send and receive arbitrary packets without association. Additionally, it is also possible to receive erroneous frames (where the checksum does not match). This way a true unidirectional connection is established which mimics the advantageous properties of an analog link. Those are:

 - The transmitter sends its data regardless of any associated receivers. Thus there is no risk of sudden video stall due to the loss of association
 - The receiver receives video as long as it is in range of the transmitter. If it gets slowly out of range the video quality degrades but does not stall. Even if frames are erroneous they will be displayed instead of being rejected.
 - The traditional scheme “single broadcaster – multiple receivers” works out of the box. If bystanders want to watch the video stream with their devices they just have to “switch to the right channel”
 - Wifibroadcast allows you to use several low cost receivers in parallel and combine their data to increase probability of correct data reception. This so-called software diversity allows you to use identical receivers to improve relieability as well as complementary receivers (think of one receiver with an omnidirectional antenna covering 360° and several directional antennas for high distance all working in parallel)
 - Wifibroadcast uses Forward Error Correction to archive a high reliability at low bandwidth requirements. It is able to repair lost or corrupted packets at the receiver.


## Hardware modification.
Alpha WUS051NH is a high power card and eats too much current while TX. If you power it from USB will reset port on Odroid C1/C0.
So you need to connect it to 5V BEC directly. You can do this two ways:

 1. Make a custom usb cable.
 2. Cut a 5V wire on odroid PCB near usb port and wire it to BEC.
    Also I suggest to add 470uF low ESR capacitor (like ESC has) between power and ground to filter voltage spikes.

## Software setup
Download wifibroadcast [sources](https://github.com/svpcom/wifibroadcast).

You need to patch kernel to:
 
 1. Enable TX rate lock. Use ``mac80211-radiotap-bitrate_mcs_rtscts.linux-4.4.patch``. Instead there are no way to specify data rate for injected radiotap packets.
 2. Enable TX power lock. Use ``ez-wifibroadcast-1.4-kernel-4.4-patches.diff``. This will lock tx power to maximum supported by card.
 3. Enable RX of frames with bad FSC (checksum). Use ``ez-wifibroadcast-1.4-kernel-4.4-patches.diff``. This is optional and don't use in current code.

So you can only patch kernel on TX side.

### On TX side you need:

1. Setup camera to output RTP stream:
```
gst-launch-1.0 uvch264src device=/dev/video0 initial-bitrate=6000000 average-bitrate=6000000 iframe-period=1000 name=src auto-start=true \
               src.vidsrc ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5600
```
 2. Setup wifibroadcast in TX mode:

```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
ifconfig wlan1 down    #assume that TX card is wlan1
iw reg set BO          #set CRDA region with max allowed TX power
iw dev wlan1 set monitor otherbss fcsfail
ifconfig wlan1 up
iwconfig wlan1 channel 149
./tx -r 24 wlan1
```
This will setup wifibroadcast using 24Mbit/s data rate on 149 wifi channel (in 5GHz band) listening on UDP port 5600 for incoming data.

### On RX side you need:

 1. Setup wifibroadcast in RX mode:
```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
ifconfig wlan1 down    #assume that RX card is wlan1
iw reg set BO          #set the same region as TX to ensure that you can listen TX channel
iw dev wlan1 set monitor otherbss fcsfail
ifconfig wlan1 up
iwconfig wlan1 channel 149
./rx wlan1
```
 2. Run qgroundcontrol or
```
gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
             ! rtph264depay ! avdec_h264 ! clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false
```
to decode video.

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

## TODO
1. Do a flight test with different cards/antennas.
2. Investigate how to set TX power without CRDA hacks.
3. Tune FEC for optimal latency/redundancy.

## Wifi Cards:

The following Atheros chipsets should work:

 -  Atheros AR9271, Atheros AR9280, Atheros AR9287

The following Ralink chipsets should work:

 -  RT2070, RT2770, RT2870, RT3070, RT3071, RT3072, RT3370, RT3572, RT5370, RT5372, RT5572

However, there might be whatever small issues that prevent some cards from working, so if you want to play it safe, choose one of the cards that have been tested by different people and definitely work:

 -  CSL 300Mbit Stick (2.4/5Ghz, Diversity, RT5572 chipset)
 -  Alfa AWUS036NHA (2.3/2.4Ghz, high power, Atheros AR9271 chipset)
 -  TPLink TL-WN722N (2.3/2.4Ghz, Atheros AR9271 chipset)
 -  ALFA AWUS051NH v2 (2.4Ghz/5Ghz, high power, Ralink RT3572 chipset)
 -  ALFA AWUS052NH v2 (2.4Ghz/5Ghz, Diversity, high power, Ralink chipset)
 -  TP-Link-TL-WDN3200 (2.4/5Ghz, Diversity, RT5572 chipset)
 -  Ralink RT5572 (2.4/5Ghz, Diversity???, RT5572 chipset)

On the other hand, if everybody gets the same cards, we'll never find out which other ones work. There are also very small and lightweight RT5370 cards available in china shops for under 4$. Aliexpress for example has a lot of cheap wifi cards in general. It would be nice if you report back your findings in case you tried a wifi card that is not listed here.

* AWUS036NHA This adapter will provide around 280mW output power. Ranges of several kilometers have been reported (with directional antennas though).

* TL-WN722N This adapter will provide around 60mW output power. Range should be roughly around 800-1000m with 2.1dbi stock antennas. IMPORTANT: Under certain circumstances, the second antenna on the PCB causes bad reception. Please disconnect the antenna by removing the white PCB component on the back of the PCB like shown below (in the picture, the component was soldered to the upper pad to be able to reverse the mod if needed)

* CSL 300Mbit stick This adapter provides around 30mw output power. Range on 5Ghz is not very high, around 200-300m. Stock antennas are not usable on 5Ghz, as they are simple 2.4Ghz 2.1dbi sleeved-dipole antennas.

When used as an Rx dongle, bad blocks can occur when the received signal strength is higher than -20dbm. This can be worked-around by using more than one adapter and pointing antennas in different directions / polarizations.

* AWUS051NH This adapter will provide around 330mw output power. Range on 5Ghz is around 800-1000m. Stock antenna is not recommended because they have 5dbi gain, which will give a too-flat radiation pattern.

* AWUS052NH This adapter will provide around 330mw output power. This is the same adapter as the 051NH, but with two TX chains. Stock antennas are not recommended because they have 5dbi gain, which will give a too-flat radiation pattern.

## Links:
 -  [Original version](https://befinitiv.wordpress.com/wifibroadcast-analog-like-transmission-of-live-video-data/) of wifibroadcast
