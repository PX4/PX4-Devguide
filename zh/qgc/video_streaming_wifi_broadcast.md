# QGroundControl 的远程视频流

此页演示如何使用相机（Logitech C920 或 RaspberryPi 相机）设置配套计算机，这样的视频流从无人机传输到地面计算机并以 *QGroundControl* 显示。 此设置使用未连接 (广播) 模式下的 wifi 和 [Wifibroadcast project](https://github.com/svpcom/wifibroadcast/wiki) 中的软件。

## 无线广播概述

*Wifibroadcast 项目 * 旨在模仿使用模拟链路传输高清视频（和其他）数据在使用 wifi 无线电时的优势特性。 例如, 它尝试提供视频馈送, 该视频馈送会随着信号的降低/距离而缓慢地降低。

> **Note** 在使用 *Wifibroadcast* 检查规章是否允许在您的国家使用这种 wifi。

*Wifibroadcast* 的高级别优势包括:

- 通过将每个传入 RTPS 数据包编码到单个 WiFi （IEEE80211） 数据包并立即发送（不序列化到字节流），最大限度地减少延迟。
- 智能 FEC 支持（如果 FEC 管道没有间隔，立即将数据包提供给视频解码器）。
- 流加密和身份验证 ([libsodium](https://download.libsodium.org/doc/))
- 分布式操作。 它可以从不同主机上的卡中收集数据，以便带宽不限于单个 USB 总线。
- Aggregation of MAVLink packets. It doesn't send WiFi packet for every MAVLink packet.
- [Enhanced OSD for Raspberry Pi](https://github.com/svpcom/wifibroadcast_osd) (consumes 10% CPU on Pi Zero).

有关详细信息，请参阅 [FAQ](#faq)。

## 硬件安装

硬件由如下部分组成：

在发送端（无人机）：

- [NanoPI NEO2](http://www.friendlyarm.com/index.php?route=product/product&product_id=180) (或者 Raspberry Pi 如果使用 Pi camera).
- [Logitech camera C920](https://www.logitech.com/en-us/product/hd-pro-webcam-c920?crid=34) 或者 [Raspberry Pi camera](https://www.raspberrypi.org/products/camera-module-v2/).
- WiFi 模块 [ALPHA AWUS051NH v2](https://www.alfa.com.tw/products_show.php?pc=67&ps=241).

在接收端（地面站）：

- Any computer with Linux (tested on Fedora 25 x86-64).
- WiFi module with Ralink RT5572 chipset ([CSL 300Mbit Sticks](https://www.amazon.co.uk/high-performance-gold-plated-technology-Frequency-adjustable/dp/B00RTJW1ZM) or [GWF-4M02](http://en.ogemray.com/product/product.php?t=4M02)). OEM modules are cheap but you need to order them from China. CSL stick is expensive but available on ebay. See [wifibroadcast wiki > WiFi hardware](https://github.com/svpcom/wifibroadcast/wiki/WiFi-hardware) for more information on supported modules.

## 硬件设置

Alpha WUS051NH is a high power card that uses too much current while transmitting. If you power it from USB it will reset the port on most ARM boards. So you need to connect it to 5V BEC directly. You can do this two ways:

1. Make a custom USB cable. [You need to cut `+5V` wire from USB plug and connect it to BEC](https://electronics.stackexchange.com/questions/218500/usb-charge-and-data-separate-cables)
2. Cut a `+5V` wire on PCB near USB port and wire it to BEC. Don't do this if doubt. Use custom cable instead! Also I suggest to add 470uF low ESR capacitor (like ESC has) between power and ground to filter voltage spikes. Be aware of [ground loop](https://en.wikipedia.org/wiki/Ground_loop_%28electricity%29) when using several ground wires.

## 软件设置

1. Install **libpcap** and **libsodium** development libs.
2. Download [wifibroadcast sources](https://github.com/svpcom/wifibroadcast).
3. [Patch](https://github.com/svpcom/wifibroadcast/wiki/Kernel-patches) your kernel. You only need to patch the kernel on TX (except if you want to use a WiFi channel which is disabled in your region by CRDA).

### 生成加密密钥

    make
    keygen
    

Copy `rx.key` to RX host and `tx.key` to TX host.

### UAV Setup (TX)

1. Setup camera to output RTP stream:
    
    a. Logitech camera C920 camera: 
    
        gst-launch-1.0 uvch264src device=/dev/video0 initial-bitrate=6000000 average-bitrate=6000000 iframe-period=1000 name=src auto-start=true \
                   src.vidsrc ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5600 b. RaspberryPi camera: ```raspivid --nopreview --awb auto -ih -t 0 -w 1920 -h 1080 -fps 30 -b 4000000 -g 30 -pf high -o - | gst-launch-1.0 fdsrc ! h264parse !  rtph264pay !  udpsink host=127.0.0.1 port=5600```

2. Setup *Wifibroadcast* in TX mode:
    
        git clone https://github.com/svpcom/wifibroadcast
        cd wifibroadcast
        make
        ./scripts/tx_standalone.sh wlan1   # wlan1 的位置是你的 WiFi 发送界面
        
    
    This will setup wifibroadcast using `MCS #1: QPSK 1/2 40MHz Short GI` modulation (30 Mbit/s) on 149 WiFi channel (in 5GHz band) and listening on UDP port 5600 for incoming data.

### Ground Station Setup (RX)

1. Setup *Wifibroadcast* in RX mode:
    
        git clone https://github.com/svpcom/wifibroadcast
        cd wifibroadcast
        make
        ./scripts/rx_standalone.sh wlan1  # 你的用于接收的 WiFi 界面
        

2. Run qgroundcontrol or
    
        gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
                 ! rtph264depay ! avdec_h264 clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false
        
    
    to decode video.

## Enhanced setup with RX antenna array, FPV goggles and OSD

See [wiki](https://github.com/svpcom/wifibroadcast/wiki/enhanced-setup) article. Using RX setup above (and ALPHA AWUS051NH v2 as TX) I was able to receive stable 1080p video on 1-2km in any copter pitch/roll angles.

## FAQ

#### What are the limitations of normal WiFi for long-distance video transfer?

Normal WiFi has the following problems when used for long distance video transfer:

- **Association:** The video transmitter and receiver need to be "associated". If one device looses association (for example due to weak signal strength) then video transmission stops instantly.

- **Two-way communication:** Even if you are sending data only from source to sink a bi-directional data flow is required using WiFi. The reason for this is that a WiFi receiver needs to acknowledge the received packets. If the transmitter receives no acknowledgments it will drop the association. Therefore, you would need equally strong transmitters and antennas both on the aircraft and on the ground station. A setup with a strong transmitter in the air using an omni-directional antenna and a weak device on the ground using a high-gain antenna is not possible with normal WiFi.

- **Rate control:** Normal WiFi connections switch automatically to a lower transmission rate if signal strength is too weak. This can result in an (automatically) selected rate that is too low to transfer video data. The result is that data can queue up and introduce an unpredictable latency of up to several seconds.

- **One to one transfers:** Unless you use broadcast frames or similar techniques, a normal WiFi data flow is a one-to-one connection. A scenario where a bystander just locks onto your "channel" to watch your stream (as is possible in analog video transmission) is not easy to accomplish using traditional WiFi.

- Limited diversity: Normal WiFi limits you to the number of diversity streams that your WiFi card offers.

#### How does Wifibroadcast overcome these limitations

*Wifibroadcast* puts the WiFi cards into monitor mode. This mode allows to send and receive arbitrary packets without association. This way a true unidirectional connection is established which mimics the advantageous properties of an analog link. Those are:

- The transmitter sends its data regardless of any associated receivers. Thus there is no risk of sudden video stall due to the loss of association
- The receiver receives video as long as it is in range of the transmitter. If it gets slowly out of range the video quality degrades but does not stall.
- The traditional scheme “single broadcaster – multiple receivers” works out of the box. If bystanders want to watch the video stream with their devices they just have to “switch to the right channel”
- *Wifibroadcast* allows you to use several low cost receivers in parallel and combine their data to increase probability of correct data reception. This so-called software diversity allows you to use identical receivers to improve reliability as well as complementary receivers (think of one receiver with an omnidirectional antenna covering 360° and several directional antennas for high distance all working in parallel)
- *Wifibroadcast* uses Forward Error Correction (FEC) to archive a high reliability at low bandwidth requirements. It is able to repair lost or corrupted packets at the receiver.

#### How does the new Wifibroadcast differ from the original project?

The [original version of wifibroadcast](https://befinitiv.wordpress.com/wifibroadcast-analog-like-transmission-of-live-video-data/) shares the same name as the [current project](https://github.com/svpcom/wifibroadcast/wiki), but does not derive any code from it.

The original version used a byte-stream as input and split it to packets of fixed size (1024 by default). If a radio packet was lost and this was not corrected by FEC you'll got a hole at random (unexpected) place in the stream. This is especially bad if the data protocol is not resistant to (was not designed for) such random erasures.

The new version has been rewritten to use UDP as data source and pack one source UDP packet into one radio packet. Radio packets now have variable size depends on payload size. This is reduces a video latency a lot.

#### What type of data can be transmitted using Wifibroadcast?

Any UDP with packet size <= 1466. For example x264 inside RTP or MAVLink.

#### What are transmission guarantees?

*Wifibroadcast* uses Forward Error Correction (FEC) which can recover 4 lost packets from a 12 packet block with default settings. You can tune both TX and RX (simultaneously) to fit your needs.

#### What can cause multiple frame drops and messages `XX packets lost`?

This can be due to:

1. Signal power is too low. Use high-power card or antennas with more gain. Use directed antenna on the RX side. Use additional RX card for diversity (add wlan2, wlan3, ... to RX program)
2. Signal power is too high. Especially if you use 30dBm TX indoors. Try to reduce TX power (for example hack CRDA database inside kernel and make several regions with power limit 10dBm and 20dBm).
3. Interference with other WiFi. Try to change WiFi channel and/or WiFi band.
    
    > **Caution** Don't use band that the RC TX operates on! Or setup RTL properly to avoid model loss.

You can increase FEC block size (by default it is 8/12 - 8 data blocks and 4 FEC blocks), at the cost of increasing latency. Use additional RX card for diversity (add wlan2, wlan3, ... to RX program)

#### What ARM Boards are recommended for the UAV?

| Board                                                                                    | Pros                                                                                                                                                                                             | Cons                                                                                                                                                                                                                                            |
| ---------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Raspberry Pi Zero](https://www.raspberrypi.org/products/raspberry-pi-zero/)             | - Huge community  
- Camera support  
- HW video encoder/decoder with OMX API.                                                                                                                   | - Hard to buy outside US (shipping costs >> its price)  
- Slow CPU  
- Only one USB bus  
- 512MB SDRAM                                                                                                                                        |
| [Odroid C0](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G145326484280)  | - Fast CPU  
- EMMC  
- 1GB SDRAM                                                                                                                                                                | - Very sensitive to radio interference  
- Doesn't supported by mainline kernel  
- High cost  
- HW video encoder is broken  
- Bad PCB quality (too thin, ground pins without [thermal relief](https://en.wikipedia.org/wiki/Thermal_relief)) |
| [NanoPI NEO2](http://www.friendlyarm.com/index.php?route=product/product&product_id=180) | - ARM 64-bit CPU  
- Very cheap  
- Supported by mainline kernel  
- 3 independent USB busses  
- 1Gbps Ethernet port  
- 3 UARTs  
- Very small form-factor  
- Resistant to radio interference | - Small community  
- 512MB SDRAM  
- No camera interface                                                                                                                                                                                       |

This article chose to use Pi Zero as camera board (encode video) and NEO2 as main UAV board (wifibroadcast, MAVLink telemetry, etc.)

## TODO

1. Make prebuilt packages. Pull requests are welcome.
2. Do a flight test with different cards/antennas.
3. Investigate how to set TX power without CRDA hacks.
4. Tune FEC for optimal latency/redundancy.
5. Inject packets with radio link RSSI to MAVLink stream