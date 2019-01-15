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
- MAVLink 数据包聚合。 它不会为每个 MAVLink 数据包发送 WiFi 数据包。
- [Enhanced OSD for Raspberry Pi](https://github.com/svpcom/wifibroadcast_osd) （在 Pi Zero 上消耗10% 的 CPU）。

有关详细信息，请参阅 [FAQ](#faq)。

## 硬件安装

硬件由如下部分组成：

在发送端（无人机）：

- [NanoPI NEO2](http://www.friendlyarm.com/index.php?route=product/product&product_id=180) (或者 Raspberry Pi 如果使用 Pi camera).
- [Logitech camera C920](https://www.logitech.com/en-us/product/hd-pro-webcam-c920?crid=34) 或者 [Raspberry Pi camera](https://www.raspberrypi.org/products/camera-module-v2/).
- WiFi 模块 [ALPHA AWUS051NH v2](https://www.alfa.com.tw/products_show.php?pc=67&ps=241).

在接收端（地面站）：

- 任何使用 linux 的计算机 (在 fedora 25 x86-64 上测试)。
- 带有 ralink rt5572 芯片组 ([CSL 300Mbit Sticks](https://www.amazon.co.uk/high-performance-gold-plated-technology-Frequency-adjustable/dp/B00RTJW1ZM) or [GWF-4M02](http://en.ogemray.com/product/product.php?t=4M02)) 的 wifi 模块。 oem 模块价格低廉, 但您需要从中国订购。 csl 棒是昂贵的, 但可在 ebay 上购买。 更过兼容的模块请参考 [wifibroadcast wiki > WiFi hardware](https://github.com/svpcom/wifibroadcast/wiki/WiFi-hardware) 。

## 硬件设置

Alpha WUS051NH 是一种高功率卡, 在传输时使用大的电流。 如果您从 USB 供电, 它将导致大多数的 ARM 板子的端口被重置。 因此, 您需要将其直接连接到 5V BEC。 你可以通过这种方式实现：

1. 自制 USB 电缆。 [你需要从 USB 插头中切断 `+5V` 线，并将其连接到 BEC](https://electronics.stackexchange.com/questions/218500/usb-charge-and-data-separate-cables)
2. 在 USB 端口附近的 PCB 上切割 `+5V` 线，并将其连接到 BEC。 如果有疑问，不要这样做。 改为使用自定义电缆! 还建议在电源和接地之间添加 470uf 低 ESR 电容器 (如电调电容器) 来过滤电压峰值。 使用多根地线时，请注意 [ground loop](https://en.wikipedia.org/wiki/Ground_loop_%28electricity%29)。

## 软件设置

1. 安装 **libpcap** 和 **libsodium** 开发库。
2. 下载 [wifibroadcast sources](https://github.com/svpcom/wifibroadcast)。
3. [Patch](https://github.com/svpcom/wifibroadcast/wiki/Kernel-patches) 内核。 You only need to patch the kernel on TX (except if you want to use a WiFi channel which is disabled in your region by CRDA).

### 生成加密密钥

    make
    keygen
    

将 `rx.key` 复制到RX主机，将` tx.key `复制到TX主机。

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

### 地面站

1. Setup *Wifibroadcast* in RX mode:
    
        git clone https://github.com/svpcom/wifibroadcast
        cd wifibroadcast
        make
        ./scripts/rx_standalone.sh wlan1  # 你的用于接收的 WiFi 界面
        

2. Run qgroundcontrol or
    
        gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
                 ! rtph264depay ! avdec_h264 clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false
        
    
    to decode video.

## RX 天线阵列, fpv 眼镜和 osd 的高级设置

请参阅 [wiki](https://github.com/svpcom/wifibroadcast/wiki/enhanced-setup) 文章。 使用上面的 rx 设置 (和 alha awus05nh v2 作为 tx), 我能够在任何飞行器的俯仰横滚角度的1-2 公里距离接收稳定的1080p 视频。

## 常见问题

#### 普通 wifi 对远程视频传输有哪些限制？

普通 wifi 在用于远距离视频传输时存在以下问题:

- **Association:** 视频的发射机和接收机必须配对 如果一台设备丢失关联 (例如, 由于信号强度较弱), 则视频传输会立即停止。

- **Two-way communication:**即使您仅从一个源头发射数据，WiFi连接依然会建立双向的数据流。 这样做的原因是 wifi 接收器需要确认接收到的数据包。 如果发射机没有收到任何确认, 它将删除关联。 因此，飞机和地面站都需要同样强大的发射机和天线。 使用普通WiFi无法使用高增益天线使用全向天线和地面上的弱设备在空中建立强大的发射机。

- **Rate control:** 如果信号强度太弱，普通WiFi连接会自动切换到较低的传输速率。 这可能导致（自动）选择的速率太低而无法传输视频数据。 结果是数据可以排队并引入长达几秒的不可预测的延迟。

- **One to one transfers:** 除非您使用广播帧或类似的技术, 否则正常的 wifi 数据流是一对一的连接。 使用传统 wifi, 不容易有这样一种情况: 旁观者只需锁定您的 "频道" 就可以观看您的流 (在模拟视频传输中可能是可能的)。

- 有限的多样性: 正常的 wifi 将您限制在您的 wifi 卡提供的多样性流的数量。

#### Wifibroadcast 如何克服这些限制

*Wifibroadcast* 将 wifi 卡置于监控模式。 此模式允许在没有关联的情况下发送和接收任意数据包。 通过这种方式建立了一个真正的单向连接, 它模仿了模拟链路的有利特性。 它们是:

- 发射器发送其数据, 而不考虑任何关联的接收器。 Thus there is no risk of sudden video stall due to the loss of association
- 接收器接收视频, 只要它是在发射机的范围内。 如果它慢慢超出范围, 视频质量会降低, 但不会停止。
- 传统的 "单播音员-多个接收机" 方案开箱即用。 如果旁观者想用他们的设备观看视频流, 他们只需要 "切换到正确的频道"
- *Wifibroadcast* 允许您并行使用多个低成本接收器, 并将其数据组合在一起, 以提高正确接收数据的可能性。 This so-called software diversity allows you to use identical receivers to improve reliability as well as complementary receivers (think of one receiver with an omnidirectional antenna covering 360° and several directional antennas for high distance all working in parallel)
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
- Resistant to radio interference | - 小的社区群体  
- 512MB SDRAM  
- 没有相机接口                                                                                                                                                                                                             |

This article chose to use Pi Zero as camera board (encode video) and NEO2 as main UAV board (wifibroadcast, MAVLink telemetry, etc.)

## 待完成

1. 制作预编译的包。 Pull requests are welcome.
2. 使用不同的卡天线进行飞行测试。
3. Investigate how to set TX power without CRDA hacks.
4. 调整 fec 以获得最佳的延迟冗余。
5. 将无线链路 rssi 值注入到 MAVLink 流的数据包