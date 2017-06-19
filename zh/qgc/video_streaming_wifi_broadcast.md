---
translated_page: https://github.com/PX4/Devguide/blob/master/en/qgc/video_streaming_wifi_broadcast.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 在QGroundControl软件中实现远距离视频传输

本页面向您展示如何用一台机载卡片电脑（本页面用Odroid C1或C0进行演示）将摄像头（本页面用罗技 C920摄像头进行演示）捕捉的视频通过WIFI实时传输到另一台联网的计算机，计算机上需要安装并运行QGroundControl软件。我们主要通过处于unconnected (broadcast)模式的WIFI来实现这一想法。

下面列出了所有的需要的硬件：

TX发送端（无人机）：
* Odroid C1卡片电脑
* 罗技 C920 摄像头
* ALPHA AWUS051NH v2无线模块

RX接收端（地面站）：
* 任何运行Linux系统的计算机
* ALPHA AWUS051NH v2无线模块

## 为什么普通的Wifi协议不适用于长距离视频传输？
采用普通Wifi协议有如下弊端：

- “关联性”过强：视频发送方和接收方需要建立关联关系。如果任意一方设备断开连接（例如仅因接收方网络信号质量过差）视频传输就会立即停止。
-  要求“无错传输”：普通Wifi协议仅接收正确的数据包，若发现数据有差错则不予理会。这意味着在进行FPV视频传输时，一丁点数据的错误就会导致丢包。虽然数据的正确性得到了严格保证，但这样严格的数据校验会导致视频卡顿。
-  通信的“双向性”：即便您只发送了一个简单的数据，普通Wifi也要求接收方在收到数据后给出一个“回应”以确认接收方确实收到了数据包。如果发送方没有收到接收方的“回应”，它就将关闭连接。因此，如果使用普通Wifi协议，您就必须在无人机和地面站设置配置同规格的设备和天线。如果使用普通Wifi协议，您就不可以在无人机上使用全向天线，而在地面站使用高增益的定向天线了。
 - 自适应传输速率: 普通Wifi连接会在信号弱时自动切换到较低的传输速率，而协议自动选择的较低的传输速率不一定就适合视频的传送。在这种情况下很可能会导致不可预料的延迟，高达数秒之久。
 - 点对点传输：普通Wifi协议规定数据只能点对点传输。在这种情况下，除非您使用广播帧（ broadcast frames）或者其他类似的技术，旁观者无法像收看依赖模拟信号传输的视频（analog video transmission）一样容易地看到您所看到的图像。
 - 受限的多样性：普通Wifi协议会限制网卡提供不同的数据流。

## Wifi广播技术（WifiBroadcast）的优势
Wifi广播技术（WifiBroadcast）会让无线网卡处于监听模式。监听模式允许网卡接收任意到来的数据包，而不用验证数据包来源（不用验证关联性），而且还允许接收校验失败的帧数据。这样就相当于建立了一个真单向的连接，它具备了和模拟信号一样的优势。
以下是采用Wifi广播技术（WifiBroadcast）的优势：

 - 发送方不论接收方是否存在都将广播数据，因此不会因为接收方失去连接而导致视频中断。
 - 只要在发送方的信号覆盖内，接收器就可以得到图像。就算是信号不好，视频图像上也只是会显示干扰而不会终止传送。
 - 实现了“单发送 – 多接收”，如果其他人想要观看您的视频流，他们只需要切换到正确的频道即可。
- Wifi广播技术（WifiBroadcast）允许你同时使用多个廉价接收器并将接收到的数据组合起来以增加数据正确的概率。这种协议上的支持允许你使用多个接收器（比如就可以使用一个全向天线接收器再加上几个定向天线接收器协同工作来增加传输距离）来获得更好的传输质量。
 - Wifi广播技术（WifiBroadcast）使用前向纠错技术（Forward Error Correction） ，能够在低带宽的情况下提高传输可靠性，它能修复一定的数据错误。

## 硬件改装
Alpha WUS051NH 网卡在发射时的功率会较高，如果你用USB直接插接在Odroid C1/C0卡片电脑上，就会带走更多电流导致隐患。因此你需要直接用5V BEC供电，有两种方法可以做到:

 1. 自己做一根USB线
 2. 在Odroid卡片电脑的PCB板上，然后将直接将供电连接到BEC
 我建议在电源和地之间加一个470uF的低容量电容(像ESC一样)来滤除电压峰值。

## 软件安装
下载Wifibroadcast [sources](https://github.com/svpcom/wifibroadcast).

您需要打几个Linux内核补丁来:
 
 1. 打开TX速率锁（TX rate lock），请安装``mac80211-radiotap-bitrate_mcs_rtscts.linux-4.4.patch``补丁。否则没有办法固定的数据包（injected radiotap packets）的发送速率。
 2. 打开TX电源锁（TX power lock），请安装 ``ez-wifibroadcast-1.4-kernel-4.4-patches.diff``补丁。这会锁定网卡在支持的最大发送电压（maximum tx power）。
 3. 让发送方的数据包校验失效，请安装 ``ez-wifibroadcast-1.4-kernel-4.4-patches.diff``补丁。这个操作可以不做，目前代码中不需要。

这样你将只能在TX发送端打补丁了。

###在TX发送端您需要:

1. 配置相机输出RPT流:
```
gst-launch-1.0 uvch264src device=/dev/video0 initial-bitrate=6000000 average-bitrate=6000000 iframe-period=1000 name=src auto-start=true \
               src.vidsrc ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5600
```
 2. 将Wifibroadcast设置成TX发送模式:

```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
ifconfig wlan1 down    #假设发送网卡是wlan1
iw reg set BO          #区域设置为B0，以最大功率发送
iw dev wlan1 set monitor otherbss fcsfail
ifconfig wlan1 up
iwconfig wlan1 channel 149
./tx -r 24 wlan1
```
这将会将Wifibroadcast设置成数据传输速率24Mbit/s、WIFI通道（wifi channel）149、5GHz 频段并且监听5600端口。

### 在RX发送端您需要:

 1. 将Wifibroadcast设置为RX模式:
```
git clone https://github.com/svpcom/wifibroadcast
cd wifibroadcast
make
ifconfig wlan1 down    #假设发送网卡是wlan1
iw reg set BO          #Region和TX发送方一样，确保可以监听
iw dev wlan1 set monitor otherbss fcsfail
ifconfig wlan1 up
iwconfig wlan1 channel 149
./rx wlan1
```
 2. 运行Q Groundcontrol软件或者输入以下命令解码视频：
```
gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
             ! rtph264depay ! avdec_h264 ! clockoverlay valignment=bottom ! autovideosink fps-update-interval=1000 sync=false
```

## FAQ
Q: 这样做和传统的Wifi广播有什么不同?

A:原来的版本使用单字节输入，并将UDP数据包分割成固定大小（默认1024字节）通过无线电分别发送。如果无线电数据包丢失或者没有被FEC修复，你会得到不完整的数据流（相当于数据上面出现了一个破洞）。这对于一些不希望数据包丢失一部分的协议来说是很大的隐患。 因此我重写了代码，将一个UDP数据包完整的放入一个无线电数据包中发送，这样就可以让无线电数据包根据数据的大小有不同的尺寸了，这大大降低了延迟。

Q: 什么样的数据适合用wifibroadcast传输呢?

A: 任何小于1466字节的UDP包，比如RTP和Mavlink中的x264编码。

Q: 如何确保数据的准确性？

A: Wifibrodcast使用FEC (forward error correction) ，默认可以从12个数据块中恢复出4个丢失的数据包。
   你可以同时调整TX发送端和RX接收端来满足自己的需求。

Q: 我碰到了很多丢帧的情况，并看到了 ``XX packets lost``这样的提示，这是怎么回事?

A: 这是由于:
   1. 信号质量过差，尝试更高功率的网卡，或者更高增益的天线。在接收端使用定向天线，并添加更多网卡
   2. 信号功率过高，特别是当在室内使用30dBm的发送的时候。尝试减小发送功率(比如修改内核中的CRDA数据库并且将某些区域（regions）的功率设置在10dBm到20dBm之间)
   3. 收到了其他Wifi的干扰。尝试更换Wifi频道或者WIFI频段。注意：不要在无人机运行的时候，或者选用合适的RTL以免弄丢飞机！ 
      您可以增加FEC块在数据包中的比例（初始值为8/12（8个数据块，4个FEC块）），但是修改这个值可能会导致延迟。您可以添加更多RX接收网卡选择不同设置。

## TODO
1. 使用不同的天线/网卡进行飞行测试
2. 研究一下不用修改CRDA就能设置TX发送功率的方法
3. 将FEC调整到合适的延迟/冗余.

## 已知可用的Wifi网卡:

使用以下Atheros芯片的均可:

 -  Atheros AR9271, Atheros AR9280, Atheros AR9287


使用以下Ralink芯片的均可:

 -  RT2070, RT2770, RT2870, RT3070, RT3071, RT3072, RT3370, RT3572, RT5370, RT5372, RT5572

但是有可能会因为一些原因导致某些网卡不可用，因此推荐您使用已经经过测试过的网卡来保证绝对可用。：

 -  CSL 300Mbit Stick (2.4/5Ghz, Diversity, RT5572 chipset)
 -  Alfa AWUS036NHA (2.3/2.4Ghz, high power, Atheros AR9271 chipset)
 -  TPLink TL-WN722N (2.3/2.4Ghz, Atheros AR9271 chipset)
 -  ALFA AWUS051NH v2 (2.4Ghz/5Ghz, high power, Ralink RT3572 chipset)
 -  ALFA AWUS052NH v2 (2.4Ghz/5Ghz, Diversity, high power, Ralink chipset)
 -  TP-Link-TL-WDN3200 (2.4/5Ghz, Diversity, RT5572 chipset)
 -  Ralink RT5572 (2.4/5Ghz, Diversity???, RT5572 chipset)

另外，如果每个人都用相同的网卡，我们就不知道还有哪些网卡可以工作了。中国的厂商有提供RT5370这款小而轻的网卡，售价在4美元以下。Aliexpress上也有很多廉价的网卡。如果你发现了上面没有列出的网卡也能用，请告诉我们~

* AWUS036NHA 这款适配器能够提供大约280mW的功率输出。有人尝试用吸盘天线达到过几公里的范围。
* TL-WN722N 这款适配器能够提供大约60mW的功率输出。使用2.1dbi  可以打到大约800-1000m的距离。
重要事项：在特定情形下，PCB板上的第二根天线会影响接收信号质量。请像下面一样移除PCB板后面的白色组件。（在图片中，组件被焊接到上面的板子上，这样如果后悔的话可以改回去）

* CSL 300Mbit stick 这款适配器能够提供大约30mw的功率输出。 由于使用了5Ghz的频率，它的传输距离不是很长在200-300m左右。由于吸盘天线是2.4Ghz 2.1dbi长袖偶极（sleeved-dipole）天线，它不能用在5Ghz的频率上。

当被用作Rx接收端时，信号强度大于-20dbm可能会导致造成数据在传输过程中损坏。这种问题可以通过增加多个适配器，并使用指向不同方向或偏振的天线来解决。

* AWUS051NH 这款适配器能够提供大约330mw的功率输出。在5GHZ频率下大概传输距离在800-1000m左右。不推荐使用吸盘天线，因为吸盘天线有5dbi的增益，会导致过于平坦的散射模式（too-flat radiation pattern）。

* AWUS052NH 这款适配器能够提供大约330mw的功率输出。它和051NH一样，只是没有两条TX链（TX chains）。不推荐使用吸盘天线，因为吸盘天线有5dbi的增益，会导致过于平坦的散射模式（too-flat radiation pattern）。

## 相关链接:
 -  [Original version](https://befinitiv.wordpress.com/wifibroadcast-analog-like-transmission-of-live-video-data/) of wifibroadcast
