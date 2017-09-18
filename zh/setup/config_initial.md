---
translated_page: https://github.com/PX4/Devguide/blob/master/en/setup/config_initial.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
translated: true
---

# 初始配置

在开始开发PX4之前，系统应当以默认配置进行初始化，以确保硬件已正确装配并通过测试。下方视频讲解了 [Pixhawk硬件](https://docs.px4.io/en/flight_controller/pixhawk_series.html)与[QGroundControl](../qgc/README.md)的安装过程。 [这里](../airframes/architecture.md)是已支持的参考机架列表。

> **须知：** [下载DAILY BUILD版本的QGroundControl](http://qgroundcontrol.org/downloads)并跟随下方的视频教程来设置你的飞行器。参考[QGroundControl 教程](../qgc/README.md)来了解任务规划(mission planning)，飞行(flying)和参数整定(parameter setting)的具体细节。

下面的视频将介绍一系列的设置选项

{% raw %}
<video id="my-video" class="video-js" controls preload="auto" width="100%" 
poster="https://files.readme.io/218544b-2016-09-14_09_38_55-QGroundControl_v3.0.0.png" data-setup='{"aspectRatio":"16:9"}'>
  <source src="http://7xvob5.com1.z0.glb.clouddn.com/1-PX4%20Autopilot%20Setup%20Tutorial%20Preview.mp4" type='video/mp4' >
  <p class="vjs-no-js">

    To view this video please enable JavaScript, and consider upgrading to a web browser that
    <a href="http://videojs.com/html5-video-support/" target="_blank">supports HTML5 video</a>
  </p>
</video>
{% endraw %}

## 遥控选项

PX4飞行控制并不强制要求有遥控系统,也不要求必须使用单独的开关来选择飞行模式。

### 不用遥控器进行飞行

所有的遥控装置的检查可以通过将参数`COM_RC_IN_MODE`为` 1 `来禁用，这将不允许手动飞行。但是，除了比如flying in之类的飞行模式。

### 单通道模式切换开关

在这种模式下，系统将接受一个单一的通道作为模式开关，而不是使用多个开关，这在 [旧版wiki](https://pixhawk.org/peripherals/radio-control/opentx/single_channel_mode_switch)有解释。

