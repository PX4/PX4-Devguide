---
translated_page: https://github.com/PX4/Devguide/blob/master/en/simulation/airsim.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# AirSim仿真

AirSim是一个基于虚幻引擎([Unreal Engine](https://www.unrealengine.com/zh-CN))的开源、跨平台无人机模拟器。它可以使用硬件在环（HITL）或软件在环（SITL）的方式为Pixhawk / PX4提供逼真的物理和视觉仿真。

相关的主要文档请参阅[Github AirSim README](https://github.com/Microsoft/AirSim/blob/master/README.md).

## HITL

硬件在环仿真环境的设置说明在[这里](https://github.com/Microsoft/AirSim/blob/master/docs/prereq.md).

The walkthrough video below shows the setup for HITL with Pixhawk in more detail.

{% raw %}
<video id="my-video" class="video-js" controls preload="auto" width="100%" 
poster="../pictures/poster/log_viewer.png" data-setup='{"aspectRatio":"16:9"}'>
  <source src="http://7xvob5.com2.z0.glb.qiniucdn.com/AirSimDemo.mp4" type='video/mp4' >
  <p class="vjs-no-js">
    To view this video please enable JavaScript, and consider upgrading to a web browser that
    <a href="http://videojs.com/html5-video-support/" target="_blank">supports HTML5 video</a>
  </p>
</video>
{% endraw %}

## SITL

The instructions for [SITL setup are here](https://github.com/Microsoft/AirSim/blob/master/docs/prereq.md).