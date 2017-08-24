---
translated_page: https://github.com/PX4/Devguide/blob/master/en/airframes_vtol/README.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 垂直起降



事实上，[PX4飞控系统](../concept/flight_stack.md)支持所有的垂直起降机型配置：

- 尾座式 (X/+型布局的双/四旋翼)
- 倾转式 (Firefly Y6)
- 复合式 (飞机+四旋翼)

垂直起降飞行器与其它种类的飞行器共享代码库，所不同的仅仅是增加了额外的控制逻辑，特别是转换阶段。

<aside class="note">
所有的这些垂直起降飞行器都经过了积极地试飞，并且已经准备好以供使用了。确保为飞行器添加了空速传感器，系统需要根据空速信息决定是否可以安全地进行转换。
</aside>

## 关键配置参数

创建新的垂直起降飞行器的配置时，需要正确设置下面这些参数。

- `VT_FW_PERM_STAB` 系统在悬停模式下使用姿态增稳。如果这个参数设置为1，那么在飞机模式下也会使用姿态增稳；如果这个参数设置为0，那么飞机模式下将会使用纯手动飞行。
- `VT_ARSP_TRANS`参数决定飞行器悬停状态转换到前飞状态的空速阈值，这个值设置过小会导致转换阶段的失速现象。 
- `RC_MAP_TRANS_SW` should be assigned to a RC switch before flight. This allows you to check if the multicopter- and fixed wing mode work properly. (Can also be used to switch manually between those two control modes during flight)


## 尾座式

[构建日志](https://docs.px4.io/en/frames_rover/traxxas_stampede.html)包括更加详细的信息。

{% raw %}
<video id="my-video" class="video-js" controls preload="auto" width="100%" 
poster="http://image84.360doc.com/DownloadImg/2015/04/1617/52474470_2.jpg" data-setup='{"aspectRatio":"16:9"}'>
  <source src="http://7xw24i.com1.z0.glb.clouddn.com/PX4%20VTOL%20-%20Call%20for%20Testpilots.mp4" type='video/mp4' >
  <p class="vjs-no-js">
    To view this video please enable JavaScript, and consider upgrading to a web browser that supports HTML5 video
  </p >
</video>
{% endraw %}

## 倾转式

[构建日志](https://docs.px4.io/en/frames_vtol/vtol_tailsitter_caipiroshka_pixracer.html)包括所有的设置以及操作指南，这些会指导整个系统顺利运行。

{% raw %}
<video id="my-video" class="video-js" controls preload="auto" width="100%" 
poster="http://image84.360doc.com/DownloadImg/2015/04/1617/52474470_2.jpg" data-setup='{"aspectRatio":"16:9"}'>
  <source src="http://7xw24i.com1.z0.glb.clouddn.com/PX4%20Flight%20Core%20controlling%20FireFly%20Y6%20VTOL%20in%20forward%20and%20back%20transition.mp4" type='video/mp4' >
  <p class="vjs-no-js">
    To view this video please enable JavaScript, and consider upgrading to a web browser that supports HTML5 video
  </p >
</video>
{% endraw %}

## 复合式

[构建日志](https://pixhawk.org/platforms/vtol/fun_cub_quad_vtol)包括详细的指南指导如何构建以及复现线面的结果。

{% raw %}
<video id="my-video" class="video-js" controls preload="auto" width="100%" 
poster="http://image84.360doc.com/DownloadImg/2015/04/1617/52474470_2.jpg" data-setup='{"aspectRatio":"16:9"}'>
  <source src="http://7xw24i.com1.z0.glb.clouddn.com/Fun%20Cub%20PX4%20VTOL%20Maiden.mp4" type='video/mp4' >
  <p class="vjs-no-js">
    To view this video please enable JavaScript, and consider upgrading to a web browser that supports HTML5 video
  </p >
</video>
{% endraw %}

{% raw %}
<video id="my-video" class="video-js" controls preload="auto" width="100%" 
poster="http://image84.360doc.com/DownloadImg/2015/04/1617/52474470_2.jpg" data-setup='{"aspectRatio":"16:9"}'>
  <source src="http://7xw24i.com1.z0.glb.clouddn.com/-PX4%20Autopilot-%20-%20Experimental%20-VTOL-%20with%20-Pixhawk-%20and%20-U-Blox%20M8N%20GPS-.mp4" type='video/mp4' >
  <p class="vjs-no-js">
    To view this video please enable JavaScript, and consider upgrading to a web browser that supports HTML5 video
  </p >
</video>
{% endraw %}


