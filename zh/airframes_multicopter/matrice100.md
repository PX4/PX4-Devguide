# Matrice 100

{% raw %}
<video id="my-video" class="video-js" controls preload="auto" width="100%" 
poster="http://image84.360doc.com/DownloadImg/2015/04/1617/52474470_2.jpg" data-setup='{"aspectRatio":"16:9"}'>
  <source src="http://7xw24i.com1.z0.glb.clouddn.com/Matrice%20100%20pixhawk%20px4.mp4" type='video/mp4' >
  <p class="vjs-no-js">
    To view this video please enable JavaScript, and consider upgrading to a web browser that supports HTML5 video
  </p >
</video>
{% endraw %}

![Matrice 100](../../images/airframes/multicopter/matrice100/Matrice100.jpg)

## 部件列表

  * [DJI Matrice 100](http://store.dji.com/product/matrice-100) Just ESC''s motors, and frame.

## 电机连接

![Connections](../../images/airframes/multicopter/matrice100/Wiring Diagram.jpg)

![Wiring Harness](../../images/airframes/multicopter/matrice100/WiringHarness.jpg)

![PWM Connections](../../images/airframes/multicopter/matrice100/PwmInput.jpg)

![Top](../../images/airframes/multicopter/matrice100/Top.jpg)

![Back](../../images/airframes/multicopter/matrice100/Back.jpg)

![No Stack](../../images/airframes/multicopter/matrice100/NoStack.jpg)

![No Top Deck](../../images/airframes/multicopter/matrice100/NoTopDeck.jpg)

| 输出    | 频率     | 执行器     |
| ----- | ------ | ------- |
| MAIN1 | 400 Hz | 右前方，逆时针 |
| MAIN2 | 400 Hz | 左后方，逆时针 |
| MAIN3 | 400 Hz | 左前方，顺时针 |
| MAIN4 | 400 Hz | 右后方，顺时针 |
| AUX1  | 50 Hz  | RC AUX1 |
| AUX2  | 50 Hz  | RC AUX2 |
| AUX3  | 50 Hz  | RC AUX3 |

## 参数

* At high throttle the inner loop causes oscillations with default x quad gains. At low throttle, higher gains give a better response, this suggests that some gain scheduling based on the throttle may improve the overall response and this could be implemented in mc_att_control. For now we will just tune it so that there are no oscillations at low or high throttle, and take the bandwidth hit at low throttle.
  * MC_PITCHRATE_P: 0.05
  * MC_PITCHRATE_D: 0.001
* The battery has 6 cells instead of the default 3
  * BAT_N_CELLS: 6
