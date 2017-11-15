---
translated_page: https://github.com/PX4/Devguide/blob/master/en/tutorials/land_detector.md
translated_sha: 5d3a4b74ed78490309ad921f2a6a78fd092d196d
---

# 着陆探测器配置


着陆探测器是一个表示关键飞行器状态的动态飞行器模型，比如着陆和地面接触。


## 自动上锁配置


默认情况下，着陆探测器会检测到着陆，但不会自动上锁。如果滞后参数[COM_DISARM_LAND](../advanced/parameter_reference.md#COM_DISARM_LAND)设置为非零值，系统将在N秒后自动上锁（N为设定值）。


## 多旋翼着陆探测器配置


完整的参数集可以在QGroundControl地面站的参数编辑器中找到，以`LNDMC`为前缀。每个机身可能不同的关键参数有：

* [MPC_THR_HOVER](../advanced/parameter_reference.md#MPC_THR_HOVER) - 系统的悬停油门（以百分比表示，默认为50％）。 重要的是正确设置，因为它不仅使高度控制更准确，而且确保正确的着陆检测。 无负载安装的竞赛手或大型航拍无人机可能需要低得多的设置（例如35％）。
* [MPC_THR_MIN](../advanced/parameter_reference.md#MPC_THR_MIN) - 整个系统的最小油门。设置此项以启用受控下降。
* [LNDMC_THR_RANGE](../advanced/parameter_reference.md#LNDMC_THR_RANGE) - 这是一个缩放因子，用于定义最小和悬停油门之间可被接受作为着陆油门的范围。Example: If the minimum throttle is 0.1, the hover throttle is 0.5 and the range is 0.2 \(20%\), then the highest throttle value that counts as landed is: `0.1 + (0.5 - 0.1) * 0.2 = 0.18`.


## 固定翼着陆探测器配置

可用以`LNDFW`为前缀的完整参数集。下列两个用户参数有时需要进行调整：

* [LNDFW_AIRSPD_MAX](../advanced/parameter_reference.md#LNDFW_AIRSPD_MAX) - 降落时仍然要考虑系统允许的最大空速。 默认的8 m / s是空速感测精度和足够快的触发之间可靠的折衷。 更好的空速传感器应允许较低的参数值。

* [LNDFW_VELI_MAX](../advanced/parameter_reference.md#LNDFW_VELI_MAX) - 降落时仍然要考虑系统的最大速度。 可以调整此参数，以确保在投掷机身时进行手动启动的着陆检测。