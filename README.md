# Smart-home-system-based-on-Raspberry-Pi-pico

空间科学与技术综合性实验。都大四了。。。还有实验课，64个学时就离谱，每周有两整天的实验，在实验室呆着就跟坐牢一样，幸好我手快，选了一个最简单的实验，拿着几个一块钱一个的传感器和一块树莓派pico做一个智能家居系统。
花了差不多一上午就写完了，就是这个温度传感器太差了，一是不准确，冷天的温度有时候28° C。。。而且变化很慢。简单用面包板搭了一下，反映有点迟钝，温控模块显示延迟0.5s，照明系统延迟0.5s，但是可以满足要求。
人体红外检测模块也很拉。。。但是勉强能用。
而且pico只有AD没有DA，输出只能使用 PWM 或者外接DA，没办法，题目要求就是使用最简单的器件实现，否则直接上stm32了。。。

# 实现功能
温控：

当温度超过26°C时打开风扇，随着温度的增加风扇的转速会增加，显示屏会实时显示温度和凤扇转速。

照明：

当环境光照较亮时，LED不亮，当光照逐渐变暗时，LED开始亮，并且亮度随着光照强度变暗而增大，显示屏显示LED的状态

人体检测：

当室内有人时整个系统才会工作，显示屏开始显示，当人离开时系统停止工作，进入低功耗模式。

显示：

显示屏显示温度，风扇转速，光照强度，LED状态，室内是否有人。

# 注
具体代码已经上传，显示屏是ssd1306 128*64，如果数据放不下可以换大一点的（器材要求），具体传感器的连接需要具体实现，这里我用的都是I2C传输的。