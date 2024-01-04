# Pico PWM使用

Pico具有16个可控的PWM channel，可用于PWM的30个GPIO引脚具体分布如下：

![./images/PWM-Pins-Pi-Pico-640x152.svg](./images/PWM-Pins-Pi-Pico-640x152.svg)

总共8个（编号从0到7）独立的PWM切片（slice），每个切片分为A和B两个channel，共16个channel，同一个切片的A channel和B channel的频率只能相同，但是可以设置不同的输出占空比。其中B 引脚还可用作频率和占空比测量的输入。也就是说，每个切片可以驱动两个频率相同但是占空比可以不同的PWM输出信号，或者测量一个输入信号的频率和占空比。

同一个channel的频率和占空比完全相同，即使是不同的GPIO引脚。譬如GPIO 0和GPIO 16，都是0A channel，因此更改其中任意一个的频率或占空比，另一个也会跟着改变。

## 使用解释

以下对官方示例做出中文解释。

```c
// 在GPIO 0和GPIO 1输出PWM信号
#include "pico/stdlib.h"
#include "hardware/pwm.h"

int main()
{
    // 把GPIO 0 和 1分配作PWM用
    gpio_set_function(0, GPIO_FUNC_PWM);
    gpio_set_function(1, GPIO_FUNC_PWM);

    // 找出GPIO 0对应的PWM切片号，这里是切片0
    uint slice_num = pwm_gpio_to_slice_num(0);

    // 设置PWM周期（period）为4个时钟周期（cycle）（从0到3）
    pwm_set_wrap(slice_num, 3);
    // 设置切片0的A channel，也就是GPIO 0，高电平时间为1个时钟周期（cycle），即占空比25%
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
    // 设置切片0的B channel，也就是GPIO 1，高电平时间为3个时钟周期（cycle），即占空比75%
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
    // 启动PWM
    pwm_set_enabled(slice_num, true);


    // 也可以使用 pwm_set_gpio_level(gpio, x)设置高电平时钟周期数，它可以自动识别GPIO的切片号和channel
}
```

实际上，有三个可以设置的值来更改PWM输出的频率与占空比：

* clock_divide

  通过`pwm_set_clkdiv(slice_num , clock_divide)`函数对系统时钟进行分频。PWM使用分频之后的时钟作为真正的计数时钟。这个值最大不超过255，最小为1。如果不设置，默认为1，即系统时钟。每个切片的这个值是独立的，但同一个切片的不同channel是相同的。

* wrap_value

  PWM计数周期长度，通过`pwm_set_wrap(slice_num, wrap_value)`设置，表示一个PWM周期需要从0计数到x，共有x+1个计数时钟周期。每个切片的这个值是独立的，但同一个切片的不同channel是相同的。

* flip_value

  PWM在计数多少个周期后从高电平跳到低电平。通过`pwm_set_chan_level(slice_num, PWM_CHAN_A, flip_value)`或者`pwm_set_gpio_level(gpio, flip_value)`设置。这个值除以wrap_value+1之后就是占空比，每个channel都可以不一样。

因此，PWM的频率可以如下计算：

```c
freq = clock_sys / clock_divide / (wrap_value+1);
```

占空比可以如下计算：

```c
duty_cycle = flip_value / (wrap_value + 1);
```



## 参考资料

1. https://how2electronics.com/pwm-usage-in-raspberry-pi-pico-breathing-led-example/

2. https://forums.raspberrypi.com/viewtopic.php?t=309632#p1851891