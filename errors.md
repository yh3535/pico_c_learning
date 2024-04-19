# 常见错误即解决

以下是一些调试过程中常见的错误及解决方案。

## Failed to connect multidrop rp2040.dap0

已连接到调试器（即Probe），但调试器未正确连接到树莓派Pico。请检查被调试的Pico板接线。

## Error: unable to find a matching CMSIS-DAP device

可能是调试器固件太老，尝试升级picoprobe或者把`interface/cmsis-dap.cfg`改为`interface/picoprobe.cfg`。

## Can't find interface/picoprobe.cfg

新版调试器固件（大概1.0.2之后）可直接使用`interface/cmsis-dap.cfg`。`picoprobe.cfg`已被弃用。

## pico Timers not counting when debugging

## Hardware timer not working after reset under debugger raspberrypi/pico-sdk

I got another CMSIS based probe to test. Turns out this is not a bug with the pico probe or openocd.

The default configuration for openocd 0.12rc2 with an rp2040 target will debug both cores with dedicated gdb instances. With my configuration this meant the second core was always stopped as I never connected the second gdb instance. The rp2040 timer is set up by default to stop counting when any core is stopped by a debugger and so the timer would never advance.

You can use `set USE_CORE 0` in the openocd config to select just the first core for debugging and the problem disappears. Multicore debugging with a single gdb instance is not yet supported with upstream openocd on the rp2040.

https://github.com/raspberrypi/debugprobe/issues/45

另：

I had the same problem and can confirm that adding `timer_hw->dbgpause = 0;` to my code fixed the problem for me.

https://github.com/raspberrypi/pico-sdk/issues/1152

因此解决该问题需要：

* 在rp2040.cfg开头加set USE_CORE 0 （不可以在vscode launch.json的openocd命令参数加，无效）