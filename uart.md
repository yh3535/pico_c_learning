# UART注意事项

以下是uart使用过程中的注意事项。

## 1、波特率

当使用uart0作为stdio，无法使用uart_set_baud_rate()或者uart_init()等函数更改uart0的波特率。要么改为使用uart1，要么不使用uart0而是使用usb作为stdio：  

```cmake
    # enable usb output, disable uart output
    pico_enable_stdio_usb(target_name 1)
    pico_enable_stdio_uart(target_name 0)
```

参考：https://forums.raspberrypi.com/viewtopic.php?t=330762