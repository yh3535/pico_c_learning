# 供电相关

详见pico-datasheet.pdf第7页。

VBUS：micro-USB输入电压，与micro-USB端口1直接相连，一般5V（若未使用USB，则为0V）。

VSYS ：系统主要输入电压，用于整板供电。供电范围1.8~5.5V，板载SMPS使用它来产生3.3V电压驱动RP2040和GPIO。与VBUS通过一个二极管相连，当使用USB供电时其大小为VBUS - Von，Von为二极管导通电压。

3V3_EN：连接到SMPS使能引脚。拉低此引脚使SMPS停止工作，从而断开RP2040供电。

3V3：RP2040和GPIO供电，由SMPS产生。可用于驱动外部电路，最高输出电流由RP2040负载和VSYS决定，但建议不超过300mA。

ADC_VREF：ADC供电及参考电压。由Pico通过过滤3.3V电压得到。若需要更好的ADC性能，也可以使用外部参考电压。