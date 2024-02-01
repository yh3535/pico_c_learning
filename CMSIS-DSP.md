# 在Pico中使用CMSIS-DSP

一般并不建议在Pico中使用CMSIS，因为它没有针对cortex M0+的优化，因此速度甚至比不上自己手写的[^1]。但有时仍然需要使用，因此这里给出Cmake的配置方法。

首先git克隆https://github.com/ARM-software/CMSIS_5和https://github.com/ARM-software/CMSIS-DSP这两个项目，注意，后者曾经是前者的一部分，但后来独立出去了，因此仍然能在前者的CMSIS目录下看到一个DSP文件夹，里面有一个说明文档解释了这一点。这里为了统一存放位置，我将后者复制到前者的CMSIS/DSP目录下（即还原独立前的目录结构），以下示例全部基于此。

使用pico-project-generator新建项目test-dsp，然后在CMakeFiles.txt中增加：

```
set(CMSISCORE "/home/yh/pico/CMSIS_5/CMSIS/Core")
set(CMSISDSP "/home/yh/pico/CMSIS_5/CMSIS/DSP")
```

这两行定义两个项目的存储位置。

然后，根据官方说明，增加：

```
add_subdirectory(${CMSISDSP}/Source bin_dsp)
```

再然后，要使用库中的函数与类型，需要头文件目录：

```
target_include_directories(test-dsp PRIVATE
        ${CMSISDSP}/Include
        ${CMSISCORE}/Include
)
```

最后，指定链接CMSISDSP库：

```
target_link_libraries(test-dsp 
        CMSISDSP
        )
```

现在可以在C源程序中通过`#include "arm_math.h"`来使用CMSIS-DSP。

链接时，链接器只会链接使用到的二进制程序到最终文件里，但是结果文件仍然十分庞大，这是不可避免的。

## 参考资料

[^1]:来自https://forums.raspberrypi.com/viewtopic.php?t=349861，但实际测试似乎并不支持这一结论