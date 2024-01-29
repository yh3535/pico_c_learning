# 为Pico SDK增加Fortran支持

Pico SDK原生并没有加入对Fortran编译器的支持，但arm-none-eabi-gcc显然支持Fortran（注意，按照Pico官方文档在Debian上使用apt安装gcc-arm-none-eabi时所安装的包并不包含arm-none-eabi-gfortran，必须从[arm官方页面](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)下载工具链才包含gfortran编译器）。因此需要对SDK中的CMake文件进行一定的修改。

首先在`pico-sdk/cmake/preload/toolchains/pico_arm_gcc.cmake`中加入对Fortran编译器的识别：

```cmake
pico_find_compiler(PICO_COMPILER_FORTRAN ${PICO_GCC_TRIPLE}-gfortran)
set(CMAKE_Fortran_COMPILER ${PICO_COMPILER_FORTRAN} CACHE FILEPATH "Fortran compiler")
```

第一行用于找到交叉编译器，可以放在`# Find GCC for ARM.`这一段末尾；第二行指定Pico SDK使用的Fortran编译器，放在`# Specify the cross compiler.`这一段末尾。

然后为Fortran编译器设置编译器标志，需要在`pico-sdk/cmake/preload/toolchains/set_flags.cmake`中`foreach(LANG IN ITEMS C CXX ASM)`这一行的`CXX`后增加`Fortran`，改为`foreach(LANG IN ITEMS C CXX Fortran ASM)`。

最后，由于嵌入式gfortran编译器始终无法通过cmake的编译器检查（因为编译器检查用到`PRINT`语句，涉及到一些特殊的库，而嵌入式编译器显然没有实现这些库），需要强制跳过检查，在`pico-sdk/cmake/preload/toolchains/set_flags.cmake`最后增加一行：

```cmake
set(CMAKE_Fortran_COMPILER_WORKS 1)
```

然后，在项目的CMakeLists.txt中增加Fortran语言，即可：

```cmake
project(TARGET C CXX Fortran ASM)
```

现在就可以编译并链接Fortran源代码了。