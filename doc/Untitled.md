# Bulbboot

Bulbboot项目基于esp32c3平台，交付智能灯泡启动的第一阶段固件。

第一阶段固件只实现



## 功能列表





## 详细设计





### 蓝牙广播

持续侦听b





### 老化测试

老化测试兼容涂鸦的5灯测试方案，做少许修改。





### 版本

使用一个自己定义的版本格式，类似semver，使用4个字节（uint8_t）分别表示major, minor, patch和prerelease，含义和semer定义相同。例子：

```
# in hex code
01 00 00 a1		# 1.0.0 alpha 1版本
01 00 00 b1		# 1.0.0 beta 1版本
01 00 00 c1		# 1.0.0 release candiate 1版本
01 00 00 00		# 1.0.0 发布版
```

实际使用不会进行版本比较，仅在源码和二进制中使用。



在源码中配置版本的方式是在根目录下的`CMakeLists.txt`文件中：

```cmake
...
set(PROJECT_VER "010000a1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
...
```



在二进制文件中，改版本信息位于 `esp_app_desc_t`数据结构体内，该结构体距离文件头的偏移量是固定的，其中版本信息的数据格式是32字节字符

```c
char version[32];
```



参考：https://docs.espressif.com/projects/esp-idf/en/v4.3.1/esp32c3/api-reference/system/app_image_format.html



项目中实际仅使用前面4个字节，原则上最后一个字节的合法取值只有`[00, a1-af, b1-bf, c1-cf, ff]`，其中`ff`表示dirty版本。开发者应尽量避免分发dirty版本。



在代码内，使用api获取该版本信息并打印。版本在代码里的api和数据包括：

```c
uint8_t version[4];
void ver_init();
```

后者在启动时初始化version字符数组，该数组被ble广播，使用的adt数据格式如下：

```
0x06 		# 长度
0x00		# ADT_DEVICE_INFO，表示该adt是device info数据
0x00		# BULBBOOT，表示该固件是bulbboot，同时也表示了硬件是灯泡设备
version[0]	# major
version[1]	# minor
version[2]	# patch
version[3]	# pre-release
```

版本功能在如下文件中定义和实现：

```
version.h
version.c
```











