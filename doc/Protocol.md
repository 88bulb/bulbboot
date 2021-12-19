# Project BulbBoot

## 简介

`BulbBoot`是基于`esp32c3`平台的一个智能灯泡项目里的升级和启动程序。



`BulbBoot`的业务目标是智能灯泡上电启动后，根据收到的蓝牙指令要求，直接启动应用程序镜像，或者先升级应用程序镜像然后启动；最大限度满足『不死』升级的业务要求，同时升级速度让用户不会注意到固件升级过程，这样系统有持续的更新能力和良好的用户体验。



在设计上`BulbBoot`采用最小化原则，不支持蓝牙指令直接控制LED显示，不支持分组，不支持动态播报状态，除遗嘱外不支持动态修改BLE广播内容，不支持监测控制器的（蓝牙）心跳包，也不会读写数据分区。它致力于用最简单、可靠、和快速的方式启动应用。



`Bulbboot`支持如下功能：

1. 广播，广播会自动包含蓝牙地址，广播内仅包含设备老化测试状态（未完成，已经完成，和启动时跳过老化检查）；
2. 扫描蓝牙广播，根据指令启动`ota1`应用，或者wifi连网升级`ota1`应用后再启动；
3. 遗嘱广播，再升级应用过程中发生任何错误设备都会直接重启，重启前有一次错误原因编码的广播；

4. 支持涂鸦产测协议，有少许修改；
5. 支持一个Blink动作，RGB三色循环闪烁几次，方便再包装和库存管理业务中寻找灯泡的业务；



技术上，`BulbBoot`是第二级启动器，位于`ota0`分区，不是esp32系统通常位于`0x8000`地址的bootloader；`BulbBoot`利用esp-idf提供的公开api完成功能，不修改bootloader也不使用hook。0



使用`b0lbca57`（which is a hexspeak for bulbcast）作为每个数据包开头，该magic原则上和应用层的一致。



### bulbboot协议

bulbboot协议只有一条指令；网关使用bulbboot指令启动应用，灯也仅接收这一条指令工作。

| magic 4B    | mac 6B            | sha80 10B         | group id 4B | padding 2B |
| ----------- | ----------------- | ----------------- | ----------- | ---------- |
| b0:1b:b0:07 | 7c:df:a1:61:ec:72 | xx:xx:xx:xx:xx:xx | xx:xx:xx:xx | xx:xx      |



```
b01bb0077cdfa161ec72afc7aa13b6d640e9a437a5a5a5a70208
b01bb0077cdfa161ec72ffc7aa13b6d640e9a437a5a5a5a70208

b01bb0077cdfa161ec728daf63c883e4e9c8cb92a5a5a5a70208

8daf63c883e4e9c8cb92

b01bb0077cdfa161ec72e72f1c921c64a37e0fa4a5a5a5a70208
e72f1c921c64a37e0fa4

b01bb0077cdfa161ec72c4f483bacdd73ad113f2a5a5a5a70208
c4f483bacdd73ad113f2

b01bb0077cdfa161ec727fde9823c1e50312a43fa5a5a5a70208
7fde9823c1e50312a43f

b01bb0077cdfa161ec72622076bb63dc786ddfe7a5a5a5a70208
622076bb63dc786ddfe7

b01bb0077cdfa161ec72751d5643ef42f2eaa7ffa5a5a5a70208
751d5643ef42f2eaa7ff

b01bb00784f7030897be751d5643ef42f2eaa7ffa5a5a5a70207

b01bb00784f70332a5fa751d5643ef42f2eaa7ffa5a5a5a70207

84:f7:03:08:97:be

84f70332a5fa


b01bca57200710a5a5a5a7010007202020202000


b01bca5720081099a5a5a5a700800720F000000000
b01bca5720081099a5a5a5a70080072000F0000000
b01bca5720081099a5a5a5a7008007200000F00000
b01bca5720081099a5a5a5a700800720000000F000


84f70332a5fa


b01bb00784f7030897be751d5643ef42f2eaa7ffa5a5a5a70207

b01bb00784f7030897be751d5643ef42f2eaa7ffa5a5a5a70207

b01bb0077cdfa161ec723bbd2af7918083ab19cfa5a5a5a70207

b01bb00784f70332a5fa1a94b8f1ba48143a63b3a5a5a5a70207
1a94b8f1ba48143a63b3

b01bb00784f70332a5fa3bbd2af7918083ab19cfa5a5a5a70201

b01bca5784f70332a5fa00000000000000000000000000000000

b45a018e8d6bca6ff6b2

b01bb00784f70332a5fab45a018e8d6bca6ff6b2a5a5a5a70

b01bb00784f70332a5fa1a94b8f1ba48143a63b3a5a5a5a70201

b01bca5784f70332a5fa00000000000000000000000000000000

b01bb00784f7030897be1a94b8f1ba48143a63b3a5a5a5a70201


b01bca5720081001a5a5a5a7ffff072ff000000000

b01b00784f7030897be






b01bca5720081011a5a5a5a70fff0720ff00000000

b01bca5720081011a5a5a5a70fff072000ff000000

b01bca5720081012a5a5a5a70fff07200000ff0000



b01bb00784f7030897be00000000000000000000000000000000
b01bb00784f7030897be01000000000000000000000000000000
b01bb00784f7030897be02000000000000000000000000000000
b01bb00784f7030897be03000000000000000000000000000000
b01bb00784f7030897be04000000000000000000000000000000
b01bb00784f7030897be05000000000000000000000000000000
b01bb00784f7030897be06000000000000000000000000000000
b01bb00784f7030897be07000000000000000000000000000000

b01bb00784f7030897be1a94b8f1ba48143a63b3a5a5a5a80201






b01bb00784f70332a5fa1a94b8f1ba48143a63b3a5a5a5a70201
```



0x00 设备信息，Device ID, Program ID, Version mandatory

0x01 分组信息（启动参数）app mandatory

0x02 温度	mandatory

0x04 老化时间 bootloader mandatory



0xE0 遗嘱



灯的payload部分数据格式类似蓝牙的ad element格式设计，采用LTV方式（Length-Type-Value）；但boot指令需装载较多内容无法这样使用。

| code |          | payload                                                      | 完整例子                               |
| ---- | -------- | ------------------------------------------------------------ | -------------------------------------- |
| 0x00 | 灯的广播 | 0x02，0x00，第三个字节是`aged_time_in_minutes`               | b0:1b:ca:57::00::02:00:xx (xx = 00-ff) |
| 0x00 | 灯的遗嘱 | 0x04, 0x01, reason, error, errno                             | b0:1b:ca:57::00::04:01:01:00:00        |
| 0x10 | 灯的广播 | 0x07, 0x02, [boot_params 6 bytes],                           |                                        |
| 0x10 | 灯的遗嘱 | 同前                                                         |                                        |
| 0x20 | 网关指令 | 0x08, 0x10, [seq 1], [group id 4], [bits 2]; 07, 0x20, r, g, b, w, t |                                        |

说明：

1. `aged_time_in_minutes`总计是50分钟设计，如果是0，老化时间不足1分钟，如果是小与等于50，表示老化时间；如果超过50，该灯泡通过人为干预或者固件升级禁止了启动时检查老化测试的ap（`tuya_mdev_test2`），以加快启动速度；
2. 遗嘱数据包里的reason是在程序里定义的enum类型，参见`bulbboot.h`文件里的源码定义，error是optional的，如果错误是函数返回的错误值，由该字段提供，一些函数不提供error值，遗嘱会记录当时的C标准库里的`errno`，比如lwip协议栈的读写错误就使用该宏表示错误原因；乐鑫的平台混合了多种开源项目，每个项目代码对错误返回的约定不一致，所以方便起见遗嘱数据包里包含所有这些值；
3. target mac的endianness；在蓝牙协议里所有数据包内数值定义，例如`uint32_t`都使用little endian，在包头里的源设备地址（以及directed advertisement数据包的目标地址）也是little endian的，这一点可以在linux电脑上用hcidump看到；但是esp32整个平台的地址是从网络地址派生的，比如`esp_read_mac()`函数返回的地址实际上是big endian的，即mac[0]是most significant byte而不是least significan byte，mac[0-2]是乐鑫分配到的OUI（`7c:df:a1`）；为了编程和debug打印时观察方便，本项目所有manufacturer data内包含的蓝牙设备地址，逻辑分组地址，以及打印输出，console输入，均使用big endian方式，乐鑫模块启动后蓝牙协议栈打印的蓝牙地址也是big endian的；
4. ssid token会包含在ap的ssid里，例如如果ssid token是`0x00 0x00 0x01 0x0a`则ap ssid可以是`jubensha-0000010a`；
5. `sha88`是乐鑫的可执行程序的二进制文件包含的镜像的sha256值的前面11个byte，该值包含在编译生成的bin文件的结尾，无需自己计算也不要在传播bin文件时更改；`esp_image_verify()`函数会检查该值确认镜像的完整性；



Reference: https://macaddresschanger.com/what-is-bluetooth-address-BD_ADDR



## 量产测试

量产测试基本按照涂鸦的量产测试要求，简述如下：

1. 第一个测试是老化测试，用路由器`tuya_mdev_test1`触发；初始固件强制老化测试要求，如果nvs（non-volatile storage）内记录的老化时间不足50分钟，则只能进入老化程序，如果找不到`tuya_mdev_test1`，设备成为一个无功能的白灯；
2. 老化测试中的老化时间以分钟为单位，如果老化时出现断电，则再次上电继续，之前的老化时间存储在nvs上；
3. 老化测试包含20分钟冷白最大亮度，20分钟暖白最大亮度，10分钟RGB最大亮度，完成后设备显示低亮度绿灯，同时nvs记录了累计50分钟的老化时间，按照涂鸦产测设计，设备再也不会回到老化测试状态；
4. 老化之后设备启动时仍然要去搜索`tuya_mdev_test2`路由器，如果找到该路由器，设备进入RGB冷白暖白呼吸循环状态，主要是给工厂在老化后观察灯的五色LED仍完好的状态；



设备实现的产测基本上和涂鸦测试要求一致，区别有两点：

1. 涂鸦测试有一个连网授权步骤，本设备测试跳过该步骤；
2. 在设备启动搜索`tuya_mdev_test1`时，如果找到的路由器名称是`skip_tuya_mdev_test1`，则记录老化时间为0xEE，表示该设备人为跳过了老化测试；
3. 在设备启动搜索`tuya_mdev_test2`时，如果找到的路由器名称是`skip_tuya_mdev_test2`，则记录老化时间为0xFF，表示该设备通过了老化测试，但不再会被



## 分区表

rd`



## 蓝牙广播

蓝牙广播在下述状态下存在：

1. 尚未完成老化测试时启动未发现tuya_mdev_test1，进入无功能白灯状态；
2. 已完成老化测试，启动时检查到tuya_mdev_test2，进入五色循环状态；
3. 正常启动状态，包括ota升级时，升级时ble scan关闭但advertisement一直都在；



## 时间参数

`ota`从连接指定的wifi ap开始，系统最终会给一个`STA_GOT_IP`事件表示已经成功通过DHCP获得地址；如果中间遇到错误，程序不仔细检查底层的错误原因，直接重启，设置的超时时间为15秒。



## 调试用程序和数据

```
b0:1b:ca:57::0f::[target mac]::[ssid token]::[sha88]


b0:1b:ca:57::0f::7c:df:a1:61:fa:0a::a5:a5:a5:a5::00:11:22:33:44:55:66:77:88:99:aa
b01bca570f7cdfa161fa0aa5a5a5a500112233445566778899aa

7c:df:a1:61:ec:72

b01bca570f7cdfa161ec72a5a5a5a500112233445566778899aa

ff23ffdc4cbd7d31927f5a0487f7d9d79e401f6f98

b01bca570f7cdfa161ec72a5a5a5a5afc7aa13b6d640e9a4370a

```



# BulbCast

`bulbcast`是应用层使用的传输协议，也是