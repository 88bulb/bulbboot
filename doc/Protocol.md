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



技术上，`BulbBoot`是第二级启动器，位于`ota0`分区，不是esp32系统通常位于`0x8000`地址的bootloader；`BulbBoot`利用esp-idf提供的公开api完成功能，不修改bootloader也不使用hook。



`BulbBoot`使用`ota0`和`ota1`分区命名，是因为esp32平台的设计限制，app分区的subtype只能是factory, test, 和ota_x (x = 0..n)；`BulbBoot`并不使用esp-idf中提供的ota升级（a/b升级）功能，而是使用Deep Sleep方式载入应用（史称Sleep Boot），升级固件时会使用esp-idf提供的partition读写，app镜像完整性检查，和`bootloader_common.h`提供的写入启动镜像地址到rtc mem的功能。



## 协议格式

控制器（网关）和灯泡均只使用BLE Advertisement，仅使用manufacturer data属性，最多有26字节容量，每个命令头部以4字节的magic和1字节的命令码开始。



为了能让manufacturer data的容量可以达到26字节，必须关闭所有可选在蓝牙广播里的所有其它字段，包括name, txpower, slave connection interval range (min/max interval set to 0)，service uuid，service data等参数，例如选择Bluedroid协议栈时adv_data的设置如下。

```c
static const esp_ble_adv_data_t adv_data_default = {
    .set_scan_rsp = false,
    .include_name = false,
    .include_txpower = false,
    .min_interval = 0x0000, // important!
    .max_interval = 0x0000, // important!
    .appearance = 0x00,
    .manufacturer_len = 26,         
    .p_manufacturer_data = &out_mfr_data[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
```



### magic

使用`b0lbca57`作为每个数据包开头，该magic原则上和应用层的一致。



### message

灯的payload部分数据格式类似蓝牙的ad element格式设计，采用LTV方式（Length-Type-Value）；但boot指令需装载较多内容无法这样使用。

| code |          | payload                                          | 完整例子                                             |
| ---- | -------- | ------------------------------------------------ | ---------------------------------------------------- |
| 0x00 | 灯的广播 | 0x02，0x00，第三个字节是`aged_time_in_minutes`   | b0:1b:ca:57::00::02:00:xx (xx = 00-ff)               |
| 0x00 | 灯的遗嘱 | 0x04, 0x01, reason, error, errno                 | b0:1b:ca:57::00::04:01:01:00:00                      |
| 0x0f | 网关指令 | target mac 6字节，ssid token 4字节，sha88 11字节 | b0:1b:ca:57::0f::[target mac]::[ssid token]::[sha88] |

说明：

1. `aged_time_in_minutes`总计是50分钟设计，如果是0，老化时间不足1分钟，如果是小与等于50，表示老化时间；如果超过50，该灯泡通过人为干预或者固件升级禁止了启动时检查老化测试的ap（`tuya_mdev_test2`），以加快启动速度；
2. 遗嘱数据包里的reason是在程序里定义的enum类型，参见`bulbboot.h`文件里的源码定义，error是optional的，如果错误是函数返回的错误值，由该字段提供，一些函数不提供error值，遗嘱会记录当时的C标准库里的`errno`，比如lwip协议栈的读写错误就使用该宏表示错误原因；乐鑫的平台混合了多种开源项目，每个项目代码对错误返回的约定不一致，所以方便起见遗嘱数据包里包含所有这些值；
3. target mac的endianness；在蓝牙协议里所有数据包内数值定义，例如`uint32_t`都使用little endian，在包头（而不是payload）里的源设备地址也是little endian的，这一点可以在linux电脑上用hcidump看到；但是esp32整个平台的地址是从网络地址派生的，比如`esp_read_mac()`函数返回的地址实际上是big endian的，即mac[0]是most significant byte，mac[0-2]是乐鑫分配到的OUI；为了编程和debug打印时观察方便所有manufacturer data内包含的蓝牙设备地址，逻辑组地址，均使用big endian方式，乐鑫的模块启动后蓝牙协议栈打印的蓝牙地址也是big endian的；
4. ssid token会包含在ap的ssid里，例如如果ssid token是`0x00 0x00 0x01 0x0a`则ap ssid可以是`jubensha-0000010a`；
5. `sha88`是乐鑫的可执行程序的二进制文件包含的镜像的sha256值的前面11个byte，该值包含在编译生成的bin文件的结尾，无需自己计算也不要在传播bin文件时更改；`esp_image_verify()`函数会检查该值确认镜像的完整性；



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



## 蓝牙广播

蓝牙广播在下述状态下存在：

1. 尚未完成老化测试时启动未发现tuya_mdev_test1，进入无功能白灯状态；
2. 已完成老化测试，启动时检查到tuya_mdev_test2，进入五色循环状态；
3. 正常启动状态，包括ota升级时，升级时ble scan关闭但advertisement一直都在；



## 其它

`ota`从连接指定的wifi ap开始，系统最终会给一个`STA_GOT_IP`事件表示已经成功通过DHCP获得地址；如果中间遇到错误，程序不仔细检查底层的错误原因，直接重启，设置的超时时间为15秒。
