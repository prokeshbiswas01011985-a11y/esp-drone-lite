# ESP-Drone Lite

> **中文**：本项目基于 [espressif/esp-drone](https://github.com/espressif/esp-drone)（GPL-3.0）修改，2026-08 精简为仅自稳定飞行版本。
>
> **English**: This project is a derivative of [espressif/esp-drone](https://github.com/espressif/esp-drone) (GPL-3.0), pruned in 2026-08 to an attitude-stabilization-only firmware.

## 这是什么

ESP-Drone Lite 是一份**极简四轴自稳定固件**：

- 芯片：ESP32-S2
- 传感器：仅 MPU6050（加速度计 + 陀螺仪）
- 控制：互补滤波（sensfusion6）+ 姿态/角速率双环 PID + 电机混控
- 链路：WiFi AP + UDP CRTP（端口 2390），兼容 cfclient / ESP-Drone App

相比上游，删除了 EKF/Kalman、INDI/Mellinger 控制器、定高/定点、光流/ToF/磁罗盘/气压计、deck 扩展板框架、dsp_lib 等全部非必需模块。完整删除清单见 [`docs/obsidian/05-裁剪记录.md`](docs/obsidian/05-裁剪记录.md)。

## 文档（Obsidian 文档库）

用 [Obsidian](https://obsidian.md) 打开 [`docs/obsidian`](docs/obsidian) 目录即可浏览，入口为 `00-总览.md`：

| 笔记 | 内容 |
|---|---|
| 01-硬件与接线 | 引脚表、最小 BOM、电机驱动电路 |
| 02-焊接步骤 | 分步焊接流程与检查点 |
| 03-编译与烧录 | ESP-IDF v5.0 环境、编译、flash、WiFi 遥控 |
| 04-架构说明 | 数据流 mermaid 图与任务结构 |
| 05-裁剪记录 | 删除清单与体积对比 |
| 06-协议声明 | GPL-3.0 义务与上游归属 |

## 快速开始

```powershell
# ESP-IDF v5.0 环境激活后
idf.py set-target esp32s2
idf.py build
idf.py -p COMx flash monitor
```

详见 [`docs/obsidian/03-编译与烧录.md`](docs/obsidian/03-编译与烧录.md)。

## 第三方归属

| 来源 | 协议 | 说明 |
|---|---|---|
| [espressif/esp-drone](https://github.com/espressif/esp-drone) | GPL-3.0 | 直接上游（ESP32 移植、WiFi CRTP） |
| [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) `tag_2021_01` | GPL-3.0 | 核心飞控代码来源 |
| dsp_lib（CMSIS-DSP 移植） | — | **已完整删除**，不再分发 |

本项目按 GPL-3.0 分发，许可证见 [LICENSE](LICENSE)。所有修改过的源文件均保留原始版权头并标注修改说明（GPL-3.0 §5(a)）。
