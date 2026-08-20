# esp-drone-lite 精简二创固件 设计文档

日期：2026-08-21
状态：已确认（用户批准方案 A）

## 1. 概述

基于乐鑫开源项目 [espressif/esp-drone](https://github.com/espressif/esp-drone)（GPL-3.0，核心源自 Bitcraze Crazyflie 固件 tag_2021_01）进行二创：通过**物理裁剪**得到只保留「自稳定飞行」的最小固件，运行于 ESP32-S2 开发板 + 手焊 MPU6050 的最小硬件系统，并附带可直接用 Obsidian 打开的 Markdown 文档库。最终发布到用户自己的 GitHub 仓库，严格遵守 GPL-3.0。

## 2. 目标与非目标

**目标**
- 只保留自稳定模式：姿态 PID + 互补滤波（sensfusion6），WiFi CRTP 控制
- 大量删减代码（删除 EKF、定高/定点、光流/TOF、磁罗盘等约 200KB 源码及 8 个驱动组件）
- 硬件焊接最小化：ESP32-S2 DevKit + MPU6050 模块飞线 + 4 路 MOSFET 电机驱动，无定制 PCB
- 仓库内 `docs/obsidian/` 为纯 Markdown 文档库（wikilink + frontmatter），Obsidian 直接打开
- GPL-3.0 合规：保留 LICENSE 与版权头、显著标注修改、归属上游

**非目标（YAGNI）**
- 不支持定高、定点、光流、室内定位等任何高级模式
- 不支持 ESP32 / ESP32-S3（仅 S2 单目标）
- 不重新设计硬件 PCB
- 不构建文档站点（VitePress 等），Obsidian 原样浏览即可

## 3. 仓库基础

- 路径：`D:\esp-drone-lite`（D 盘）
- 全新 `git init`，提交历史：
  1. `import:` 上游 esp-drone 原始快照（提交信息注明上游仓库地址与来源 commit/tag）
  2. `prune:` 物理删除 + CMakeLists / Kconfig / sdkconfig 裁剪
  3. `docs:` Obsidian 文档库、LICENSE 声明、README
- 平台目标：`TARGET_ESP32_S2_DRONE_V1_2`（沿用官方引脚定义，方便对照官方原理图；用户可通过 menuconfig 改引脚适配自己的开发板走线）
- 电机默认 `MOTOR_BRUSHED_720`
- sdkconfig 精简：删除 ESP32/S3 相关默认项，S2 单目标（`sdkconfig.defaults.esp32s2` 保留 240MHz 单核配置）

## 4. 裁剪清单

### 4.1 删除的驱动组件（components/drivers）
| 组件 | 原用途 |
|---|---|
| `deck` | 扩展板总线框架 |
| `i2c_devices/hmc5883l` | 磁罗盘 |
| `i2c_devices/ms5611` | 气压计（定高） |
| `i2c_devices/vl53l0`、`vl53l1` | 激光测距（定点） |
| `spi_devices/pmw3901` | 光流传感器（定点） |
| `i2c_devices/eeprom` | 配置存储（见 4.4 stub） |
| `general/adc` | 电池电压检测 |
| `general/buzzer` | 蜂鸣器 |

保留：`i2c_bus`、`i2c_devices/mpu6050`、`general/led`、`general/motors`、`general/wifi`。

### 4.2 删除的核心模块（components/core/crazyflie/modules/src）
- **EKF 系**：`kalman_core.c`、`estimator_kalman.c`、`kalman_supervisor.c`、`outlierFilter.c`
- **高级控制器**：`controller_indi.c`、`controller_mellinger.c`、`position_controller_indi.c`、`position_controller_pid.c`、`position_estimator_altitude.c`
- **高级指令/轨迹**：`crtp_commander_high_level.c`、`pptraj.c`、`pptraj_compressed.c`、`planner.c`
- **定位系**：`lighthouse/`（整目录）、`tdoaEngineInstance.c`、`peer_localization.c`、`crtp_localization_service.c`
- **其他**：`collision_avoidance.c`、`sitaw.c`、`msp.c`、`mem.c`、`sound_cf2.c`、`extrx.c`、`info.c`、`trigger.c`、`range.c`（已验证：trigger 仅被 sitaw 引用，range.h 无其他引用方；sitaw 被 `stabilizer.c` include，删除时需同步清理 stabilizer.c 中的 sitaw 引用）

保留：`stabilizer.c`、`attitude_pid_controller.c`、`controller_pid.c`、`controller.c`、`sensfusion6.c`、`estimator_complementary.c`、`estimator.c`、`power_distribution_stock.c`、`pid.c`、`commander.c`、`crtp.c`、`crtpservice.c`、`crtp_commander.c`、`crtp_commander_rpyt.c`、`crtp_commander_generic.c`、`comm.c`、`console.c`、`log.c`、`param.c`、`platformservice.c`、`system.c`、`worker.c`、`queuemonitor.c`、`sysload.c`、`static_mem.c`、`app_handler.c`、`app_channel.c`。

### 4.3 HAL 层改动（components/core/crazyflie/hal）
- `sensors_mpu6050_hm5883L_ms5611.c` → 改写为 `sensors_mpu6050.c`：仅初始化/读取 MPU6050，删除 HMC5883L/MS5611 分支
- 删除 `espnow_ctrl.c`（ESP-NOW 控制链路，仅保留 WiFi CRTP）
- `pm_esplane.c` 简化：去除 ADC 电池读取依赖，保留最小电源管理框架（或降级为 stub）
- 保留：`wifilink.c`、`sensors.c`、`ledseq.c`、`usec_time.c`、`freeRTOSdebug.c`

### 4.4 utils 与依赖 stub
- `configblockeeprom.c`：eeprom 驱动删除后改为 stub，直接返回默认校准/配置值（不做持久化）
- `dsp_lib` **整体删除**（已验证）：`xtensa_*` 数学函数的实际调用方仅 `kalman_core.c`（将被删除）；保留的 `commander.c` 只 include `cf_math.h` 而未调用 dsp 函数。配套改动：将 `utils/interface/cf_math.h` 中基于 dsp_lib 的包装（`xtensa_sqrt` 等）改写为标准 `math.h` 实现（`sqrtf`/`fmaxf` 等）
- 其余 utils 体积小，全部保留

### 4.5 platform 层
- `platform_esp32.c` / `platform_cf2.c`：删除 ESPlane V1、STM32 legacy 等非 S2 分支，保留 `TARGET_ESP32_S2_DRONE_V1_2`
- `stm32_legacy.h`：若裁剪后无引用则删除

## 5. 保留的最小数据流

```
MPU6050 (I2C0, CONFIG_I2C0_PIN_SDA/SCL)
   ↓ sensors task (1000Hz)
sensfusion6 互补滤波 → 姿态角 (roll/pitch/yaw)
   ↓ stabilizer task
attitude_pid_controller → controller_pid
   ↓
power_distribution_stock → motors (4 路 PWM, 720 空心杯)

控制输入: WiFi UDP CRTP (wifilink)
   → crtp_commander_rpyt / crtp_commander_generic → commander → setpoint
调参/观察: param + log 模块（cfclient 兼容）
```

硬件接线（写入 Obsidian 文档，引脚以 Kconfig 默认值为准）：ESP32-S2 DevKit 的 I2C0 引脚接 MPU6050（VCC/GND/SDA/SCL，模块带上拉则免外加），4 路 GPIO → SI2302 类 N-MOS + 续流二极管 → 716/720 空心杯电机。

## 6. GPL-3.0 合规（发布到 GitHub）

1. `LICENSE` 保留 GPL-3.0 原文；保留源文件原始版权头注释
2. README 顶部显著声明（GPL §5a/§5b）：修改自 espressif/esp-drone（GPL-3.0），标注修改日期（2026-08）与修改内容摘要，链接上游仓库与 Crazyflie 来源
3. 保留/更新第三方代码归属表（Crazyflie tag_2021_01 b448553；dsp_lib 已删除，归属表中注明已裁剪）
4. 二创整体仍以 GPL-3.0 发布；不使用 Espressif / Bitcraze 商标做背书宣传
5. 使用 GitHub MCP 创建仓库、推送、附 LICENSE

## 7. Obsidian 文档库（docs/obsidian/）

纯 Markdown + wikilink + YAML frontmatter，Obsidian 将该目录作为 vault 直接打开：

| 文件 | 内容 |
|---|---|
| `00-总览.md` | 索引页，wikilink 串联全部笔记 |
| `硬件与接线.md` | 引脚表、电机驱动电路、BOM（最少焊点原则） |
| `焊接步骤.md` | 新手向、按焊点数量排序的焊接顺序 |
| `编译与烧录.md` | ESP-IDF release/v5.0 环境搭建、build/flash 命令 |
| `架构说明.md` | 裁剪后数据流（mermaid 图，Obsidian 原生渲染） |
| `裁剪记录.md` | 删除清单、理由、与上游差异摘要 |
| `协议声明.md` | GPL-3.0 义务清单、上游归属 |

## 8. 验证标准

1. `idf.py build`（ESP-IDF release/v5.0，ESP32-S2）编译通过，0 error
2. 全仓 grep 无对已删模块/驱动的残留引用
3. 记录裁剪前后固件大小（.bin / map 文件）对比，确认"大量降低"达成
4. 编译证据齐全后才推送 GitHub（遵循 verification-before-completion）
5. （可选，视用户是否有硬件）IMU 读数、电机测试、自稳定联调

## 9. 风险与对策

| 风险 | 对策 |
|---|---|
| `stabilizer.c` 等文件内部条件编译引用已删模块 | 逐一理顺 `#ifdef`/引用，必要时保留最小 stub |
| `log.c`/`param.c` 依赖被删模块注册的变量 | 清理对应注册表条目，保证编译与运行期不崩 |
| 本机缺少 ESP-IDF v5.0 环境 | 实施阶段先确认环境；缺失则先完成代码裁剪，编译验证后置 |

## 10. 工具链

- superpowers 技能流：brainstorming（本文档）→ writing-plans → 实施（TDD/debugging/verification 按需）
- obsidian-markdown 技能：生成 Obsidian 规范文档
- GitHub MCP：核对上游信息、创建仓库、推送
- Postman MCP：与本任务（嵌入式固件）无直接关联，不使用；如后续涉及 API 测试再启用
