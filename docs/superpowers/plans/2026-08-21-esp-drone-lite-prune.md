# esp-drone-lite 精简二创固件 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 esp-drone（GPL-3.0）物理裁剪为只保留自稳定飞行的极简固件仓库 `esp-drone-lite`，附 Obsidian 文档库，合规发布到 GitHub。

**Architecture:** 复制上游源码到新 git 仓库 → 分层物理删除（驱动 → 数学库 → 核心模块 → HAL → 平台/Kconfig），每层删完即以 `idf.py build` 编译驱动修复 → 全量验证 → 文档 → 发布。本项目无单元测试框架，**编译通过 + grep 零残留 + 固件体积对比**即为测试。

**Tech Stack:** ESP-IDF release/v5.0（目标 esp32s2）、FreeRTOS、C、CMake、Git、GitHub MCP、Obsidian Markdown。

**Spec:** `docs/superpowers/specs/2026-08-21-esp-drone-lite-design.md`

**通用约定（所有任务适用）：**
- 工作目录 `D:\esp-drone-lite`，shell 为 PowerShell：命令分隔用 `;`，不能用 `&&`
- 每个任务末尾必须 commit；删除文件用 `git rm -r`
- 编译命令（需已激活 ESP-IDF 环境）：`idf.py set-target esp32s2; idf.py build`
- 若本机无 ESP-IDF v5.0 环境：在 Task 1 Step 2 处暂停并询问用户（安装 or 延后编译验证），其余裁剪任务可继续

---

### Task 1: 创建仓库并导入上游快照（基线构建）

**Files:**
- Create: `D:\esp-drone-lite\`（整仓复制自 `D:\esp-drone-master`）
- Create: `D:\esp-drone-lite\docs\superpowers\specs\2026-08-21-esp-drone-lite-design.md`（从原工作区复制）

- [ ] **Step 1: 复制源码并初始化 git**

```powershell
Copy-Item D:\esp-drone-master D:\esp-drone-lite -Recurse -Exclude build,managed_components,sdkconfig
Copy-Item D:\esp-drone-master\docs\superpowers\specs\2026-08-21-esp-drone-lite-design.md D:\esp-drone-lite\docs\superpowers\specs\ -Force
cd D:\esp-drone-lite
git init
git add -A
git commit -m "import: upstream espressif/esp-drone snapshot (GPL-3.0, core from bitcraze/crazyflie-firmware tag_2021_01 b448553)"
```

- [ ] **Step 2: 确认 ESP-IDF 环境**

Run: `idf.py --version`
Expected: 输出 ESP-IDF 5.0.x。若无此命令，暂停询问用户：安装 ESP-IDF v5.0（放 D 盘，勿占 C 盘）或延后编译验证。

- [ ] **Step 3: 基线编译并记录体积**

```powershell
idf.py set-target esp32s2
idf.py build
```
Expected: BUILD SUCCESSFUL。记录 `build\ESPDrone.bin` 与 `build\ESPDrone.map` 大小到 `baseline-size.txt`（裁剪后对比用，最终随 docs 提交）：

```powershell
Get-Item build\ESPDrone.bin, build\ESPDrone.map | Select-Object Name, Length | Out-File baseline-size.txt
```

- [ ] **Step 4: Commit**

```powershell
git add baseline-size.txt docs
git commit -m "docs: add design spec and baseline build size"
```

---

### Task 2: 裁剪驱动组件层

**Files:**
- Delete: `components/drivers/deck/`、`components/drivers/i2c_devices/{hmc5883l,ms5611,vl53l0,vl53l1,eeprom}/`、`components/drivers/spi_devices/`（整个，含 pmw3901）、`components/drivers/general/{adc,buzzer}/`、`components/lib/dsp_lib/`
- Modify: `components/core/crazyflie/CMakeLists.txt:72`（REQUIRES 行）

- [ ] **Step 1: 物理删除**

```powershell
git rm -r components/drivers/deck components/drivers/i2c_devices/hmc5883l components/drivers/i2c_devices/ms5611 components/drivers/i2c_devices/vl53l0 components/drivers/i2c_devices/vl53l1 components/drivers/i2c_devices/eeprom components/drivers/spi_devices components/drivers/general/adc components/drivers/general/buzzer components/lib/dsp_lib
```

- [ ] **Step 2: 更新核心组件 REQUIRES**

将 `components/core/crazyflie/CMakeLists.txt` 第 72 行：

```cmake
                REQUIRES i2c_bus deck mpu6050 ms5611 hmc5883l pmw3901 vl53l1 vl53l0 platform config led eeprom dsp_lib motors wifi adc esp_timer)
```

改为：

```cmake
                REQUIRES i2c_bus mpu6050 platform config led motors wifi esp_timer)
```

- [ ] **Step 3: Commit（此阶段编译预期失败，Task 4/5 修复）**

```powershell
git add -A
git commit -m "prune: drop deck/sensor-extension/eeprom/adc/buzzer/dsp_lib driver components"
```

---

### Task 3: 重写 cf_math.h，彻底解除 dsp_lib 依赖

**Files:**
- Modify: `components/core/crazyflie/utils/interface/cf_math.h`

- [ ] **Step 1: 用以下实现替换 cf_math.h 第 29 行（`#pragma once`）之后的全部内容**（保留第 1-29 行的原版权头，GPL 要求）：

```c
#pragma once

// esp-drone-lite: dsp_lib (xtensa_math) removed.
// Only scalar helpers are kept; matrix ops were used exclusively by the
// removed Kalman estimator. Based on the original cf_math.h from
// espressif/esp-drone (GPL-3.0), modified 2026-08.

#include <math.h>
#include <stdint.h>

#include "cfassert.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define MIN(a, b) ((b) < (a) ? (b) : (a))
#define MAX(a, b) ((b) > (a) ? (b) : (a))

static inline float xtensa_sqrt(float in)
{
    ASSERT(in >= 0.0f);
    return sqrtf(in);
}

static inline float limPos(float in)
{
    if (in < 0.0f) {
        return 0.0f;
    }

    return in;
}

static inline float clip1(float a)
{
    if (a < -1.0f) {
        return -1.0f;
    }

    if (a > 1.0f) {
        return 1.0f;
    }

    return a;
}
```

说明：`mat_trans`/`mat_inv`/`mat_mult`/`assert_aligned_4_bytes` 及 `xtensa_matrix_instance_f32` 全部删除（仅 EKF 使用，已验证）。保留 `xtensa_sqrt` 函数名，避免改动保留模块中的调用方。

- [ ] **Step 2: 验证无残留矩阵 API 引用**

Run: `Get-ChildItem components -Recurse -Include *.c,*.h | Select-String -Pattern "mat_mult|mat_inv|mat_trans|xtensa_matrix" | Select-Object Path, LineNumber`
Expected: 无输出（kalman_core.c 等调用方将在 Task 4 删除；若有其他命中，先删该文件再补记入裁剪记录）。

- [ ] **Step 3: Commit**

```powershell
git add components/core/crazyflie/utils/interface/cf_math.h
git commit -m "prune: rewrite cf_math.h on top of math.h, drop xtensa/dsp matrix API"
```

---

### Task 4: 裁剪核心模块（含 stabilizer/estimator/controller 定点修改）

**Files:**
- Delete: `components/core/crazyflie/modules/src/` 下 26 个文件 + `modules/src/lighthouse/` 整目录 + `modules/interface/` 下对应头文件
- Modify: `components/core/crazyflie/CMakeLists.txt`（SRCS 清单）、`modules/src/stabilizer.c`、`modules/src/system.c`、`modules/src/estimator.c`、`modules/src/controller.c`

- [ ] **Step 1: 删除模块源文件**

```powershell
git rm components/core/crazyflie/modules/src/kalman_core.c components/core/crazyflie/modules/src/estimator_kalman.c components/core/crazyflie/modules/src/kalman_supervisor.c components/core/crazyflie/modules/src/outlierFilter.c components/core/crazyflie/modules/src/controller_indi.c components/core/crazyflie/modules/src/controller_mellinger.c components/core/crazyflie/modules/src/position_controller_indi.c components/core/crazyflie/modules/src/position_controller_pid.c components/core/crazyflie/modules/src/position_estimator_altitude.c components/core/crazyflie/modules/src/crtp_commander_high_level.c components/core/crazyflie/modules/src/pptraj.c components/core/crazyflie/modules/src/pptraj_compressed.c components/core/crazyflie/modules/src/planner.c components/core/crazyflie/modules/src/collision_avoidance.c components/core/crazyflie/modules/src/sitaw.c components/core/crazyflie/modules/src/msp.c components/core/crazyflie/modules/src/mem.c components/core/crazyflie/modules/src/sound_cf2.c components/core/crazyflie/modules/src/extrx.c components/core/crazyflie/modules/src/info.c components/core/crazyflie/modules/src/trigger.c components/core/crazyflie/modules/src/range.c components/core/crazyflie/modules/src/tdoaEngineInstance.c components/core/crazyflie/modules/src/peer_localization.c components/core/crazyflie/modules/src/crtp_localization_service.c
git rm -r components/core/crazyflie/modules/src/lighthouse
```

并删除 `modules/interface/` 下同名头文件（用同一命令格式追加：`kalman_core.h kalman_supervisor.h outlierFilter.h controller_indi.h controller_mellinger.h position_controller.h position_estimator_altitude.h crtp_commander_high_level.h pptraj.h pptraj_compressed.h planner.h collision_avoidance.h sitaw.h msp.h mem.h sound.h extrx.h trigger.h range.h lighthouse/ tdoa* peer_localization.h crtp_localization_service.h`，以 `dir modules/interface` 实际清单为准；`power_distribution.h`、`controller.h`、`estimator.h` 等保留模块的头文件**不要删**）。

- [ ] **Step 2: 同步 CMakeLists SRCS 清单**

从 `components/core/crazyflie/CMakeLists.txt` 的 SRCS 列表中删除上述已删文件对应的行（第 16-17、21、27、29-30、33-34、39-41、43-44、48-49 行附近：`controller_indi.c`、`controller_mellinger.c`、`crtp_commander_high_level.c`、`estimator_kalman.c`、`kalman_core.c`、`kalman_supervisor.c`、`msp.c`、`outlierFilter.c`、`position_controller_indi.c`、`position_controller_pid.c`、`position_estimator_altitude.c`、`pptraj_compressed.c`、`pptraj.c`、`sitaw.c`、`sound_cf2.c`、`trigger.c`、`range.c`），同时删除 SRCS 中的 `./hal/src/buzzer.c`、`./hal/src/espnow_ctrl.c`（HAL 文件在 Task 5 物理删除）。保留 `./modules/src/range.c` 之外的所有其他行不动。

- [ ] **Step 3: stabilizer.c 定点修改**

`components/core/crazyflie/modules/src/stabilizer.c`：

1. 删除第 43-44 行的两个 include：
```c
#include "crtp_localization_service.h"
#include "sitaw.h"
```
2. `stabilizerInit()`（约 189-209 行）中删除：
```c
  if (estimator == anyEstimator) {
    estimator = deckGetRequiredEstimator();
  }
```
与
```c
  sitAwInit();
```
3. `stabilizerTask()` 主循环中删除：
```c
      sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
```

- [ ] **Step 4: system.c 定点修改**

`components/core/crazyflie/modules/src/system.c`：

1. 删除 include：`"adc_esp32.h"`、`"mem.h"`、`"buzzer.h"`、`"sound.h"`、`"estimator_kalman.h"`（第 40、58、62-63、65 行）
2. `systemInit()` 中删除：`adcInit();`、`buzzerInit();`
3. `systemTest()` 中删除：`pass &= buzzerTest();`
4. `systemTask()` 中删除：`estimatorKalmanTaskInit();`、`soundInit();`、`memInit();`、`pass &= estimatorKalmanTaskTest();`（及其 DEBUG_PRINTI 行）、`pass &= soundTest();`（及 DEBUG_PRINTI）、`pass &= memTest();`（及 DEBUG_PRINTI）、`soundSetEffect(SND_STARTUP);`

- [ ] **Step 5: estimator.c 收敛为仅互补滤波**

`components/core/crazyflie/modules/src/estimator.c`：删除所有 `estimatorKalman*` 相关 include、函数指针表项、case 分支与 `estimatorKalmanTaskInit/Test` 引用，仅保留 `estimatorComplementary` 分支。`stateEstimatorSwitchTo` 中对 kalman 的分支一并删除（编译报错会精确指出位置）。

- [ ] **Step 6: controller.c 收敛为仅 PID**

`components/core/crazyflie/modules/src/controller.c`：删除 `controllerINDI*`、`controllerMellinger*` 的 include、函数指针表项与 case 分支，仅保留 `controllerPID`。`getControllerType`/`controllerInit` 中非法枚举值直接 ASSERT。

- [ ] **Step 7: 编译修复循环**

```powershell
idf.py build
```
逐个消除编译错误。预期高频报错及处理：
- `xxx.h: No such file` → 找到 include 方，删除该 include 与对应调用（调用方若属保留模块，做最小化处理而非恢复被删文件）
- `configblockeeprom.c` 因缺 eeprom 驱动报错 → 改为 stub：`configblockInit()` 直接返回成功、`configblockTest()` 返回 true、校准读取函数返回默认值（不做持久化）
- `crtp.c`/`crtpservice.c` 若引用 `mem.h`/`msp.h` → 删除对应 CRTP 端口注册（mem/mso 服务随模块删除）
- `log.c`/`param.c` 自身不引用被删模块，无需改动；若报错按上条处理

- [ ] **Step 8: Commit**

```powershell
git add -A
git commit -m "prune: keep only attitude-stabilize flight path (complementary filter + PID + WiFi CRTP)"
```

---

### Task 5: HAL 层裁剪（sensors 精简 + espnow/buzzer 移除 + pm 去 ADC）

**Files:**
- Delete: `components/core/crazyflie/hal/src/{amg8833.c,buzzer.c,espnow_ctrl.c,ow_common.c,ow_none.c,ow_syslink.c,pca9555.c,pca95x4.c,pm_stm32f4.c,proximity.c,radiolink.c,sensors_bmi088_bmp388.c,sensors_bmi088_spi_bmp388.c,sensors_bosch.c,sensors_mpu9250_lps25h.c,storage.c,syslink.c,usb.c,usbd_desc.c,usblink.c,usb_bsp.c}`
- Modify: `components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c`（改名+精简）、`pm_esplane.c`、`components/core/crazyflie/hal/interface/` 对应头文件

- [ ] **Step 1: 删除未编译的 HAL 遗留文件**

```powershell
git rm components/core/crazyflie/hal/src/amg8833.c components/core/crazyflie/hal/src/buzzer.c components/core/crazyflie/hal/src/espnow_ctrl.c components/core/crazyflie/hal/src/ow_common.c components/core/crazyflie/hal/src/ow_none.c components/core/crazyflie/hal/src/ow_syslink.c components/core/crazyflie/hal/src/pca9555.c components/core/crazyflie/hal/src/pca95x4.c components/core/crazyflie/hal/src/pm_stm32f4.c components/core/crazyflie/hal/src/proximity.c components/core/crazyflie/hal/src/radiolink.c components/core/crazyflie/hal/src/sensors_bmi088_bmp388.c components/core/crazyflie/hal/src/sensors_bmi088_spi_bmp388.c components/core/crazyflie/hal/src/sensors_bosch.c components/core/crazyflie/hal/src/sensors_mpu9250_lps25h.c components/core/crazyflie/hal/src/storage.c components/core/crazyflie/hal/src/syslink.c components/core/crazyflie/hal/src/usb.c components/core/crazyflie/hal/src/usbd_desc.c components/core/crazyflie/hal/src/usblink.c components/core/crazyflie/hal/src/usb_bsp.c
```

同步删除 `hal/interface/` 下对应头文件（`buzzer.h`、`espnow_ctrl.h`、`pm_stm32f4.h`、`proximity.h`、`radiolink.h`、`syslink.h`、`usb*.h`、`ow*.h`、`storage.h` 等；`sensors.h`、`pm_esplane.h`、`wifilink.h`、`ledseq.h`、`usec_time.h`、`freeRTOSdebug.h` 保留）。

- [ ] **Step 2: sensors 文件精简**

```powershell
git mv components/core/crazyflie/hal/src/sensors_mpu6050_hm5883L_ms5611.c components/core/crazyflie/hal/src/sensors_mpu6050.c
```

编辑 `sensors_mpu6050.c`：
1. 删除 `hmc5883l.h`、`ms5611.h` include 及所有磁罗盘/气压计的初始化、读取、任务代码块（按编译错误定位）
2. `sensorData->mag` 填充改为全 0；`baro` 填充改为常量默认值（`asl=0, temperature=25, pressure=1013.25`），保证保留的 LOG 组（`baro`/`mag`）仍有合法数据源
3. 文件头注释追加一行：`Modified for esp-drone-lite: HMC5883L/MS5611 support removed (2026-08).`（GPL §5a 修改标注）
4. 更新 `components/core/crazyflie/CMakeLists.txt` 第 5 行 SRCS 路径为新文件名 `./hal/src/sensors_mpu6050.c`

- [ ] **Step 3: pm_esplane.c 去 ADC**

删除其中对 `adc_esp32.h`/`adcInit`/`adcDmaTimer` 等的引用；`pmGetBatteryVoltage()` 等被 `stabilizer.c`（桨测试）使用的函数改为返回固定标称值（如 `3.7f`）并加注释说明精简原因。`pmInit/pmTest` 保持可调用。同样追加修改标注行。

- [ ] **Step 4: 编译修复循环**

```powershell
idf.py build
```
Expected: BUILD SUCCESSFUL（HAL 层自此闭环）。逐个修复残留错误后提交。

- [ ] **Step 5: Commit**

```powershell
git add -A
git commit -m "prune: HAL slimmed to mpu6050-only sensors, wifi link, minimal pm"
```

---

### Task 6: platform 层与 Kconfig/sdkconfig 精简

**Files:**
- Modify: `components/platform/{platform.c,platform_cf2.c,platform_esp32.c}`、`main/Kconfig.projbuild`、`sdkconfig.defaults`、`CMakeLists.txt`
- Delete: `components/platform/stm32_legacy.h`（若无引用）、`sdkconfig.defaults.esp32`、`sdkconfig.defaults.esp32s3`

- [ ] **Step 1: platform 收敛到 S2 单机型**

1. `platform_cf2.c`/`platform_esp32.c`：删除 `TARGET_ESPLANE_V1`、`TARGET_ESPLANE_V2_S2` 的设备配置条目，仅保留 `ESP32_S2_DRONE_V1_2` 条目
2. `platform.c`：删除对上述已删机型的分发分支
3. 检查 `stm32_legacy.h` 引用：`Get-ChildItem components -Recurse -Include *.c,*.h | Select-String "stm32_legacy"`；若只剩 `stabilizer.c`/`system.c` 的 include 且其宏（如 `NO_DMA_CCM_SAFE_ZERO_INIT`）可替换为空宏，则在本文件保留一个 10 行以内的最小兼容头（注明精简来源），否则删除

- [ ] **Step 2: Kconfig 精简（main/Kconfig.projbuild）**

1. 硬件版本 choice（第 3-18 行）：删除 `TARGET_ESPLANE_V1`、`TARGET_ESPLANE_V2_S2` 选项及所有配置项中对应的 `range/default ... if TARGET_ESPLANE_V1/V2_S2` 行，仅保留 `TARGET_ESP32_S2_DRONE_V1_2`
2. 删除整个 `buzzer` menu（第 105-133 行）
3. 删除 `SPI_PIN_MISO/MOSI/CLK/CS0`（第 180-223 行）、`ADC1_PIN`（第 235-244 行）、`EXT01_PIN`（第 246-255 行）配置
4. `app set` menu：`ENABLE_POSITION_HOLD_MODE` 与 `ENABLE_COMMAND_MODE_SET` 删除（光流已删）；`ENABLE_LEGACY_APP` 保留
5. `extend board version` choice 保留（deck 虽删，`EXT_AUTO_DETECT` 若被代码引用则先 grep 确认：无引用则整段删除）

- [ ] **Step 3: sdkconfig 与顶层 CMake**

1. 删除 `sdkconfig.defaults.esp32`、`sdkconfig.defaults.esp32s3`（`git rm`）
2. `sdkconfig.defaults`：删除 `CONFIG_FREERTOS_UNICORE=n`（与 esp32s2 的单核默认冲突，交给 `sdkconfig.defaults.esp32s2`）
3. 顶层 `CMakeLists.txt`：`PLANE_COMPONENT_DIRS` 删除 `"./components/drivers/spi_devices"` 与 `"./components/lib"`（spi_devices 已删、dsp_lib 已删，lib 目录空）
4. `project(ESPDrone)` 改为 `project(ESPDroneLite)`

- [ ] **Step 4: 编译验证**

```powershell
Remove-Item sdkconfig -ErrorAction SilentlyContinue
idf.py set-target esp32s2
idf.py build
```
Expected: BUILD SUCCESSFUL。

- [ ] **Step 5: Commit**

```powershell
git add -A
git commit -m "prune: single-target ESP32-S2 platform, simplified Kconfig/sdkconfig"
```

---

### Task 7: 全量验证（零残留 + 体积对比）

- [ ] **Step 1: 残留引用扫描**

```powershell
Get-ChildItem components,main -Recurse -Include *.c,*.h,*.txt | Select-String -Pattern "kalman|lighthouse|tdoa|pmw3901|vl53l|hmc5883|ms5611|sitaw|indi|mellinger|pptraj"
```
Expected: 仅命中 `docs/`、注释或裁剪记录；`components/`、`main/` 源码中无功能性命中。有命中则回到对应任务修复。

- [ ] **Step 2: 体积对比**

```powershell
Get-Item build\ESPDroneLite.bin, build\ESPDroneLite.map | Select-Object Name, Length | Out-File pruned-size.txt
Get-Content baseline-size.txt, pruned-size.txt
```
Expected: bin 明显小于基线（预期缩减 ≥30%）。将结果记录进 `pruned-size.txt` 头部注释。

- [ ] **Step 3: Commit**

```powershell
git add baseline-size.txt pruned-size.txt
git commit -m "verify: record firmware size reduction vs upstream baseline"
```

---

### Task 8: Obsidian 文档库 + README（使用 obsidian-markdown 技能）

**前置：** 调用 Skill 工具加载 `obsidian-markdown` 技能，按其规范写 frontmatter、wikilink、mermaid。

**Files:**
- Create: `docs/obsidian/` 下 7 个笔记 + 根 `README.md`

- [ ] **Step 1: 建 vault 结构与索引**

创建 `docs/obsidian/00-总览.md`（frontmatter: `tags: [esp-drone-lite, 索引]`），正文用 wikilink 串联其余 6 篇笔记，一句话说明"用 Obsidian 打开本目录即可"。

- [ ] **Step 2: 硬件与接线.md**

必含引脚表（`TARGET_ESP32_S2_DRONE_V1_2` Kconfig 默认值）：

| 信号 | GPIO | 接法 |
|---|---|---|
| I2C0 SDA | IO11 | MPU6050 SDA |
| I2C0 SCL | IO10 | MPU6050 SCL |
| MPU INT | IO12 | MPU6050 INT |
| M1~M4 | IO5/IO6/IO3/IO4 | 各经 N-MOS（SI2302）驱动 720 空心杯，漏极接电机+续流二极管 |
| LED 红/绿/蓝 | IO8/IO9/IO7 | 可选 |

附最小 BOM（开发板、MPU6050 模块、4×SI2302、4×1N5819、4×720 电机、4×10kΩ 栅极电阻、电池）与文字版电路说明。

- [ ] **Step 3: 焊接步骤.md**

按焊点数从少到多排序（MPU6050 4 根飞线 → 栅极电阻 → MOS → 电机线），每步含检查点（如"焊完 I2C 先用万用表量通断"）。

- [ ] **Step 4: 编译与烧录.md**

ESP-IDF v5.0 安装（建议装 D 盘）、`idf.py set-target esp32s2`、`idf.py build`、`idf.py -p COMx flash monitor`，WiFi 连接说明（SSID = `CONFIG_WIFI_BASE_SSID`-MAC，密码 `12345678`，CRTP UDP 端口 2390，cfclient 适配版见上游 README 链接）。

- [ ] **Step 5: 架构说明.md**

mermaid 数据流图（MPU6050 → sensors → sensfusion6 → stabilizer → attitude_pid → controller_pid → power_distribution → motors；WiFi CRTP → commander），与 spec 第 5 节一致。

- [ ] **Step 6: 裁剪记录.md**

从本计划 Task 2/3/4/5/6 的删除清单生成；含 `baseline-size.txt`/`pruned-size.txt` 对比数据。

- [ ] **Step 7: 协议声明.md + 根 README**

1. `协议声明.md`：GPL-3.0 义务清单、上游归属（espressif/esp-drone、bitcraze/crazyflie-firmware tag_2021_01 b448553）、修改日期 2026-08
2. 根 `README.md` 顶部显著声明（中英双语一句话）："本项目基于 espressif/esp-drone（GPL-3.0）修改，2026-08 精简为仅自稳定飞行版本"，随后是功能简介、指向 `docs/obsidian/` 的说明、第三方归属表（注明 dsp_lib 等已裁剪）

- [ ] **Step 8: Obsidian 打开验证 + Commit**

用 Obsidian 打开 `docs/obsidian` 确认无失效链接（或用 obsidian-cli 检查）；然后：

```powershell
git add README.md docs
git commit -m "docs: add Obsidian vault (wiring/soldering/build/architecture) and GPL attribution README"
```

---

### Task 9: 发布到 GitHub（GitHub MCP）

- [ ] **Step 1: 确认账号与仓库名**

询问用户 GitHub 用户名（或读取已登录态）与期望仓库名（默认 `esp-drone-lite`，public）。

- [ ] **Step 2: 创建仓库**

CallMcpTool `github/create_repository`：name=`esp-drone-lite`，description="Minimal attitude-stabilize fork of espressif/esp-drone (GPL-3.0) for ESP32-S2 + hand-wired MPU6050"，private=false。

- [ ] **Step 3: 推送**

```powershell
git remote add origin https://github.com/<用户名>/esp-drone-lite.git
git branch -M main
git push -u origin main
```

- [ ] **Step 4: 核验合规三要素**

CallMcpTool `github/get_file_contents` 抽查远端：`LICENSE` 存在（GPL-3.0 原文）、`README.md` 含修改声明与上游链接。全部通过即任务完成，输出最终汇报（含体积对比数字）。

---

## Self-Review 记录（写计划者自查）

1. **Spec 覆盖**：spec 第 3 节仓库基础 → Task 1/6；第 4.1 节驱动裁剪 → Task 2；第 4.2 节模块裁剪 → Task 4；第 4.3 节 HAL → Task 5；第 4.4 节 cf_math/dsp → Task 3（configblockeeprom stub 未单列：Task 4 编译循环中处理，eeprom 驱动删除后该文件报错即改 stub 返回默认值——已确认需补充）；第 4.5 节 platform → Task 6；第 5 节数据流 → Task 4-5 保留项；第 6 节合规 → Task 8-9；第 7 节 Obsidian → Task 8；第 8 节验证 → Task 7/9。
2. **占位符扫描**：无 TBD；"以 dir 实际清单为准"类措辞均附带了具体执行命令。
3. **类型一致性**：文件名、Kconfig 符号（TARGET_ESP32_S2_DRONE_V1_2）、任务间引用一致。
4. **修正**：configblockeeprom stub 补入 Task 4 Step 7 预期报错清单（执行时 `configblockeeprom.c` 因缺 eeprom 驱动报错 → 改为返回默认值的 stub）。
