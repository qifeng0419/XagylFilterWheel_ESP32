# Xagyl Filter Wheel — ESP32 + TMC2209 + MT6701

天文滤镜轮固件。用 ESP32 直接驱动 TMC2209 + 步进电机，以 MT6701 磁编码器做绝对位置反馈，
对外完整实现 **Xagyl 滤镜轮串口协议**，可被 `rpineau/XagylFilterWheel` 那套驱动
（以及基于它的 ASCOM / INDI 滤镜轮驱动）直接识别和控制。

**核心特性：初始位置完全可调** —— 槽位 1 的参考角、槽间距、每个槽位的独立偏移、
上电定位策略，四者全部可通过串口命令或调试命令行现场修改并掉电保存。

---

## 1. 它解决什么问题

天文相机的滤镜轮通常需要一个驱动软件对接硬件。SBIG 滤镜轮定义了一套极简串口协议，
`rpineau/XagylFilterWheel` 是它的开源主机侧驱动。社区里流传着两类 Arduino 设备端固件
（Smikkelsen / Chemistorge），它们**照抄了协议的命令字母表，但没照抄应答格式**，
接到 Xagyl 驱动上会在握手阶段就失败。

本工程是从驱动源码 `xagyl.cpp` 的 `sscanf` 模板**反向**实现的设备端固件，
每一条应答都对齐驱动的真实解析逻辑，并修正了原固件的几处硬伤：

| 问题 | 原 SBIG 系固件 | 本工程 |
|---|---|---|
| `I4` 转速格式 | `Max Speed 400`（有空格）→ 驱动解析失败、握手中断 | `MaxSpeed 100`，严格匹配模板 |
| `T0` 传感器 | 只回单个数字 → 驱动 `Sensors %d %d` 解析失败 | 回双值 `Sensors <raw14> <field>` |
| `S` 调速参数 | `isDigit()` 校验直接拒收 `SA`..`SF` | 按十六进制解析，支持 16 档 |
| `R6` 语义 | 保存 EEPROM，且**完全不应答** → 驱动读超时 | 启动逐槽自动校准，回 `Calibrating`，结束推送 `Done` |
| `R0`/`R1` | 只回字符串 `Reset`，实际不复位 | 真实复位（先存配置再 `ESP.restart()`） |
| 位置回报 | 走位失败仍回目标位号（谎报到位） | **一律回报编码器实测反推的位置**，失败如实反映 |
| 校准能力 | `(` `)` `O` `I6` `R6` 全是桩，永远返回 Offset 0 | 真实 ±1 单位偏移 + 半间距限幅 + NVS 掉电保存 |

完整协议契约见 [docs/PROTOCOL.md](docs/PROTOCOL.md)。

---

## 2. 硬件

| 器件 | 作用 | 说明 |
|---|---|---|
| ESP32 (WROOM-32 / DEVKIT) | 主控 | 需要 2 路硬件 UART（协议口 + TMC 口），ESP32-S3/C3 也可，注意 UART 数量 |
| TMC2209 | 步进电机驱动 | 静音 StealthChop，UART 可读写电流/细分/堵转 |
| 2 相步进电机 | 执行机构 | 默认按 200 整步（1.8°）配置，0.9° 电机改 `FW_MOTOR_STEPS_PER_REV` |
| MT6701 | 14 位绝对磁角度传感器 | 单圈绝对值、掉电不丢位置；I²C 或 SSI/SPI 任选 |
| 径向充磁磁铁 | 与 MT6701 配套 | 装在滤镜轮轴上，气隙按手册范围 |

### 默认引脚（全部可在 `Config.h` 改）

| 功能 | GPIO | 备注 |
|---|---|---|
| TMC2209 STEP | 26 | |
| TMC2209 DIR | 27 | |
| TMC2209 EN | −1 | 默认硬件把 ENN 接地；要用 IO 就填 GPIO 号 |
| TMC2209 PDN_UART | 16 | **单线半双工**：串 1 kΩ + 上拉 10 kΩ 到 3.3 V |
| MT6701 SDA / SCL | 21 / 22 | I²C 后端（默认） |
| MT6701 SCK / MISO / CS | 18 / 19 / 5 | 把 `FW_ENCODER_BUS` 改成 2 才用到 |
| 协议口 RX / TX | 4 / 2 | 9600 8N1，接天文台主控，走 `Serial2` |
| 调试命令行 RX / TX | 3 / 1 | 115200，板载 USB-串口桥（UART0） |

⚠ 协议口必须与 TMC 口用**不同**的 UART 实例。固件启动时会比对，撞车则自动降级为
纯 STEP/DIR 模式并在横幅里明确报出来，不会静默失灵。

---

## 3. 依赖库

按用户要求使用 **TMC2209 官方生态驱动库 `TMCStepper`**（trinamic 官方推荐的社区标准库）。
本工程不自行实现 UART 报文与 CRC8，寄存器读写全部走该库。

- Arduino IDE：库管理器搜索 `TMCStepper` 安装
- PlatformIO：`platformio.ini` 已声明 `teemuatlut/TMCStepper`，自动拉取

ESP32 适配要点（库在 ESP32 上的两个坑，本工程已处理）：

1. 库头文件里 `SW_CAPABLE_PLATFORM` 只对 AVR / LPC1768 / STM32 为真，
   ESP32 只能用 `TMC2209Stepper(Stream*, RS, addr)` 构造。
2. 库的 `beginSerial()` 在 ESP32 上被 `= delete`，
   **必须由我们自己调用 `HardwareSerial::begin()`**，再调用 `driver->begin()`。
   单线半双工时 RX 与 TX 传同一个 GPIO。

---

## 4. 编译烧录

### Arduino IDE
1. 安装 ESP32 开发板支持包；`工具 → 开发板 → ESP32 Dev Module`
2. 用文件夹名打开：`XagylFilterWheel_ESP32`（Arduino 要求 `.ino` 与目录同名）
3. 装好 `TMCStepper`，改 `Config.h` 里与硬件相符的引脚与机械参数
4. 烧录，打开串口监视器 115200（板载 USB-串口桥那一路）

### PlatformIO
```
pio run -t upload
pio device monitor
```

---

## 5. 初始位置怎么调（重点）

滤镜轮的"初始位置"由四个量决定，存在 flash 里，掉电不丢：

```
槽位 n 的目标角 = home + (n-1) × spacing + offset[n] × offsetUnit
```

| 参数 | 调试命令 | 协议命令 | 含义 |
|---|---|---|---|
| `home` | `home 137.25` | 无（安全起见只开本地） | 槽位 1 中心的绝对编码器角 |
| `spacing` | `space 72` / `space 0` | `I8` 只读 | 槽间距；填 0 表示自动 360/N |
| `offset[n]` | `offset 3 12` | `(0` / `)0` 每次 ±1 单位 | 单槽微调 |
| 上电策略 | `boot 0/1/2` | 无 | 见下 |

### 上电定位策略 `boot`

| 值 | 行为 | 适用 |
|---|---|---|
| **0** 推荐 | 读绝对角，就近吸附到槽中心。本来就在位就**一步都不走** | 绝对值编码器的主场，开机即知道自己在哪 |
| 1 | 上电主动走到槽位 1 | 想要每次开机位置绝对一致 |
| 2 | 恢复到掉电前记录的槽位 | 换相机、中途断电后接着用 |

如果轮子掉电时被人为扳到两个槽之间，策略 0/2 都会自动吸附到最近的槽中心 ——
这是磁编码器相对光电编码器最大的实际优势：**永不需要归零、永不存在归零失败**。

### 三种标定路径

**A. 示教式（最直观，推荐首次装配用）**
```
手动把想要的滤镜转到窗口正前方
> set 1            # 当前实测角成为槽位 1 的中心
> go 2             # 走到第 2 位，再手动对齐滤镜，然后
> set 2            # —— 间距会被自动反解出来
```

**B. 参数式（已知机械结构时最快）**
```
> space 72         # 5 位等分轮直接填 360/5
> home 0           # 槽位 1 对齐编码器 0°
```

**C. 自动校准（消除齿轮间隙与装配偏心）**
```
> cal              # 或上位机发 R6
```
逐槽走位 → 读实测角 → 残差累加进该槽 offset → 重走确认收敛 → 全部完成后回槽 1。
`diag` 里的 `worst=` 显示最大残差，正常应在 0.3° 以内。

⚠ **校准的能力边界（必须知道）**：MT6701 只测角度，**不知道哪个角度对应哪个滤镜**。
原厂 SBIG 靠轮上的磁铁+霍尔识别槽中心，本方案没有第二套槽位识别硬件，
所以自动校准只补偿"到位误差"，**不能自行发现槽中心**。
槽中心仍需用路径 A 或 B 标定一次。这一点在 `docs/PROTOCOL.md` 与代码注释里都写了。

---

## 6. 运动控制方式

**开环走位 + 闭环验位**，不是连续 PID。

```
读编码器实测角作起点 → 按角度差换算脉冲数 → 梯形加减速发脉冲
→ 沉降 120 ms → 读编码器验位 → 超差则低速补走残差（最多重试 N 次）→ 仍超差进 FAULT
```

为什么这么设计：滤镜轮是点对点定位机构，99% 时间在静止，周期性 PID 收益极低还会引入积分漂移。
这个分段方案能在同一次移动内检出并纠正丢步（卡阻、碰撞、皮带打滑），
而且实现简单、行为可预测。

关键参数（`Config.h`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `FW_MICROSTEPS` | 64 | 64 细分下每圈 12800 脉冲，单槽 72° = 2560 脉冲 |
| `FW_SPEED_MAX_HZ` | 3000 | 巡航脉冲频率 |
| `FW_ACCEL_HZ_PER_S` | 6000 | 恒加速梯形曲线 |
| `FW_POSITION_TOL_DEG` | 2.0 | 到位容差，必须大于 MT6701 的 INL（典型 ±1.0°、最大 ±1.5°） |
| `FW_SETTLE_MS` | 120 | 等机械振荡衰减后再验位 |
| `FW_MOVE_MAX_RETRY` | 2 | 纠错重试次数；协议 `]0` 可临时调大 |

脉冲由 `micros()` 调度，单次 `tick()` 最多占用 250 µs 就让出 CPU
（9600 波特一个字符约 1.04 ms），保证不会因发脉冲而丢协议字节。
需要 >20 kHz 脉冲频率时改用 LEDC/RMT 外设。

---

## 7. 接线与上手检查

```
1. 上电看横幅：encoder 应显示 ok，stepper 应显示 "STEP/DIR + TMC UART ok"
2. > diag          读一次角度，缓慢手动转轴，raw 应连续变化且方向正确
3. > space 72      按实际轮子设间距
4. > go 1 / go 2   确认转向正确；转向反了用 dirinvert 1 翻，不要改线
5. > set 1         示教对齐槽位 1
6. > cal           跑一次自动校准
7. > home / slots  核对各槽中心角
8. 接上位机：驱动连接应一次通过（握手读 I1/I8/I5/I9/I4/I7）
```

方向反了**用 `dirinvert 1`**，不要交换电机线 —— 交换 A/B 相会让编码器角与电机方向的
对应关系一起变，反而更难对齐。

---

## 8. 已知限制（诚实清单）

1. **SSI/SPI 后端的 CRC-6 位序未在硬件上验证过**。公开资料对帧内位序的描述存在歧义，
   因此 `FW_SSI_CHECK_CRC` 默认关闭，且校验失败只告警不丢帧。
   想启用：分别用 `FW_SSI_LSB_FIRST=1/0` 编译两次，看哪个 CRC 能稳定通过，那个就是正确位序。
   **I²C 后端不受此影响，默认即用 I²C。**
2. **自动校准找不到槽中心**，原因见第 5 节。需要真正的免示教标定得加一套槽位识别传感器。
3. **堵转检测默认关闭**（`FW_TMC_STALLGUARD=0`）。StallGuard 要求 spreadCycle 模式，
   与默认 StealthChop 静音模式冲突，二者需权衡。
4. **协议 `Jitter`/`Threshold` 是自定义映射**：本工程把 threshold 映射到到位容差、
   jitter 映射到重试次数。SBIG 原机不是这个含义，驱动侧只当字符串读，不会出错，
   但不要指望它和原厂数值等价。
5. **`T0` 回的是编码器原始计数 + 磁场状态码**，不是原厂那对光学传感器的读数。
   格式合规（`Sensors %d %d`），物理含义不同。
6. 运动是一次一动的阻塞式状态机（非阻塞推进但期间不接受新走位）。
   上位机发 `G` 后可随时用 `I2` 查询，驱动正是这么做的。

---

## 9. 文件结构

```
XagylFilterWheel_ESP32/
├── XagylFilterWheel_ESP32.ino   主程序：上电策略 + 主循环调度
├── Config.h                     全部可调参数（唯一需要改的文件）
├── Settings.h/.cpp              配置结构与 NVS 掉电保存（带 magic+version 头）
├── Encoder.h/.cpp               MT6701 抽象层（I²C / SSI 双后端，同一接口）
├── StepperDriver.h/.cpp         TMC2209 封装（基于官方库 TMCStepper）
├── Motion.h/.cpp                梯形脉冲 + 验位闭环
├── Protocol.h/.cpp              Xagyl 协议设备端实现
├── Calibrate.h/.cpp             逐槽自动校准状态机
├── Console.h/.cpp               调试命令行（独立串口，不参与协议）
├── docs/PROTOCOL.md             协议契约与实现细节
├── platformio.ini               PlatformIO 配置（可选）
└── .gitignore
```

---

## 10. 许可与致谢

- 协议来源：[rpineau/XagylFilterWheel](https://github.com/rpineau/XagylFilterWheel)（`xagyl.cpp` / `xagyl.h`）
- 驱动库：[teemuatlut/TMCStepper](https://github.com/teemuatlut/TMCStepper)
- 器件资料：Trinamic TMC2209 数据手册、MagnTek MT6701 数据手册
- 本工程代码按 MIT 许可发布
