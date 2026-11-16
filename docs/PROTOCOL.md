# Xagyl 滤镜轮协议 —— 设备端实现契约

本文描述本固件实现的串口协议，内容依据 `rpineau/XagylFilterWheel` 的
`xagyl.cpp` / `xagyl.h` 源码逆向整理，是**驱动实际发出和期待的报文**。

---

## 1. 链路层

| 项目 | 取值 | 驱动源码依据 |
|---|---|---|
| 接口 | RS-232 / USB-CDC 串口 | `CXagyl::Connect()` |
| 波特率 | **9600** | `m_pSerx->open(szPort, 9600, ...)` |
| 数据格式 | 8N1，无校验 | `B_NOPARITY`，注释 `// 9600 8N1` |
| DTR | 置位 | `-DTR_CONTROL 1` |
| 单字节读超时 | 1000 ms | `MAX_TIMEOUT` |
| 换位总超时 | **25 秒** | `MAX_FILTER_CHANGE_TIMEOUT` |
| 应答缓冲区 | **32 字节** | `SERIAL_BUFFER_SIZE` |

> **32 字节是硬约束**：`readResponse()` 最多累计读 32 字节，末尾要剥掉 CR+LF，
> 所以应答文本有效长度必须 ≤ 30 字符。
> 本固件把上限收紧到 30（`Protocol.cpp` 的 `REPLY_MAX_TEXT`），并在发送前 `strncpy` 截断，
> 保证任何格式化都不会溢出这条约束。

---

## 2. 帧格式：一个关键的不对称

### 2.1 主机 → 设备

```
<命令字符><参数字符>          例：G3
```

- 纯 ASCII，**2 字节定长**
- **不带任何结束符**：`writeFile(szCmd, strlen(szCmd))`，既不发 `\r` 也不发 `\n`
- 发送前执行 `purgeTxRx()`，**清空收发双向缓冲区**

> 设备端因此**必须靠字节计数判帧**，不能等行结束符。
> 本固件实现为两态状态机（`Protocol::tick`）：等第 1 字节 → 等第 2 字节 → 分发。
> 期间遇到的控制字符（≤ 32）逐个丢弃，兼容某些终端会附带换行的情况。

### 2.2 设备 → 主机

```
<ASCII 文本>\r\n
```

- `readResponse()` 逐字节读到 `0x0A`（LF）为止，然后**固定把末尾两个字节置 0**

```cpp
} while (*szBufPtr++ != 0xA && ulTotalBytesRead < nBufferLen);
if (ulTotalBytesRead) {
    *(szBufPtr-2) = 0;   // 假定倒数第二字节是 CR
    *(szBufPtr-1) = 0;   // 假定最后一字节是 LF
}
```

> ⚠ **必须发完整 CRLF**。只发 `\n` 时，倒数第二个字节是有效内容，会被误删：
> `P3\n` → 剥成 `P` → `sscanf("P%d")` 失配。
> 本固件所有应答统一走 `Protocol::reply()`，内部固定追加 `\r\n`。

### 2.3 无应答命令

`filterWheelCommand(cmd, NULL, 0)`：`szResult` 为 NULL 时**完全不读应答**。
驱动只在 `moveToFilterIndex()`（发 `G`）时这样调用。

> 驱动不读，但它**不等**。设备回什么、回不回都不影响驱动；
> 到位判定完全靠后续轮询 `I2`。
> 滞留在缓冲区的多余字节会被下一次命令前的 `purgeTxRx()` 清掉。

**本固件仍然坚持回一行 `Moving`**，理由是驱动的 `getSlotParams()` 发 `G` 后
把 `nResultMaxLen` 误传成 0 却仍调用 `readResponse()`（见第 7 节缺陷 1）。
有应答时它读满一次就返回；完全静默时它会白等 1 秒超时。回一行更省事。
可用 `FW_PROTO_REPLY_ON_MOVE=0` 关闭。

---

## 3. 连接握手序列

`Connect()` 严格按下表执行，任一步失败即关闭串口并返回错误：

| 步骤 | 命令 | 期待应答 | 失败错误码 |
|---|---|---|---|
| 1 | 打开串口 | — | `ERR_COMMNOLINK` |
| 2 | `I1` | 固件版本 | `FIRMWARE_NOT_SUPPORTED` |
| 3 | 本地归一化版本号 | — | — |
| 4 | `I8` | `FilterSlots <n>` | `ERR_CMDFAILED` |
| 5 | `I5` → `I9` → `I4` → `I7` | 见第 4 节 | `ERR_CMDFAILED` |

### 版本号归一化陷阱

`convertFirmwareToFloat()`：删掉除第一个 `.` 之外的所有 `.`，再 `atof()`。

| 设备返回 | 归一化结果 | `atof` 得到的浮点值 | 驱动行为 |
|---|---|---|---|
| `4.2` | `4.2` | **4.2** | 启用 `R7`、`S` 按 16 档解释 ✅ 本固件采用 |
| `3.1.5` | `3.15` | 3.15 | 走「≤3.4 老固件」分支 |
| `FW3.1.5` | `FW3.15` | **0.0** | 误入老固件分支，`R7` 永不触发 ⚠ |
| `FW3.1.5`（SBIG 原样） | 同上 | 0.0 | 同上 |

> **`atof` 遇到字母 `F` 直接返回 0**。SBIG 风格带 `FW` 前缀的版本串会让驱动的版本判定完全失效。
> 本固件 `I1` 回纯数字 `4.2`，由 `FW_PROTO_VERSION_STR` 控制。

---

## 4. 命令集与应答模板

### 4.1 信息查询 `I<n>`

| 命令 | 本固件应答 | 驱动解析模板 | 合规 |
|---|---|---|---|
| `I0` | `ESP32-Xagyl-FilterWheel` | 原样保存 | ✅ |
| `I1` | `4.2` | 原样保存后归一化 | ✅ 不带字母前缀 |
| `I2` | `P<n>` 静止 / `Moving` 走位中 | `strstr("Moving")` 否则 `sscanf("P%d")` | ✅ |
| `I3` | `S/N:FW0001` | 原样保存 | ✅ |
| `I4` | `MaxSpeed <0-100>` | **`MaxSpeed %d`（无空格）** | ✅ SBIG 写法不合规 |
| `I5` | `Jitter <n>` | `Jitter %d` | ✅ |
| `I6` | `P<n> Offset <u>` | `P%d Offset %d` | ✅ 真实值 |
| `I7` | `Threshold <n>` | `Threshold %d` | ✅ 注意拼写，SBIG 有一版是 `Theshold` |
| `I8` | `FilterSlots <n>` | `FilterSlots %d` | ✅ |
| `I9` | `Pulse Width 0mS` | `Pulse Width %dmS` | ✅ 见下 |

**`I2` 的两种合法应答**
- 静止：`P<当前实测槽位>`
- 走位中：包含 `Moving` 子串的任意文本

**`I9` 返回 0 的含义**（唯一的设备能力声明途径）：

```cpp
if (nPulseWidth == 0) m_bHasPulseWidthControl = false;
else                  m_bHasPulseWidthControl = true;
```

直驱方案没有「用脉宽编码位置」这回事，报 0 让驱动认定不支持，
于是它**永远不会发 `M0`/`N0`**。这比回一个假数字更诚实 —— 用能力协商代替虚假实现。
若旧驱动或手工终端仍发来，本固件幂等地回同一个值。

### 4.2 定位 `G<n>`

- 绝对定位，**协议没有相对移动命令**：驱动从不发 `B<n>`/`F<n>`
- 驱动不读应答，靠轮询 `I2` 收敛
- 换位必须 25 秒内完成

**本固件对越界 `G0`/`G7` 的处理**：回当前真实位置 `P<n>`。
驱动按「还没到目标」继续轮询，25 秒后自行报 `XA_COMMAND_FAILED`。
这样既不谎报到位，也不会让等待 `P` 的主机挂住。

**到位轮询状态机**（`isMoveToComplete`）：

```
1. target == current → 立即完成
2. 版本 ≤ 3.4 → 先 bytesWaitingRx() 探测（老固件走位期间完全不应答）
3. 发 I2 读应答
4. 含 "Moving"      → 未完成
5. sscanf("P%d") 失配 → 返回 OK 但视为杂散应答，下轮再读
6. target == 0      → 用读到的值当 target
7. 读到 == target   → 完成
8. 超 25 秒         → XA_COMMAND_FAILED
```

> 对设备的要求：走位期间**不能**回一个尚未到位的 `P<n>`，否则第 7 步会误判提前完成。
> 本固件走位中一律回 `Moving`，静止后回实测位置。

### 4.3 参数调节（只有 ±1 步进，没有直接赋值）

| 命令 | 本固件语义 | 应答 |
|---|---|---|
| `(0` | 当前槽偏移 **+1 单位**，立即重新对中，写入 flash | `P<n> Offset <u>` |
| `)0` | 当前槽偏移 **−1 单位**，同上 | `P<n> Offset <u>` |
| `S<hex>` | 巡航速度，16 档 | `MaxSpeed <pct>` |
| `[0` `]0` | 纠错重试预算 −1/+1 | `Jitter <n>` |
| `{0` `}0` | 到位容差 ∓0.25° | `Threshold <n>` |
| `M0` `N0` | 幂等（能力已声明为不支持） | `Pulse Width 0mS` |

**偏移单位**：`FW_OFFSET_UNIT_DEG = 0.25°`。驱动拿到的数字就是单位计数值。

**为什么必须限幅**：协议只有 ±1 步进，`setSlotParams()` 靠循环发 N 次命令逼近目标值，
差 100 单位就要发 100 条命令、读 100 条应答。本固件把偏移硬性限制在**半个槽距**以内
（`Settings::setOffsetUnits`），既防止槽位互相穿插，也让收敛可控。

**`S` 是唯一参数可为字母的命令**：

```cpp
snprintf(szCmd, ..., "S%X", nRotationSpeed / 10);
```

| 固件版本 | 参数范围 | 每档 | 驱动换算 |
|---|---|---|---|
| < 4.2 | `0`–`A` | 10% | `speed/10` |
| ≥ 4.2（本固件） | `0`–`F` | ≈6.66% | `(int)((speed/100.0f)*15+0.5)` |

驱动采用两段式尝试：先按旧制发一次，读回 `MaxSpeed %d`；不符再按 16 档公式重发。

> ⚠ SBIG 系固件的 `isDigit(secondChar)` 校验会**直接拒收 `SA`..`SF`**。
> 本固件对 `S` 单独走十六进制解析路径，其余命令仍要求数字参数。

### 4.4 `T0` 传感器原始值

| | |
|---|---|
| 驱动模板 | `Sensors %d %d` → `filter_params.nLL`, `nRR` |
| 本固件应答 | `Sensors <14位原始计数> <磁场状态码>` |
| SBIG 系固件 | 只回单个数字（Smikkelsen 回 `1`，Chemistorge 回引脚号）→ 解析失败 |

诚实说明：原厂有一对光学槽位传感器，本方案只有单只磁编码器，
因此两字段填的是「编码器原始计数」与「磁场状态码」——
都有物理意义、都是有效诊断量，但**与原厂含义不同**，不能当成左右限位读数解读。

### 4.5 校准 `R6`（与 SBIG 语义完全不同）

```
startCalibration():  发 R6 → strstr(resp,"Calibrating") 校验
isCalibrationComplete(): 不发任何命令，只 bytesWaitingRx() 探测
                         有数据才 readResponse()，strstr(resp,"Done") 判完成
                         完成后驱动把当前位置硬记为槽 1
```

**这是本协议中唯一的设备主动上报（unsolicited）报文。**

本固件实现：
1. 收到 `R6` → 启动逐槽校准状态机，立即回 `Calibrating`
2. 校准期间：`tick()` 推进状态机（走位 → 测残差 → 写补偿 → 重走确认 → 下一槽）
3. 全部完成后回到槽 1，然后**主动推送** `Calibration Done`
4. 失败同样必须推 `Done`（协议里这是唯一结束信号），改由后续 `I2`/`I6` 的真实数值体现失败

> ⚠ 校准期间驱动的 `m_bCalibrating` 标志从未被置真（缺陷 2），
> 所以查询命令仍会真的发过来。本固件的 `I2` 在校准时回 `Moving`，安全。

**校准的能力边界**：本方案没有第二套槽位识别传感器，
校准只补偿「到位误差」，**无法自行发现槽中心**。槽中心需先用示教或参数方式标定。

### 4.6 复位族

| 命令 | 本固件 | 备注 |
|---|---|---|
| `R0` `R1` | 存配置 → 延时 → 真实复位 | SBIG 固件只回字符串、不复位 |
| `R2` | 清全部偏移 + 保存 + 回槽 1 | 回 `Calibration Removed` |
| `R3` `R4` `R5` | 回读 jitter / speed / threshold | 格式与驱动模板一致 |
| `R6` | 启动自动校准 | **不是** SBIG 的「保存 EEPROM」 |
| `R7` | 恢复全部出厂默认 | 驱动仅在版本 ≥4.2 才发，并置 `bNeedCal` |

`R7` 后驱动会提示上层需要重新校准 —— 语义正确，本固件恢复默认后确实该重新标定。

### 4.7 协议中存在但驱动不使用

`B` `F` `O<n>` `R6 之外的保存语义` `<` `>` `I2` 的 SBIG 专有变体等。
本固件对 `B`/`F` 保留实现（环形进退 + 防死循环限幅），
因为同一台设备可能被 ASCOM/INDI 通用滤镜轮驱动使用。

---

## 5. 位置回报的设计原则

**一律回报编码器实测反推的槽位，绝不回报「命令里期望的位置」。**

```cpp
int Protocol::currentSlot() const {
  if (!g_encoder.healthy()) return 0;              // 传感器不可信 → 报 0（未知）
  return g_settings.slotForAngle(g_encoder.angleDeg());
}
```

- `slotForAngle()` 把实测角与每个槽中心比较，超过半槽距则返回 0（不在任何槽位上）
- 走位失败进 `FAULT` 时，`I2` 回 `Moving error`：含 `Moving` → 驱动继续轮询 →
  25 秒后如实报超时，而不是接受一个假的到位信号

这与两份社区 SBIG 固件形成对照：它们走位失败仍回目标位号，上位机会误以为成功。

---

## 6. 自定义语义声明（不与原厂等价，必须知道）

| 命令 | 原厂含义 | 本固件实现 |
|---|---|---|
| `Threshold` (`I7` `{0` `}0`) | 到位判定的传感器阈值 | **到位容差**，每单位 ±0.25°，基准 1 = `FW_POSITION_TOL_DEG` |
| `Jitter` (`I5` `[0` `]0`) | 电机抖动/微步扰动参数 | **纠错重试次数预算**，每单位 +1 次 |

这样做的理由：这两个量在 SBIG 固件里只是被读出来的字符串，
本方案有真实的闭环验位机制，把它们映射到真正起作用的参数，
比保留一个"看起来能调但完全没作用"的假接口更有价值。
驱动侧只当字符串读，不会出错。

---

## 7. 驱动侧已知缺陷（设备端无法规避）

这些是 `xagyl.cpp` 源码中客观存在的问题，列出来避免误判为固件 bug。

**1. `getSlotParams()` 拷贝长度恒为 0**

```cpp
nErr = filterWheelCommand(szCmd, szResp, 0);   // nResultMaxLen = 0
rc = sscanf(szResp, "P%d", &nSlot);
```

应答被读进函数内局部数组，`strncpy(szResult, szResp, 0)` **一个字节都不拷贝**，
调用方的 `szResp` 是未初始化栈内存。`sscanf` 读垃圾数据：
- `rc == 0` → 返回 `XA_COMMAND_FAILED`
- 垃圾恰好解析出不等于目标的数 → `while (nSlotNumber != nSlot)` **死循环**

影响：需要跨槽位移动时 `getSlotParams()` / `setSlotParams()` 基本不可用。

**2. `m_bCalibrating` 从未置真** —— 校准期间查询不会被屏蔽，设备端须自行给出安全应答。

**3. `readResponse()` 假定 CRLF** —— 设备只发 LF 会吃掉最后一个有效字符。

**4. `strncpy` 不保证 NUL 结尾** —— 因缓冲区已 `memset` 清零且应答 ≤30 而实际安全，
但应答超长填满缓冲区时存在越界读风险。（本固件的 30 字符截断正是针对此条。）

**5. 版本浮点化对字母前缀敏感** —— 见第 3 节。

---

## 8. 与 SBIG 系固件的兼容性总表

| 项目 | Xagyl 驱动期待 | SBIG 系固件实际 | 本固件 |
|---|---|---|---|
| `I4` | `MaxSpeed %d` | `Max Speed 400` ❌ | ✅ |
| `I7` | `Threshold %d` | `Threshold 1` / `Theshold 1` ❌ | ✅ |
| `I9` | `Pulse Width %dmS` | `...4950uS` ⚠ 数值可解析 | ✅ 报 0 声明能力 |
| `I1` | 纯数字版本 | `FW3.1.5` ⚠ 归一化成 0.0 | ✅ `4.2` |
| `T0` | `Sensors %d %d` | 单值 ❌ | ✅ |
| `S<hex>` | 参数可 A–F，回 `MaxSpeed %d` | `isDigit` 拒收 ❌ | ✅ |
| `R6` | 启动校准 + `Done` 上报 | 存 EEPROM，不应答 ❌ | ✅ |
| `G` 越界 | — | 英文错误串，无 `P` ⚠ | 回真实 `P<n>` ✅ |
| 走位失败 | — | 谎报目标位号 ❌ | 如实反映 ✅ |
| `(0`/`)0` | 真实 ±1 偏移 | 桩，恒 `Offset 0` ⚠ | ✅ 真实生效 + 掉电保存 |
| 命令结束符 | 不发 | 主动剥离 ≤32 控制符 | ✅ |
| 应答结束符 | 必须 CRLF | `Serial.println()` | ✅ |

**结论**：把社区 SBIG 系固件直接接 Xagyl 驱动会在 `Connect()` 的 `I4` 一步失败；
本固件按驱动真实解析逻辑逐条对齐，握手可一次通过。

---

## 9. 手工验证（不接上位机）

设备自带 `proto` 命令，可直接注入协议命令观察完整解析路径：

```
> proto I4          服务台打印 injected，应答从协议口发出
> wheeldiag         看 cmds / bytes / bad / replies 计数是否 +1
```

接 USB-TTL 到协议口（GPIO4/2）做逐条验证：

```bash
# 握手序列
printf 'I1' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1 | xxd   # 期望 34 2e 32 0d 0a = "4.2\r\n"
printf 'I8' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1          # 期望 "FilterSlots 5\r\n"
printf 'I4' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1          # 期望 "MaxSpeed 100\r\n"
printf 'I9' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1          # 期望 "Pulse Width 0mS\r\n"
printf 'I7' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1          # 期望 "Threshold 1\r\n"

# 走位与轮询
printf 'G3' > /dev/ttyUSB1                                      # 期望 "Moving\r\n"（可选）
printf 'I2' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1           # 走位中 "Moving"，完成后 "P3"

# 调速（字母参数）
printf 'SF' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1           # 期望 "MaxSpeed 100\r\n"
printf 'S8' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1           # 期望 "MaxSpeed 53\r\n"

# 校准
printf 'R6' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1           # 期望 "Calibrating\r\n"
# 等数十秒，不发消息，应收到主动上报
timeout 60 cat /dev/ttyUSB1                                      # 期望 "Calibration Done\r\n"
printf 'I2' > /dev/ttyUSB1; timeout 1 cat /dev/ttyUSB1           # 校准后应回 "P1"

# 偏移
printf 'I6' > /dev/ttyUSB1                                       # "P<n> Offset 0"
printf '(0' > /dev/ttyUSB1                                        # "P<n> Offset 1"，轮子重走对中
printf 'O3' > /dev/ttyUSB1                                        # "P3 Offset <u>"
```

判断是否合规的快捷标准：**任何应答用 `sscanf` 套第 4 节的模板，必须返回 ≥1。**
