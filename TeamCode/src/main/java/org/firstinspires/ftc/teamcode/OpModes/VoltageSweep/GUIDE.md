# 电压适配性扫描（VoltageSweep）使用指南

对应实验：**同一目标转速，在不同母线电压下，占空比是否等价**——
即 `doc/现在的控制系统.md` §2.12 判据⑥
（满电 12.9~13.1 V 与放电 ~10.5 V，同目标 `duty·V_bus` 差 ≤ 3%）。

程序：`VoltageSweepOp`（`OpModes/VoltageSweep/`，Driver Station 名称 `VoltageSweep`，分组 `Tests`）。

---

## 1. 为什么不用「等电池放电」

等电池自然放电要几十分钟，而且电压一直在变，**同一条件无法重复**。
本程序的做法：

```
负载电机（第二个电机）抽电流  →  电池内阻上的压降变大  →  母线电压下降
        ↑                                                   |
        └──────── 电压闭环：稳定在设定值 ±0.15 V ←───────────┘
```

负载电机由 **PI + 速率限幅** 控制（`kpVolt` / `kiVolt` / `loadRateLimit`），
把母线电压按住不放。于是**一块电池、一次运行就能把 8.5 / 10.0 / 11.5 / 12.9 V 四个点全跑完**，
且每个点可重复。

> 这不是「模拟电池」，只是「模拟低电量时的母线电压」。电池的可用电流、内阻会随 SoC 变化，
> 本程序不改变 SoC，只改变**负载电流**。所以它验证的是**前馈对母线电压的补偿**，
> 不是「电池快没电时整个系统还能不能打」的电压跌落动力学。

---

## 2. 硬件准备

| 项 | 要求 |
|---|---|
| 负载电机 | 一个**空闲电机口**（Control Hub / Expansion Hub 均可），随便什么电机，**不需要编码器** |
| 负载电机接法 | 用 `setPower` 当电气负载；**必须带一点机械负载**，否则抽不动电流 |
| 负载形式 | ①轴上装小风扇/桨叶（空气阻尼，最省事、连续可调）；②橡皮筋或皮筋轻压轴（轻微堵转）；③轴上加配重块 |
| 命名 | Robot Configuration 里命名为 **`loadMotor`**（或改 Dashboard 的 `loadMotorName`） |
| 方向 | 只要转起来就是负载；若发现**加占空比母线反而升高**（反电动势），把 `loadDirectionReversed` 置 true |

> ⚠ 不要用手长时间压轴，也不要让负载电机完全堵死：堵转电流可达 9 A 以上，
> 会触发 Hub 过流保护并让整台机器掉电。目标是「略微堵转」，占空比通常落在 0.2~0.7。

---

## 3. 参数（FTC Dashboard → `VoltageSweep` 分组）

### 3.1 计划

| 参数 | 默认 | 含义 |
|---|---|---|
| `aVelocity` | 1600 | 稳态段 A 的目标转速 (tick/s) |
| `bVelocity` | 1900 | 稳态段 B 目标转速，也是掉速段工作点 |
| `vbusTargets` | `{8.5, 10.0, 11.5, 12.9}` | 要扫的母线电压点 (V)，**按书写顺序**依次跑 |
| `voltageSettleSec` | 2.0 | 每个电压点开始前「把母线拉到目标」的时间（飞轮不转） |
| `steadySec` | 3.0 | 每个稳态段时长（分析取末尾 1~2 s） |
| `preDisturbSec` | 1.0 | 掉速前基线 |
| `dipSec` | 0.35 | 模拟掉速时长 |
| `dipDuty` | 0.15 | 掉速期间的开环占空比（越小跌越深） |
| `recoverySec` | 2.0 | 掉速后恢复观察窗 |
| `releaseSec` | 0.5 | 每个电压点结束后的释放时间（负载归零、飞轮停） |
| `disturbanceEnabled` | true | 关掉就只跑稳态段，实验时间减半 |

### 3.2 负载与安全

| 参数 | 默认 | 含义 |
|---|---|---|
| `loadMotorName` | `loadMotor` | 负载电机名（找不到不会崩，会提示并跳过负载） |
| `loadDirectionReversed` | false | 负载电机方向 |
| `loadPowerMax` | 1.0 | 负载占空比上限 |
| `loadRateLimit` | 2.5 | 占空比变化率上限 (占空比/s)，越小越稳、拉电压越慢 |
| `kpVolt` / `kiVolt` | 0.45 / 0.8 | 电压环 P / I（单位：占空比/V、占空比/(V·s)） |
| `minSafeVbus` | 7.5 | 母线低于此值立即中止 (V) |
| `vbusTolerance` | 0.15 | 「稳定在目标 ±该值」仅用于遥测显示 |

### 3.3 记录

| 参数 | 默认 | 含义 |
|---|---|---|
| `dashboardLog` | **true** | 发布扁平数值遥测，由 **FTC Dashboard 自带 Logging** 记录（本程序默认开，不记录就没有意义） |
| `logPeriodMs` | 10 | 采样周期 (ms)，≈100 Hz |

> 本程序**不写自定义 CSV**。数据全部走 `telemetry`（已绑 `MultipleTelemetry`），
> 在 Dashboard 里点 **Logging → Start** 即可记录；导出的 CSV 列名 = 下面的遥测键名，
> 时间列由 Dashboard 自己加。

---

## 4. 操作流程

### 4.1 接线自检（Init 期间）

1. RC 上选 `VoltageSweep`，按 **INIT**。
2. 按 **Y** 进手动负载模式：十字键上/下直接给负载电机占空比。
3. 观察遥测 `vbus` 是否随负载变低、`负载占空比` 是否跟手 → 说明负载电机接对了。
4. 再按 **Y** 退出手动模式（占空比归零）。

> 若加负载后 `vbus` 几乎不动：负载太小（风扇太小 / 轴太松），或负载电机接在了另一块电池上。

### 4.2 自动扫描

1. 按 **START**。程序按计划跑：
   `V_settle → V_a1600 → V_b1900 → V_pre → V_dip → V_rec → V_rel`，
   对 `vbusTargets` 里每个电压点重复一遍。
2. 全程遥测显示：当前段、进度、`母线 目标/实测/差`、`母线合格 YES/NO`、负载占空比、
   飞轮 `目标/实测/duty`、电流、`inDrop/iActive/settled`。
3. 跑完（或按 **X** 中止）后停在**汇总屏**，按 **X** 退出。

4.6 个电压点 × 7 段 ≈ **47 s**（默认参数）。

### 4.3 取数据

数据在 **Dashboard Logging** 里：程序跑起来后点 `Logging → Start`，跑完点 `Stop`，
在 Logging 面板里把 CSV 下载下来（列名就是遥测键名，见 §4.4）。

```bash
# 一图一表：每列一张 PNG + 汇总表（推荐先跑这个）
python tools/tuning/plot_csv.py ./VoltageSweep_log.csv \
       --segment segment_id --out ./tools/tuning/out

# 分量叠图：看 duty 与电压的关系
python tools/tuning/plot_csv.py ./VoltageSweep_log.csv \
       --overlay duty,vbus_meas --tag duty_vs_vbus --out ./tools/tuning/out
```

### 4.4 遥测键（Dashboard CSV 的列）

`segment_id, vbus_target, vbus_meas, load_power, target, velocity, duty, error,
current, iactive, settled`

- `segment_id`：计划段编号（`_settle/_a1600/_b1900/_pre/_dip/_rec/_rel` 依次递增），
  段名与电压目标请看程序开跑的遥测屏（或按每段的 `vbus_target` 反查）。
- `duty`：掉速段读的是电机实际输出（开环值），其余段读闭环命令值。

---

## 5. 判读

### 5.1 汇总屏（程序自己算的）

- **每段一行**：`Vbus / VbusSD / v_avg / vSD / duty / d×Vbus / |I| / settled%`
  ——只统计该段**后半段**（跳过前 50% 暂态）。
- **跨电压一致性**：对 `aVelocity`、`bVelocity` 两个目标，比较各电压点的 `duty·V_bus`，
  打印 `差 x.xx% PASS/FAIL(>3%)`。
- **`kp×台阶`**：顺手核对 §2.12 判据①。

### 5.2 图上的信号

| 现象 | 含义 | 处理 |
|---|---|---|
| `vbus_meas` 在每段开头下冲后回稳，`|vbus_meas−vbus_target| ≤ 0.15` | 电压环收敛，该点数据**有效** | — |
| `load_power` 长期贴着 `loadPowerMax` 而 `vbus_meas` 仍高于目标 | 负载不够，**这个点没拉下去** | 加机械负载 / 提高 `loadPowerMax`（谨慎）/ 去掉最高那个电压点 |
| `load_power` 在 0 与某值之间大幅来回跳 | 电压环增益过大 | 降 `kpVolt`（0.45→0.25）或 `kiVolt`，或降 `loadRateLimit` |
| `duty·V_bus` 跨点差 > 3% | 前馈**没有**补偿电压（常见：`useVoltageNorm=false`） | 见 §5.3 |
| `inDrop` 在 `_rec` 段起点为 1 且很快回 0 | 掉速判据正常 | — |
| `settled` 在稳态段末尾仍为 0 | 该电压点下 `|e|` 进不了 `eBand` | 看 `velocity` 图是否真的抖，或该点电压太低带不动 |

### 5.3 关键前提：`useVoltageNorm`

第 4 代的电压归一化只乘**前馈**：

```
s = clamp(V_nom / V_busFilt, vScaleMin=0.85, vScaleMax=1.25)
ff = s·(kS·sgn(ω*) + kff·ω*)
```

- `useVoltageNorm = false` → `s ≡ 1`，**前馈不随电压变化**，低电压时占空比必然更高 →
  判据⑥**注定 FAIL**（2026-09-02 19:07 的日志就是这样：`vscale≡1`）。这时本程序的作用是**量出「不补偿差多少」**。
- `useVoltageNorm = true`（**2026-10-27 起 `BIG`/`SMALL` 的默认值**）→ `s = 12/V_busFilt`，
  前馈自动补齐母线跌落，判据⑥应通过。首次开之前先按 `Tune6.7` 定好 `tauBus`。

### 5.3.1 已完成的实测（2026-10-27 20:38，大球）

`vbusTargets = {10.5, 11.0, 11.5, 12.0}`（低到 8.5 V 单负载电机压不下来，负载占空比已到 0.91），
`aVelocity=1600`、`bVelocity=1800`：

| 档位 | 母线实测 | 负载占空比 | duty@1600 | duty@1800 | duty·V_bus@1600 | duty·V_bus@1800 |
|---|---|---|---|---|---|---|
| 10.5 V | 10.52 | 0.91 | 0.740 | 0.822 | 7.782 | 8.663 |
| 11.0 V | 10.96 | 0.78 | 0.707 | 0.792 | 7.756 | 8.708 |
| 11.5 V | 11.44 | 0.48 | 0.677 | 0.750 | 7.740 | 8.616 |
| 12.0 V | 11.86 | 0.01 | 0.642 | 0.721 | 7.608 | 8.621 |

结论：**跨电压最大偏差 1600 → 2.2%、1800 → 1.1%，判据⑥ PASS**；母线闭环稳态
`max|实测−目标| = 0.141 V`；各档稳态误差 ≤2 tick/s、`duty_sd` 0.007~0.013、`settled` 100%。
`_dip` 段（开环压到 0.15）速度掉到 950~1000 后 0.5 s 内回到目标，各档 `settled` 段全 100%。

> 因此本程序的第一用途是**量化开关前后差异**，第二用途才是验收。

### 5.4 与真实发射的差别

`_dip` 段用**开环把占空比压到 `dipDuty`** 制造掉速，等价于「突然失去动力」；
真实发射是「突然增加机械负载」。两者对控制器的挑战不同：

| | 本程序 `_dip` | 真实发射 |
|---|---|---|
| 掉速方式 | 输出被压到 0.15 | 轮子被球拖慢，输出反而顶到 1.0 |
| 电流 | 掉速期间**小** | 掉速期间**大**（9 A 量级） |
| 恢复段母线 | 电压**回弹**（电流变小） | 电压**继续下陷**（电流变大） |

所以 `_dip` 只能验证「控制器从低速追回目标」的速度与过冲，
**不能**验证「大电流 + 母线跌落」的耦合。真实发射与电压耦合的验证仍要带球（§2.10 第 6 步）。

---

## 6. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 遥测一直 `!! 找不到负载电机 "loadMotor"` | Robot Configuration 没命名，或名字不一致 → 改 `loadMotorName` |
| 一按 START 就中止 | `minSafeVbus=7.5` 太低说明不是正常电池；先量空载母线电压，确认 > 9 V |
| 加了负载母线反而升高 | 负载电机在发电（反电动势）→ `loadDirectionReversed=true` |
| 8.5 V 那个点跑不完 | 负载不够或电池内阻太大 → 提高 `loadPowerMax`、加机械负载，或把 `vbusTargets` 改成 `{10.0, 11.5}` |
| 扫描到一半 RC 掉电重启 | 负载电流 + 飞轮加速电流叠加过大 → 降低 `loadPowerMax`、把 `voltageSettleSec` 加长（让飞轮先停着把电压稳住） |
| 汇总屏 `d×Vbus` 差很大但 `useVoltageNorm=true` | `tauBus` 太小/太大 → 跑 `Tune6.7` |
| 数据里 `duty` 在 `_dip` 段是 0.15 而不是闭环值 | **正常**：掉速段走开环，程序读的是电机实际输出 |

---

## 7. 一次完整实验的建议顺序

1. 空载量母线电压（万用表对比 Dashboard 的 `vbus`，差 > 0.2 V 先查电压传感器）。
2. Init 期手动负载自检（§4.1）。
3. `useVoltageNorm=false` 跑一遍（基线：不补偿差多少）。
4. Dashboard 打开 `useVoltageNorm=true`，`Tune6.7` 定 `tauBus`。
5. 再跑一遍（应看到 `duty·V_bus` 差缩小到 ≤3%）。
6. 把两份 Dashboard 日志一起喂给 `plot_csv.py`，`--overlay duty,vbus_meas` 对照。
