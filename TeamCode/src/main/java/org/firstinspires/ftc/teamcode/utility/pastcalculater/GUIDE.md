# utility/pastcalculater —— 多代飞轮控制系统对比实验

> 目录名沿用用户历史写法 `pastcalculater`（少一个 l），**故意不改**，免得引用路径到处对不上。
> 本目录里的 **OpMode 也放在这里**，不遵守 `TeamCode/.../OpModes/**` 的分层要求（明确例外，
> 见 `FTC_Format.md` 的说明与用户的直接指示）。它只做实验，不参与正式控制。

---

## 1. 这个目录解决什么问题

第 4 代速度环重构后，只能证明“新的比旧的好一点”，但**好在哪、代价是什么**没有数据。
本目录把 DECODE 赛季以来的单飞轮控制律各留一份**精简、可复算**的实现，做到三件事：

| 想验证的事 | 用什么 |
|---|---|
| 同一段转速轨迹，各代给的占空比差多少 | `PastCompareOp`（在线/离线两种跑法） |
| 掉速（发射）后各代恢复快慢、有没有过冲 | `PastCompareOp` + 出图脚本 |
| 每一代自己的参数是多少、怎么单独整定 | `Gen1Params` / `Gen3Params` / `Gen4Params` + `TuneGenXOp` |

**关键前提**：在线时**一次只驱动一代**（Dashboard `activeGen` 或按 Y 切换），
另外两代用**完全相同的输入** `(target, velocity, vBus, dt)` 并行演算。
所以“每次跑一个”并不影响可比性——三代吃的是同一段实测转速序列。

---

## 2. 文件清单

| 文件 | 作用 |
|---|---|
| `PastGen.java` | 代际枚举：`GEN1_PIDSVA` / `GEN3_BOOST` / `GEN4_CURRENT` |
| `PastCalculator.java` | 公共骨架 + 三个精简实现 `Gen1` / `Gen3` / `Gen4`，统一 `calculate(target, velocity, vBus, dt)` |
| `Gen1Params.java` | 第 1/2 代参数容器（`@Config` static，Dashboard 可改） |
| `Gen3Params.java` | 第 3 代参数容器（同上） |
| `Gen4Params.java` | 第 4 代参数容器（镜像 `FlywheelParams.BIG`；`syncFromBig()` / `applyTo(...)`） |
| `PastFlywheelDriver.java` | 一电机 + 一计算器的驱动器（在线对比与分代 Tuner 共用） |
| `PastTuneSupport.java` | 最小二乘 + 一阶阶跃辨识（与 Tune6.1 同算法） |
| `PastCompareOp.java` | 多代对比主程序（在线 / 离线回放） |
| `TuneGen1Op.java` | 第 1/2 代整定：kS/kV（多点拟合）+ kP/kI（阶跃） |
| `TuneGen3Op.java` | 第 3 代整定：kS/kff + kp/ki + **kBoost（人为掉速）** |
| `TuneGen4Op.java` | 第 4 代**闭环验收**（§2.12 判据逐条量） |

---

## 3. 三代控制律对照

| 项 | Gen1/2 (PIDSVA) | Gen3 (boost) | Gen4 (现行) |
|---|---|---|---|
| 前馈 | `kS·sgn(ω*) + kV·ω* + kA·dω*/dt` | `kS·sgn(ω*) + kff·ω*` | `s·(kS·sgn + kff·ω*)`，`s=V_nom/V_busFilt` |
| 反馈 | `kP·e + kI·∫e + kD·de/dt` | `kp·e + ki·∫e` | `kp·e_f + ki·∫e_f`（`e_f` 用**滤波后**转速） |
| 积分门 | `|e| < iZone` 才积 | `|e| < iZone` 才积，掉速触发清零 | `|e| < eBand` 才积，冻结超 `iFreezeTimeoutSec` 强制清 |
| 掉速处理 | 无 | `boost = clamp(kBoost·e, 0, boostMax)`，持续 `boostDurationSec` | 无状态机；`|e| ≥ eBand` 时暂停吸收电压读数（2026-11-06 删掉速状态机，冻结窗口改为误差门表达） |
| 速度滤波 | 无 | 无 | EMA，`τ` 在 `tauVMax~tauVMin` 按误差调度 |
| 电压归一化 | 无 | 无 | 有（`useVoltageNorm` 开关，BIG/SMALL 均为 true） |
| 稳态判据 | 本实验统一：`|e| ≤ max(settleFrac·|ω*|, 2·quantStep)` 连续 `settleFrames` 帧（三代同口径，否则图没法并排比） |

> 第 3 代实现与 `0969cbd^` 的历史 `FlywheelCalculator` **逐行等价**（含 `dropArmed` 武装规则、
> `dVelocity < -dropRateThreshold` 判据、boost 计时退出）。要看原文：
> `git show 0969cbd^:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/Shooter/FlywheelCalculator.java`

---

## 4. 快速开始

### 4.1 在线跑一代（推荐流程）

1. Dashboard `activeGen` 选 `GEN4_CURRENT`（默认），Logging 里点 **Start** 开始记录。
2. 初始化后按 A/B/十字键给目标（与 `FlywheelRun` 同键位），跑同一套动作：
   `1600 稳定 3 s → 1900 稳定 3 s → 掉速/喂球 → 停`。
3. 换 `activeGen = GEN3_BOOST`，**重跑同一套动作**；再换 `GEN1_PIDSVA` 重跑。
4. 三段记录（Dashboard Logging 导出，三代的分量在**同一份**里都有，`gen` 列告诉你这次驱动的是哪一代）一起出图对比。

> 也可以不动 Dashboard：跑动中按 **Y** 循环切代（会清零目标并复位三代滤波器），
> 一次 OpMode 里就能采三代——但每次切代前最好先回 0 转速，避免切换瞬间的占空比跳变算进数据。

### 4.2 离线回放（不需要机器人）

1. 先有任意一份含 `velocity`、`target` 列的历史 CSV（Dashboard 记的 `FlywheelRun` 日志就是），
   文件名填进 `replayFile`，并把文件放进 **`<externalFilesDir>/tuning/`**（历史 CSV 都堆在这个目录）。
2. Dashboard 勾 `offlineReplay = true`，初始化并 START（**同时按住 Logging → Start** 记录）。
3. 三代计算器把这段真实转速序列各算一遍，遥测里 `gen` 键 = 0。
4. 只比 duty：`python tools/tuning/plot_csv.py 回放日志.csv --overlay duty_g1,duty_g3,duty_g4`

> 回放不需要真实时间，跑得比实时快，几秒读完上百秒的日志。
> 若 CSV 没有 `vbus`/`voltage` 列，母线电压按 `NaN` 传（第 4 代自动退化为 `s=1`，「不归一化」）。

### 4.3 出图

```powershell
python tools/tuning/plot_csv.py .\PastCompare_log.csv --group gen            # 每代一张，横轴各自时间
python tools/tuning/plot_csv.py .\PastReplay_log.csv  --overlay duty_g1,duty_g3,duty_g4
python tools/tuning/plot_csv.py .\PastCompare_log.csv --cols duty_g1,duty_g3,duty_g4,velocity,error
```

`plot_csv.py` 一列一张 PNG + 一份 `_summary.txt`，新列（`drop_g1`/`iact_g1`/`stl_g1`…）自动识别。

---

## 5. 遥测键（Dashboard CSV 的列，`PastCompare` / `PastReplay`）

```
gen,target,velocity,vbus,
duty_g1,ff_g1,p_g1,i_g1,drop_g1,iact_g1,stl_g1,
duty_g3,ff_g3,p_g3,i_g3,boost_g3,drop_g3,iact_g3,stl_g3,
duty_g4,ff_g4,p_g4,i_g4,drop_g4,iact_g4,stl_g4,vscale_g4
```

> 程序**不写自定义 CSV**：全部通过 `telemetry`（已绑 `MultipleTelemetry`）发布，
> 由 Dashboard 自带 **Logging** 记录，时间列由 Dashboard 加；键名与上面一一对应。
> 分代 Tuner 的报告数字（`fit_kV`、`step_kp`、`dip_recovery`…）同样是扁平数值键，一并记录。

- `gen`：本次**真正驱动**电机的是哪一代（1/3/4）；离线回放记 **0**。
- `ff_*`：前馈总量（Gen1 = `kS+kV·ω*+kA·dω*`，Gen3 = `kS+kff·ω*`，Gen4 = `s·(kS+kff·ω*)`）。
- `boost_g3`：只有第 3 代有这一列（Gen1 从未有 boost，Gen4 的 boost 已删除，故两家不列）。
- `drop_*` / `iact_*` / `stl_*`：掉速标志 / 积分激活 / 已稳定，1=真 0=假（`drop_g4` 恒 0：第 4 代的掉速状态机已于 2026-11-06 删除）。
- `vscale_g4`：第 4 代的电压缩放 `s`；恒为 1 说明 `useVoltageNorm=false` 或电压读数缺失。

分代 Tuner 的遥测键：

| 程序 | 键 | 说明 |
|---|---|---|
| `TuneGen1Op` | `duty,velocity,phase` + `fit_kV,fit_kS,step_kP,step_kI` | 纯开环辨识数据 |
| `TuneGen3Op` | `duty,velocity,error,boost,phase` + `fit_kff,fit_kS,step_kp,step_ki,dip_peak_error,dip_peak_boost,dip_recovery,dip_overshoot` | 含 boost 段 |
| `TuneGen4Op` | `target,velocity,error,duty,ff,p,i,iactive,settled,vbus,vscale,phase` + `meas_*`/`kp_step`/`dip_*` | 闭环验收 |

`phase` 数字码（三份通用）：**1** 开环点/稳态前 **2** 判读窗口 **3** 阶跃 **4** 掉速
**5** 恢复 **6** 掉速前稳定。

---

## 6. 各代怎么整定（每代三步，互不干扰）

### 6.1 第 1/2 代（`TuneGen1Op`）

1. `pointDuties` 给 4 个开环占空比（默认 0.15/0.25/0.35/0.45），每点稳 2 s 后取 1 s 均值。
2. 程序自动最小二乘 → **kS = 截距、kV = 斜率**（遥测 `>>> 建议 kV / kS`）。
3. 阶跃辨识 → **kP = τ/(Kd·tc)、kI = 1/(Kd·tc)**（`tc = tRecover/ratio`，默认 0.5/4 = 0.125 s）。
4. 抄进 `Gen1Params`（Dashboard 可改，跑 `PastCompare` 立刻生效）。`kD`、`kA` 历史一直是 0，保持 0。

### 6.2 第 3 代（`TuneGen3Op`）

1. 同上两步得 **kS/kff** 与 **kp/ki**。
2. 第三阶段会自动：闭环到 `boostTarget`（1900）→ 用 `dipDuty` 压 `dipSec`（0.35 s）→ 回闭环观察。
3. 遥测给 `峰值误差 / 峰值 boost / 恢复时间 / 过冲`：
   - 恢复时间 > `targetRecoverySec`（0.43 s）×1.1 → 打印 `>>> 建议 kBoost`（按比例放大，**不自动写回**）；
   - 过冲明显（> 40 tick/s）说明 `kBoost`/`boostMax` 偏大或 `boostDurationSec` 偏长。

### 6.3 第 4 代（`TuneGen4Op`）

**不开环辨识**（那是 Tune6.1/6.5/6.6/6.7 的活），只做**闭环验收**：

1. `targets`（1600/1900/2250）各稳 2 s 后判读 1 s：
   `v_sd / |误差|% / duty_sd / |∫| / settled%`，判据见 `doc/现在的控制系统.md` §2.12。
2. 掉速段：`dipTarget`（1900）闭环 → 压 `dipDuty` 0.35 s → 回闭环测**跌幅 / 恢复时间 / 过冲**，
   并检查恢复期 `isSettled()` 是否翻 `false`（应翻）。
3. 报告里直接给 `kp×quantStep`（合格 ≤ 0.05）。

> 参数改在 `Gen4Params`（Dashboard 可调）。要让正式程序也吃到这些值，改
> `FlywheelParams.java` 末尾 `BIG` 的 static 块（**唯一参数出处**），或用 `Gen4Params.applyTo(...)`。

---

## 7. 判读：三代到底差在哪

同样的目标与路径下，按这个顺序看：

1. **稳态占空比**（`duty_*` 平台段）：低电压时 Gen1/Gen3 必须靠积分补电压缺口，
   Gen4 开 `useVoltageNorm` 后由前馈直接补——看 `i_*` 是否长期非零。
2. **掉速恢复**：Gen3 有 boost（`boost_g3>0`）、Gen4 靠“冻结积分 + 不灌饱和”；
   比较恢复时间与过冲，不要只看是否恢复。
3. **积分饱和痕迹**：`iact=1` 却 `i_*` 长时间贴着 `maxI` → 该代积分门（`iZone`/`eBand`）不合身。
4. **稳态抖动**：`v_sd`、`duty_sd`；Gen1/Gen3 没有速度滤波，抖动天生比 Gen4 大，
   Gen4 的代价是 `tauV` 带来的相位滞后（恢复初期更“肉”）。

> 别跨电压比：三代都**没有**母线电压补偿的 Gen1，在 10.5 V 与 13 V 下的 duty 差异是物理决定的，
> 不能算控制律优劣（要看的是“同电压下”的差异）。

---

## 8. 坑

1. **`@Config` 只认 `static` 字段**，两个 `Params` 容器都是全 `static`，所以 Dashboard 能改；
   `FlywheelParams` 用的是实例字段 → **Dashboard 改不动**（第 4 代正式参数只能改源码重编译或跑 Tuner）。
2. `TuneGen1Op/TuneGen3Op` 前两阶段是**开环**（`setOpenLoopPower`），此刻**绝不能**再调
   `driver.update()`，否则闭环会覆写占空比。
3. 从开环切回闭环前调 `driver.resetController()`（重播滤波器，避免拿旧转速当加速度基准）。
4. 离线回放必须含 `velocity` 列；`t` 列缺失时按 0.02 s 递推（频率不对会让第 4 代的 `dt` 相关项失真）。
5. 同一台机器**不要同时跑两个本目录 OpMode**（都抢 `shooter` 电机）。
6. `PastCompare` 的 `gen` 列只表示“这一行是哪次运行/哪一代驱动”，**不是**每行三代代号；
   三代的分量都在同一行里（`duty_g1/g3/g4`）。
7. **记录方式统一走 Dashboard**：本目录所有 OpMode 都只用 `telemetry` 打数据（已绑
   `MultipleTelemetry`），由 Dashboard 自带 **Logging**（Start/Stop → 下载 CSV）记录，
   不再往机器人上写自定义 CSV 文件；需要历史旧文件时才用 `adb pull`。
