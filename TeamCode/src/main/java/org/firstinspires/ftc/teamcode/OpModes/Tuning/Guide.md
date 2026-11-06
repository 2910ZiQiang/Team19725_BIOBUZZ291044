
# 飞轮参数整定 OpMode 使用指南（第 4 代）

本目录提供 **5 个** 整定辅助 OpMode，全部只用「占空比输入 + 速度输出 + 母线电压」，
不需要测量 `R/L/Ke/Kt/J/B`。对应《飞轮系统控制理论研究报告》第 6 章，
控制律定义见 `doc/现在的控制系统.md`。

标定目标是 [`FlywheelParams`](../../utility/Shooter/FlywheelParams.java) 里的参数
（大球 `FlywheelParams.BIG`、小球 `FlywheelParams.SMALL`），**不再改 `PrototypeShooter` 的静态字段**。

| 编号 | 类名 | Driver Station 名称 | 标定对象 |
|---|---|---|---|
| 6.1 | `StepResponseTuner` | `Tune6.1 StepResponse` | `kS/kff/kp/ki`（模型式初值） |
| 6.3 | `RelayFeedbackTuner` | `Tune6.3 RelayFeedback` | `kp/ki`（临界比例度复核） |
| 6.5 | `FeedforwardTuner` | `Tune6.5 Feedforward` | `kS/kff`（多点稳态拟合） |
| 6.6 | `VelocityFilterTuner` | `Tune6.6 VelocityFilter` | `tauVMax/tauVMin` |
| 6.7 | `BusVoltageFilterTuner` | `Tune6.7 BusVoltageFilter` | `tauBus` |

另有 `ShooterTuningTester`（开环逐级功率扫描，`kS/kff` 的第二种来源）与
`TuningLogger`（CSV 写入工具，不是 OpMode）。

> 第 4 代删除的整定项：`Tune6.2 ZieglerNichols`（与 6.3 方法重复，且其 P-only 实现没有关掉
> `kS`，测出来的不是纯 P）、`Tune6.4 DropDetection`（boost 已从控制律移除）。2026-11-06 起
> 掉速状态机（`inDrop` / `dropEnterFrac` / 电压采样冻结）也删除，掉速相关整定项一并消失。

---

## 通用说明

- **分组**：Driver Station 分组 `Tuning`，名称前缀 `Tune6.x`。
- **调参入口**：FTC Dashboard → Config。每个 OpMode 有自己的分组；被标定的参数在
  `FlywheelParams`（`BIG`/`SMALL`）分组里。
- **原始数据**：每次运行把采样写入 CSV：`<RC 外部存储>/tuning/<前缀>_<时间戳>.csv`，
  用 `adb pull` 取出后配合 `tools/tuning/` 的脚本分析（见 `tools/tuning/PLOT_GUIDE.md`）。
- **CSV 格式**：以 `#` 开头的注释行记录本次实验参数与结论，随后是表头行；
  默认表头 `t,duty,velocity`，新程序用更宽的表头（见各方法「输出」）。
  解析时 `skiprows` 跳过注释行、`comments='#'`（`numpy.loadtxt(..., comments='#')` /
  `pandas.read_csv(..., comment='#')`）。时间轴严格单调，单位秒。
- **速度单位**：统一 `DcMotorEx#getVelocity()` 的 **tick/s**。
- **量化台阶**：实测 **20 tick/s**。任何稳态精度/阈值讨论都必须先报「相当于几个台阶」；
  参数里 `quantStep=20` 就是它的固化值。若换了电机/齿比/滤波，必须重新测台阶。
- **安全**：飞轮转速高，测试前取走球与异物；开环测试务必让 `d0/dHigh/dLow` 在安全转速内。
  所有 OpMode 都可用 **X 键**立即停转。
- **参数污染**：6.1/6.3/6.5 会临时强改被测参数（如置 `kff=0`、`ki=0`）以隔离环节，
  **退出时自动恢复原值**。V4 起这些临时改动作用在 `FlywheelParams` 实例上，
  重启 OpMode 即恢复（若手动在 Dashboard 改过，注意别和整定值混在一起）。

---

## 方法一：阶跃响应辨识（`Tune6.1 StepResponse`）

> 一次给出 `Kd / τ / kff / kp / ki` 的模型式初值，是最快的起点。

### 原理

占空比 `d0` 稳定运行后阶跃到 `dStep`，速度响应近似
`ω(t) = ω0 + Kd·Δd·(1 − e^(−t/τ))`，读 `Kd = Δω/Δd`、`τ = t63.2%`，零极点对消得
`kff = 1/Kd`、`kp = τ/(Kd·τc)`、`ki = 1/(Kd·τc)`，`τc = tRecover/ratio`。

### 可调参数（Dashboard → StepResponseTuner）

| 参数 | 含义 | 建议 |
|---|---|---|
| `d0` | 阶跃前占空比 | 让飞轮稳定转起来（如 0.15） |
| `dStep` | 阶跃后占空比 | 与 d0 拉开明显差距（如 0.25） |
| `settleSec` | 阶跃前稳定时长 | 2 s |
| `recordSec` | 阶跃后记录时长 | 3 s（≥4τ） |
| `steadyWindowSec` | 稳态均值窗口 | 0.5 s |
| `tRecover` | 期望恢复时间 | 0.5 s |
| `ratio` | tRecover/τc | 3~5 |

### 操作

1. Start → 自动跑一次（`d0` 稳定 → 阶跃 → 记录 → 计算）。
2. **A** 重跑，**X** 停转。
3. 把建议值填进 `FlywheelParams.BIG`（注意：`kp` 见下方「注意」）。

### 输出

`Kd (tick/s per duty)`、`τ (s)`、`kff`、`kp`、`ki`。

### 注意

- **模型式给的 `kp` 一定偏小**：相角裕度/内模公式给出 6.1e-4~9.8e-4，
  而量化判据允许到 `0.05/quantStep = 0.0025`（当前 0.00182）。经验做法是
  **在模型值上乘 2~3 倍**，再用 `kp×台阶 ≤ 0.05` 卡上限、用继电法（6.3）交叉验证。
- `τ` 由 63.2% 上升时间估计，噪声大时多跑几次取中位数。
- `dStep` 与 `d0` 太接近、或飞轮未真正转起来，会判 invalid。

---

## 方法二：继电反馈自整定（`Tune6.3 RelayFeedback`）

> 自动产生极限环，比手动加 Kp 安全得多；是 `kp/ki` 的**第二种**整定方式。

### 原理

用继电器代替控制器：`err>0 → dHigh`，`err<0 → dLow`。进入极限环后测幅值 `a`、周期 `Tu`，
则

```text
Ku = 2·Δd / (π·a)        Δd = dHigh − dLow，a = 极限环峰值（相对设定点）
Kp = 0.45·Ku ,  Ki = 0.54·Ku/Tu
```

> **旧数据修正**：2026-10-22 之前本程序写作 `Ku = 4·Δd/(π·a)`，把全幅 `Δd` 当成半幅使用，
> 结果高估 2 倍（样例 `dHigh=0.7, dLow=0.5, a=165` → 旧值 1.543e-3，正确值 7.716e-4）。
> **凡是此前用旧版本跑出的 `Ku/Kp/Ki` 都要除以 2 再参考。**

### 可调参数（Dashboard → RelayFeedbackTuner）

| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 目标转速 | 典型工作转速（2050） |
| `dHigh` / `dLow` | 继电器输出上下限 | 让稳态占空比落在两者之间（如 0.7 / 0.5） |
| `transientSec` | 跳过建立极限环的暂态 | 2 s |
| `measureSec` | 测量时长 | 4 s（≥数个周期） |
| `hysteresis` | 继电器滞环 (tick/s) | 默认 0；噪声大可略加（建议 ≤ 台阶） |

### 操作

1. Start → 自动跑一次（建立极限环 → 测量 → 计算）。
2. **A** 重跑，**X** 停转。
3. 读建议 `Ku/kp/ki`。

### 输出

幅值 `a`、`Tu (s)`、`Ku`、建议 `kp`、建议 `ki`。

### 注意

- `Tu` 有效的前提是稳定极限环（`transientSec` 足够长）。
- `a` 太小时 `Ku` 会被噪声放大，判 invalid：拉开 `dHigh/dLow` 后重试。
- 继电法给的是「临界增益」，对飞轮这种带饱和的执行器通常再取 `0.3~0.45×Ku` 更稳。

---

## 方法三：无模型前馈增益标定（`Tune6.5 Feedforward`）

> `kS` 与 `kff` 的**首选**方法；也是判断「积分是否长期负重」的唯一现场手段。

### 原理

闭环稳定运行时记录稳态占空比 `d_ss`，由
`d_ss(ω*) = kS + kff·ω*`
**多点线性拟合**：斜率 = `kff`、截距 = `kS`。本 OpMode 强制 `kff=0`、`kS=0`，
由 PI 提供全部稳态占空比，避免前馈把读数掩盖掉。

### 可调参数（Dashboard → FeedforwardTuner）

| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 初始目标转速 | 典型工作转速（1700） |
| `velocityStep` | 目标转速步长 | 200 tick/s |
| `settleSec` | 测量前稳定时长 | **≥2 s**（短了会吃到暂态） |
| `windowSec` | 取均值窗口 | 1.5 s |

### 操作

1. **前置**：`kp/ki` 已整定（非 0），否则根本到不了目标转速。
2. Start 后用 **DPad 上/下** 设定目标转速；**每档至少停 2 s**（历史教训：
   0.57~0.62 s 的停留把 `kS` 高估到 0.13）。
3. 速度稳定后按 **A** 记录一点（自动等 `settleSec` 后取 `windowSec` 均值）。
4. 覆盖 **1700 / 2050 / 2250 各 3~4 点**，共 9~12 点。
5. 遥测显示 `Kff (origin fit, only if kS=0)`（过原点斜率）与 `Kff (fit) / kS (intercept)`
   （含截距拟合，**推荐**）。**B** 清空测量点。

### 输出

每点 `ω / d_ss / Kff`、过原点斜率、线性拟合斜率与截距（= kS）。

### 注意

- **单点拟合是高估陷阱**：`kS≈0.03` 时在 1700 tick/s 只用一点过原点拟合，会把 `kff` 高估约 4.6%。
  必须多点、必须拟合斜率。
- 稳态占空比接近 1（饱和）的点无效，降低目标转速重测。
- 若截距为负或 >0.06，说明机械/电气状态变了（脏污、轴承、电池），先查机械再改参数。

---

## 方法四：速度滤波整定（`Tune6.6 VelocityFilter`）

> 第 4 代新增。标定 `tauVMax / tauVMin`。

### 原理

速度反馈要先压掉 **20 tick/s 的量化噪声**，又不能在大误差时拖后腿，因此用误差调度的 τ：

```text
eBand = max(2%·|ω*|, 2·quantStep)
τ = tauVMax                      , |e| ≤ eLoFrac·eBand
τ = tauVMin                      , |e| ≥ eHiFrac·eBand
τ 线性过渡于两者之间
```

本程序一次跑完两段：

1. **稳态段**：在 `steadyTarget` 稳住后统计原始速度标准差 `σ_raw` 与 6 个候选 τ 的 `σ_filt`；
2. **阶跃段**：目标阶跃 `+stepDelta`，记录每个候选 τ 的 **90% 滞后**（相对原始速度的达标时刻）。

反解建议：

```text
σ_target = max(quantStep/2, 0.5%·ω*)        // 允许的残余噪声
r        = σ_target / σ_raw
α        = 2r²/(1+r²)                        // 二阶矩匹配的 EMA 系数
tauVMax  = dt·(1−α)/α
```

### 可调参数（Dashboard → VelocityFilterTuner）

| 参数 | 含义 | 建议 |
|---|---|---|
| `steadyTarget` | 稳态测量转速 | 实际工作点（2050） |
| `stepDelta` | 阶跃幅度 | 300 tick/s |
| `settleSec` | 稳态采样前稳定时长 | 2 s |
| `steadySec` | 稳态采样时长 | 3 s |
| `tau1..tau6` | 6 个候选 τ (s) | 0.008 / 0.012 / 0.016 / 0.020 / 0.025 / 0.030 |
| `lagTolerance` | 90% 滞后判据 | 0.9 |

### 操作

Start 后按 **A** 全自动跑完（两段），**X** 停转。CSV 前缀 `vf06`，列：
`t,velocity,duty,vbus,f1..f6`。

### 输出

`σ_raw`、各候选的 `σ_filt` 与 90% 滞后、建议 `tauVMax`。

### 注意

- 选 τ 的判据是**两段都要看**：只看稳态噪声会选到 30 ms，那会让发射冲击段少看 73% 的跌深
  （τ=30 ms 时冲击末只看到真值的 27%，连饱和都进不去）。
- `tauVMin` 取「约一帧」即可（采样 8.7~9.0 ms → 8 ms）；再小是纯浪费。
  当前取 **12 ms**（约 1.3 帧）。
- 拐点 `eLoFrac/eHiFrac` **由量化台阶决定，不由本程序整定**：
  必须 ≥ 1 个 `quantStep`（20 tick/s）才停得住慢档，所以当前取值是
  **`eLoFrac=0.50`（20 tick/s）、`eHiFrac=1.50`（60 tick/s）**。
  早期取值 `0.30/0.90`（12/36 tick/s）低于一个台阶，实测导致 14% 的时间把 τ 调度到
  `tauVMin`（比 `tauVMax` 更不滤波）——这就是第 4 代早期稳态梳齿的主因，见
  `doc/现在的控制系统.md` §2.15。

---

## 方法五：母线电压滤波整定（`Tune6.7 BusVoltageFilter`）

> 第 4 代新增。标定 `tauBus`（电压归一化用）。

### 原理

电压归一化 `s = clamp(V_nom/V_busFilt, 0.85, 1.25)` **只乘前馈**。`tauBus` 的取舍：

- **太小**：单发发射把母线拉低 3 V，低通把这当成「电池没电」→ `s→1.25` →
  2050 处前馈 0.815→1.02 → 输出又饱和（人造 boost）；
- **太大**：换电池/电量下降后前馈要很久才校正回来。

本程序分两段：

- **A 稳态**：稳定运行 → `σ(V)`、稳态母线、等效源内阻 `R`（`V = V0 − R·I` 最小二乘）、电流峰；
- **B 扰动**：记录 `launchRecordSec` 秒（期间**手动发射一发**），对 5 个候选 `tauBus`
  给出滤波后的最大跌落，并换算成前馈占空比误差

```text
Δduty_ff = kff·ω*·(V_nom/V_filt − 1)
```

判据：取满足 `|Δduty_ff| ≤ maxFfDutyError`（默认 0.005 ≈ 3 个 `kp×台阶`）的**最小** `tauBus`。

### 可调参数（Dashboard → BusVoltageFilterTuner）

| 参数 | 含义 | 建议 |
|---|---|---|
| `targetVelocity` | 测量转速 | 实际工作点（2050） |
| `settleSec` / `sampleSec` | 稳态前稳定 / 采样时长 | 2 s / 3 s |
| `launchRecordSec` | 扰动记录时长 | 1.2 s |
| `maxFfDutyError` | 前馈占空比误差判据 | 0.005 |
| `tau1..tau5` | 5 个候选 `tauBus` (s) | 0.3 / 0.5 / 1.0 / 2.0 / 3.0 |

### 操作

1. **A** → 稳态测量（自动）。
2. **B** → 立即手动发射一发（或用手压住飞轮模拟负载）。
3. **X** 停转。CSV 前缀 `bf07`（稳态）/`bf07d`（扰动），列：
   `t,vbus,current,duty,velocity,f1..f5`。

### 输出

母线均值、原始最大跌落、源内阻 `R`、各候选的滤波跌落与 `|Δduty_ff|`、建议 `tauBus`。

### 注意

- 电压低通是 EMA（`tauBus`），并且**误差出带时暂停吸收读数**（`|e| ≥ eBand`，无状态机）：
  `tauBus` 越大，单发 0.4 s 跌落被压得越平，但「电量缓变」的跟随也越慢。
  本程序量的是前者，定值时要把后者一起算进去。整定时若把目标速度调得远离到位带，
  暂停窗口会拉长，读数靠带内那几拍收敛——这是预期行为，不要因此调小 `tauBus`。
- `R` 明显大于 0.7 Ω 时优先改硬件（新电池/短线束），而不是继续调 `tauBus`。

---

## 推荐整定顺序

1. **准备**：取走异物；确认 `quantStep` 仍是 20（换过电机/齿比就重测）。
2. **稳态前馈**（6.5）：≥9 点重标 `kS`、`kff`。**这一步决定积分是否长期负重，必须先做。**
3. **快速性**（6.1 → 6.3）：6.1 给模型初值（`kp` 乘 2~3），6.3 给临界增益复核，
   最终 `kp` 用 `kp×台阶 ≤ 0.05` 卡上限、用实测 `duty_sd` 定稿。
4. **速度滤波**（6.6）：定 `tauVMax/tauVMin`。
5. **电压滤波**（6.7）：定 `tauBus`（只有打开 `useVoltageNorm` 才有意义）。
6. **发射实测**：带球 10~15 发，看「跌幅 / 恢复时间 / 过冲 / 饱和占比 / 积分回落」。
7. **两种电量对比**：满电 12.9~13.1 V 与放电 ~10.5 V，同目标各 30 s，`duty·V_bus` 差 ≤3%。

---

## 参数速查

| `FlywheelParams` 参数 | 单位 | 来源 |
|---|---|---|
| `kS` | 占空比 | 6.5（拟合截距）、`ShooterTuningTester`（开环截距） |
| `kff` | 占空比/(tick/s) | 6.5（拟合斜率）、6.1（=1/Kd） |
| `kp` | 占空比/(tick/s) | 6.1×2~3、6.3（0.45·Ku）、量化上限 `0.05/quantStep` |
| `ki` | 占空比/(tick/s·s) | 6.1（=1/(Kd·τc)）、6.3（0.54·Ku/Tu） |
| `maxI` | 占空比 | 经验值，防积分饱和（0.3） |
| `tauVMax` / `tauVMin` | s | 6.6 |
| `tauBus` | s | 6.7 |
| `vBusNominal` | V | 电池额定（12.0；满电实测 12.9~13.1） |
| `settleFrac` / `settleFrames` | — | 算出来的，不整定（2% / 10 帧） |
| `iFreezeTimeoutSec` | s | 安全阀（1.5） |

> 全部增益单位为「占空比」。第 4 代的电压归一化只乘前馈，
> 因此**增益量纲不变**，不需要像「整条输出 × V_nom/V_bus」那样换算成 V 量纲。

---

## 附录 A：整定程序覆盖审计（第 4 代交接时）

对照 `doc/现在的控制系统.md` §2.10 的 6 个参数族逐项核对，结论：**6 族全覆盖，无缺口**，
因此本目录在本代**不新增/不改动任何 Tuner**（缺的才补，不缺就不动）。

| §2.10 参数族 | 整定程序 | 状态 | 说明 |
|---|---|---|---|
| ① `kS` / `kff` | 6.5（主）、6.1（`kff=1/Kd`）、`ShooterTuningTester`（开环截距） | 齐 | 必须 ≥2 点；6.5 强制 `kff=0` 让 PI 扛住稳态 |
| ② `kp` / `ki` | 6.1（模型初值 ×2~3）、6.3（临界比例度复核） | 齐 | 上限由 `kp×quantStep ≤ 0.05` 卡死，不靠试 |
| ③ `tauVMax` / `tauVMin` | 6.6 | 齐 | 判据 `σ_filt ≤ max(quantStep/2, 0.5%·ω)` |
| ④ `tauBus` | 6.7 | 齐 | 仅在 `useVoltageNorm=true` 时有意义 |
| ⑤ `eBand` / `settleFrames` | 无程序（**不需要**） | 齐 | 由 `quantStep` 直接算：`eBand=max(settleFrac·ω*, 2·quantStep)`、`settleFrames=10` |
| ⑥ `iFreezeTimeoutSec` | 无程序（**不需要**） | 齐 | 安全阀（1.5 s）；原 `dropEnterFrac` 随掉速状态机删除 |

### 明确不算「缺口」的遗留项

1. **§2.12 的 6 条验收判据没有一键测量程序**（不是整定，是验收）：
   ②稳态 `v_sd/|误差|/duty_sd`、③`|I integral|` 长期值、⑤`isSettled()` 行为、⑥两种电量对比，
   都是「跑 → 记录 → 离线算」。现已由 `FlywheelRun` 提供数据源：数据全部通过 **telemetry**
    打出（键 `target,velocity,duty,voltage,error,current,integral,pTerm,iTerm,ffTerm,vscale,
    iactive,settled`），用 **Dashboard 自带 Logging** 记录导出（不再落自定义 CSV），
    发射恢复用 `tools/tuning/analyze_launch_events.py`，不同电压的 ⑥ 由
    `OpModes/VoltageSweep/` 的电压扫描程序一次扫完（见该目录 `GUIDE.md`）。
2. **`ShooterTuningTester`**（`group=Tests`，464 行，含 `MODE_KS/MODE_KV/step/disturb`）
   与 6.1/6.5 功能重叠，且**只出遥测、不落 CSV**。保留可用（开环 `kS` 截距的第二种来源），
   但不作为正式参数的来源；正式来源以本目录 5 个 Tuner 为准。
3. 6.6 的 `σ_target` 与 6.7 的 `maxFfDutyError=0.005` 都是经验阈值，尚未在真机复核；
   首次使用时把实测值记进 CSV 注释行，便于后续修订。
4. `FlywheelParams.SMALL` 已于 2026-10-27 实机整定完成（`kff 3.8346e-4` / `kp 1.739e-3` /
   `ki 2.50771e-3`），不再需要重标；要复标时把 `FlywheelRun` 的 `paramsSet=SMALL` 写法
   照搬进对应 Tuner 的构造函数即可（只影响加载哪套参数，不影响算法）。

---

## 附录 B：分代整定程序（不在本目录）

历史各代飞轮控制律的整定程序**故意不放在这里**，而是与它们各自的计算器放在一起：
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/pastcalculater/`

| 程序 | 整定对象 | 阶段 |
|---|---|---|
| `TuneGen1Op` | 第 1/2 代 PIDSVA（`Gen1Params`） | ①开环多点 → `kS`(截距)/`kV`(斜率)；②开环阶跃 → `kP`/`kI` |
| `TuneGen3Op` | 第 3 代 boost（`Gen3Params`） | ①②同上给 `kS`/`kff`/`kp`/`ki`；③人为掉速 → 判读并给 `kBoost` 建议 |
| `TuneGen4Op` | 第 4 代（`Gen4Params`） | **不做辨识**，只按 §2.12 闭环验收（稳态量 + 掉速恢复 + `isSettled` 行为） |

为什么分开放：这三份要共用 `utility/pastcalculater` 的 `PastCalculator` / `PastFlywheelDriver` /
`PastTuneSupport`，而本目录（`OpModes/Tuning`）的 5 个 Tuner 只服务**当前代**、
参数唯一出处是 `FlywheelParams`。两套参数容器不要混：`FlywheelParams` 是实例字段（Dashboard 改不动），
`GenXParams` 是全 `static`（Dashboard 能改）。用法见 `pastcalculater/GUIDE.md`。

第 4 代整定仍然走本目录：`kS/kff → 6.5`、`kp/ki → 6.1(+6.3 复核)`、`tauV → 6.6`、`tauBus → 6.7`；
`TuneGen4Op` 只用于「改完参数后确认闭环达标」。

