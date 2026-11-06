# Shooter 工具库目录说明（第 4 代）

## 概述

本目录存放**发射机构（飞轮）**相关的纯计算类。这些类不持有任何硬件引用、不读遥测、不依赖
`Controllers/`（遵守 `FTC_Format.md` 的分层要求），只做「输入 → 输出」的数值计算，
因此可以离线复算与单独验证。

| 文件 | 功能说明 |
|---|---|
| `FlywheelCalculator.java` | 飞轮速度环计算器：`(ω*, ω, V_bus, dt) → duty`。前馈（静摩擦 + 稳态，乘电压归一化）+ 比例（作用在滤波速度上）+ 积分（稳态窄带内累加、掉速冻结）。含误差调度速度滤波、母线电压低通与冻结、抗积分饱和、到位判据 |
| `FlywheelParams.java` | 参数容器（`@Config`）：**一套机器一套完整参数**，内置 `BIG`（大球）与 `SMALL`（小球）两个实例；`applyTo(calculator)` 每帧同步 |

控制器（电机、硬件初始化、遥测、参数面板）位于
- `Controllers/Shooter/PrototypeShooter/PrototypeShooter.java`（单飞轮原型，默认 `FlywheelParams.BIG`）
- `Controllers/Turret/Shooter.java`（炮台内的两个飞轮，各自内嵌一个 `FlywheelParams`）

## 控制律（第 4 代）

```text
e_raw = ω* − ω
ωf    = EMA(ω, τ(|e_raw|))            // τ 在 tauVMax ↔ tauVMin 之间按误差调度
e_f   = ω* − ωf
s     = clamp(V_nom/V_busFilt, 0.85, 1.25)        // 只乘前馈
duty  = sat( s·(kS·sgn(ω*) + kff·ω*) + kp·e_f + ki·∫e_f dt , dutyMin, dutyMax )
```

**与上一代（含 boost）的差别**：`boost`、`dropThreshold`、`dropRateThreshold`、`iZone` 全部删除。
理由（详见 `doc/现在的控制系统.md` §2.5）：发射瞬间 `ff + kp·e` 本来就 >1，boost 在饱和段贡献恒为 0；
它唯一的作用是把退饱和点从 `e_desat=(1−ff)/kp` 推到 `e≤0`，抹掉线性收敛段、结构性地造成 +50~90 tick/s 过冲。

## 要点

- **量纲**：转速 tick/s（`DcMotorEx#getVelocity()`），电压 V，全部增益量纲为「占空比」。
- **`kS·sgn(ω*)`**：静摩擦、同步带预紧、齿槽转矩对应的那份占空比（对抗摩擦、不产生转速）。
  `sgn` 不可省——`ω*=0` 时必须为 0，否则停转指令下飞轮会自己转起来。
  不拆 `kS` 的后果：低速欠给、高速过给，差额全压给积分，表现为积分长期非零。
- **电压只归一化前馈**：乘到整条输出上会把环路增益绑到电池状态（低电压时 `kp×台阶` 抬高 45%），
  且电压滤波延迟会跑进反馈通道（τ=0.3 s 时相角裕度 102°→20°）。
- **电压采样必须冻结**：误差出带（`|e| ≥ eBand`）期间不吸收读数，否则单发 0.4 s 跌落会被
  当成「电池没电」，在恢复期末把前馈放大，人造一次过冲。2026-11-06 起这一层不再用掉速
  状态机，而是无状态地复用误差带判据（`updateBusVoltage`）；`|e| ≥ 3·eBand` 才冻结不够——
  发射窗口结束得太早，模型里过冲只从 +12.0 降到 +10.0，而 `eBand` 门能到 +5.1。
- **积分冻结而非清零**：误差出带期间不累加也不清零，误差回到 `eBand` 内才继续累加；
  这样恢复过程不攒积分（过冲从 +75 降到 +10~20），稳态残差又由原有积分立刻接上。
- **量化台阶**：实测 20 tick/s（`quantStep`）。`kp × 一个台阶` 应 ≤0.05，否则反馈跳一格就让
  占空比大幅跳变（历史故障值 0.381）。遥测里的 `kp*step` 行超过 0.05 会标 `HIGH`。
- **到位判据**：`isSettled()` = `|e_raw| ≤ eBand` 连续 10 帧（`eBand = max(2%·ω*, 2·台阶)`）。
  发射流程必须等它，不要用固定延时。
- **停转语义**：`targetVelocity=0` 时前馈为 0、积分清零、占空比收到 `dutyMin`，飞轮靠 `FLOAT` 滑行。

## 使用方式

### 1. 由控制器调用（常规）

```java
PrototypeShooter shooter = new PrototypeShooter(hardwareMap, telemetry); // 用 FlywheelParams.BIG
shooter.setTargetVelocity(1700);
shooter.update();          // 每帧一次：params.applyTo(calculator) + calculate(target, ω, V_bus) + setPower
```

参数**只在 `FlywheelParams` 里维护**：上半是字段声明（只有含义、没有数值），文件末尾是
「大球飞轮参数块」与「小球飞轮参数块」两个 `static { }`，两套数值分开各写一遍——要改参数只改那两块
（FTC Dashboard 分组 `FlywheelParams.BIG` / `FlywheelParams.SMALL`，改值下一帧生效）。
`FlywheelCalculator` 的参数字段不带工程默认值，必须先 `applyTo(...)` 再 `calculate(...)`，
`Controllers/Turret/Shooter.Params` 也不再重复定义任何控制量。

### 1.1 选大球 / 小球那一套参数

构造函数有两个：

```java
new PrototypeShooter(hardwareMap, telemetry);                       // = (…, "shooter", FlywheelParams.BIG)
new PrototypeShooter(hardwareMap, telemetry, "smallFlywheel", FlywheelParams.SMALL);
```

第二个参数是 **Robot Config 里的电机名**，第四个是要用的参数实例。`PrototypeShooter` 持有的是
**引用**（不是拷贝），所以 Dashboard 改 `FlywheelParams.SMALL.*` 同样下一帧生效。
测试程序里 `OpModes/FlywheelRun.java` 已经把这个选择做成 Dashboard 开关：
`paramsSet` 填 `BIG` / `SMALL`、`motorName` 填电机名；其余程序（`PrototypeShooterTest`、
`ShooterTuningTester`、`OpModes/Tuning/` 下 5 个 Tuner）目前都硬编码 `new PrototypeShooter(hardwareMap, telemetry)`，
**只跑 BIG**。要标小球，照 `FlywheelRun` 的写法把这两行换掉即可：

```java
FlywheelParams chosen = "SMALL".equalsIgnoreCase(paramsSet) ? FlywheelParams.SMALL : FlywheelParams.BIG;
shooter = new PrototypeShooter(hardwareMap, telemetry, motorName, chosen);
```

`SMALL` 已于 2026-10-27 实机整定完成（`kff 3.8346e-4` / `kp 1.739e-3` / `ki 2.50771e-3`，
其余与大球相同），不再是占位拷贝；两份参数现在只有增益三项不同，因为两台飞轮的静态模型
`duty/s = kS + kff·ω` 实测几乎一致。换机构时 `motorReversed` / `speedMax` / `quantStep` 仍需按实际重测。

### 2. 离线复算（显式 dt）

```java
FlywheelCalculator calc = new FlywheelCalculator();
FlywheelParams.BIG.applyTo(calc);
calc.reset(velocity[0]);
for (int i = 0; i < n; i++) {
    double duty = calc.calculate(target[i], velocity[i], vbus[i], dt[i]); // 与实机逐项一致
}
```

- 四参数版本 `calculate(t, ω, V_bus, dt)`：含电压、显式步长，**实机主入口 / 离线复算入口**。
- 三参数 `calculate(t, ω, V_bus)`：显式电压、内部用 `System.nanoTime()` 推进 dt（实机常用）。
- 两参数 `calculate(t, ω)`：无电压（等价于 `useVoltageNorm` 关闭），dt 内部推进。
- **没有** `calculate(t, ω, dt)` 这个版本：Java 不能用「第三参数含义」区分重载。
  回放**无电压列**的旧日志请用 `calculate(t, ω, Double.NaN, dt)`（`NaN` → `s=1`）。
- dt 异常（≤0 或 >1 s）时用 `DEFAULT_DT = 0.02 s` 兜底。

### 3. 重置

从开环标定切回闭环前调用 `reset()` / `reset(currentVelocity)`：清积分与全部瞬态状态，
并用实测转速**播种**滤波器（否则滤波器从 0 爬升会造出一个假的大误差冲击）。

## 注意事项

1. **每帧只调用一次** `calculate(...)`：它依赖内部计时器推进 dt、维护滤波器与积分状态。
2. **调度只能用原始误差**：用滤波后的误差去算 τ 会形成自反馈（误差→τ→误差），出现滞回与慢极限环。
3. **`dV/dt` 掉速检测不可用**：恢复加速度 497~1146 tick/s² × 8.7 ms 每次采样只差 6~10 tick/s，
   小于量化台阶 20，逐帧差分全是噪声。掉速判据改用误差带滞回。
4. **`useVoltageNorm` 默认关闭**：跨电量一致性验证通过后再开。

## 相关实验目录（都不参与正式控制）

| 目录 | 内容 | 自己的说明 |
|---|---|---|
| `OpModes/VoltageSweep/` | 不同母线电压下的适配性测试（负载电机做电气负载、母线电压闭环、四档扫描、分段遥测） | `VoltageSweep/GUIDE.md` |
| `utility/pastcalculater/` | 多代计算器对比实验（Gen1/2 · Gen3 · Gen4）+ 分代整定程序 + 离线回放 | `pastcalculater/GUIDE.md` |

为什么参数分两套：

- **正式参数**只认本目录 `FlywheelParams.java` 末尾两个 `static { }` 块（大球 `BIG` / 小球 `SMALL`），
  它用的是**实例**字段，`@Config` 只对 `static` 字段生效 → **Dashboard 改不动它**。
- 实验用的 `Gen1Params` / `Gen3Params` / `Gen4Params` 是**全 `static`** 的镜像容器，
  Dashboard 能实时改；`Gen4Params.applyTo(FlywheelCalculator)` 与 `FlywheelParams.applyTo(...)` 同签名，
  实验结论要落到正式程序时，仍须手抄回 `FlywheelParams` 的 `static { }` 块。

