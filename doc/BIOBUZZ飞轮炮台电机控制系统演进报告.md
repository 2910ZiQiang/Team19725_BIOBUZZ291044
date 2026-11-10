# 飞轮发射机构（炮台）电机控制系统演进报告
## BluePowerRobotics/Team19725_DECODE → BluePowerRobotics/BIOBUZZ，2026 年 7 月以来的全部电机控制变化与改进

> **检查时点**：2026-10-16（第一至九章）；**2026-10-27 增补第十章**（第 4 代定参、双飞轮、验证工具链、判据收尾）
> **调查对象**：`G:\19725\code_part\project\BIOBUZZ`（第一至九章于 HEAD `202cd58`、分支 `flywheelControlSystemChange` 上取证；第十章于 HEAD `85cdca4` 上取证）、`G:\19725\code_part\project\Team19725_DECODE`（HEAD `328a6c3`）、`G:\19725\code_part\project\19725_DECODE_OffSeasonGame`（HEAD `f8bda9c`）
> **调查方式**：本地仓库只读审计（`git log` / `git show` / `git diff` / 工作区文件全文逐行核对）＋ 实机日志离线复算。**未修改任何被调查仓库的源代码以外的内容；第五章之后的数据分析脚本在工作区 `tools/tuning/out/` 下，未入库。**
> **报告规模**：十章

## 目录

| 章 | 标题 | 内容 |
|---|---|---|
| 一 | 报告范围、方法与证据等级 | 用户需求、调查方法、为何是三个仓库、证据等级约定 |
| 二 | 仓库与提交时间线（取证事实） | BIOBUZZ 分支事实、相关提交清单、两个里程碑提交的规模、未提交工作、分支差异 |
| 三 | DECODE 基线：2026 年 7 月之前的飞轮发射电机控制 | 文件清单、`Shooter.java` / `ShooterAction.java` / `Trigger.java` / `TurretController.java` / `PIDController.java` 全文要点 |
| 四 | BIOBUZZ 新架构：从「单速度环」到「炮台子系统」 | `Turret.java` / `YawController.java` / `Shooter.java` / `PrototypeShooter.java` / `PIDSVAController.java` 全文要点与架构改进清单 |
| 五 | 2026 年 7 月：OffSeasonGame 期间的过渡期电机控制演进 | 7 月编年、飞轮控制器改革、PIDSVA 前史、三仓库能力轨迹、stash 核对 |
| 六 | BIOBUZZ 调参方法学与实测证据 | 五套无模型标定 OpMode、`tools/tuning/` 双语言离线链路、四组实测日志的全部数字 |
| 七 | BIOBUZZ 飞轮台架测量与自动整定工具链 | 三级 Flywheel 台架、依赖关系、配套文档与数据管线 |
| 八 | 总对比与结论建议 | 六阶段演进主线、18 项能力首创归属、8 条结论、P0~P5 建议 |
| 九 | 证据索引、复现命令与产物清单 | 三仓库定位、提交索引、可直接粘贴的复现命令、未提交产物、数字出处对照、未核实事项 |
| 十 | **2026-10-22~27 增补：第 4 代定参、双飞轮与验证工具链** | 第 3→4 代的控制律断点、三个机制的量化验证、参数分家与定稿、六条判据现状、四件验证工具、本轮教训与遗留 |

## 一句话摘要

**2026 年 7 月以来，飞轮发射部分的电机控制系统发生了三次真实断点**：① 7 月在 OffSeasonGame 期间把 DECODE 的「毫秒 dt + 无前馈 + 两套增益硬切换」改成「秒 dt + S/V/A 前馈 + 单一 PIDSVA slot」；② 9 月 30 日在 BIOBUZZ 从零建起「yaw 电机 + 两发射器（各含飞轮 + 俯仰舵机 + 扳机舵机）」的三执行器炮台架构，并修掉了 7 月 yaw 纯开环（到位判据恒真）的缺陷；③ **2026-10-11（`1937409`）`PrototypeShooter` 弃用 PIDSVA，改为自包含占空比律 `duty = Kff·ω* + Kp·err + Ki·∫err + Vboost`，并首次引入掉速检测补偿与 5 套无模型标定 OpMode；2026-10-15（`b182459`）补上掉速检测的 arming 规则（+22/−1）**。

**同时，实测数据（`tools/tuning/out/` 四组日志）诚实地暴露出：截至 `202cd58`，这些参数从未按新建立的方法标定过**——`kp` 默认值比实测反演值低 36 倍（在另一份日志中低 381 倍），`kff` 低约一半，`dropRateThreshold = 8000` 低于实测正常工况最大减速度 `−9250 tick/s²`，`boostMax = 0.4` 在 13 次真实发射中 100% 顶死限幅，且控制律缺少静摩擦项 `kS`。稳态精度本身达标（最好段：速度标准差 0.55 个量化台阶、误差 0.04%）；瓶颈集中在增益标定值与发射后的恢复能力，而后者已被确认为母线电压/电机能力的物理限制。

---
## 一、报告范围、方法与证据等级

### 1.1 用户需求

原始需求（m00001，逐字）：

> 对比BluePowerRobotics\Team19725_DECODE和BluePowerRobotics/BIOBUZZ的相关代码，分析飞轮发射部分（炮台部分）自2026年7月以来的一切关于电机控制系统的变化与改进，生成一份报告，越详细越好。两个代码均于github上可见，可查询相关commit数据，本地亦有代码。

### 1.2 调查方法

1. **本地仓库全量 git 考古**：`git fetch --all --prune` 后对 `git log --all`、`git show --stat`、`git diff` 逐提交取证；
2. **GitHub API 交叉验证**：`https://api.github.com/repos/BluePowerRobotics/<repo>/branches` 与 `/pulls?state=all` 确认分支与 PR 事实；
3. **源码逐行阅读**：所有飞轮/炮台/电机控制相关类全文阅读并记录行号；
4. **设计文档阅读**：`.trae/documents/`、`OpModes/Tuning/Guide.md`、`OpModes/Flywheel/Guide.md`、`utility/PID/Guide.md`、`Controllers/Turret/*Prompt.md`、`FTC_Format.md`；
5. **实测数据**：`tools/tuning/out/*_report.txt` 与 `*_clean.csv` 中的量化结果。

### 1.3 三个仓库，而不仅是两个

用户点名了两个仓库，但**仅对比这两个仓库会得到"DECODE 侧 2026 年 7 月以后完全没有任何变化"的错误结论**。事实是 DECODE 赛季的继续工作在**第三个仓库**：

| 仓库 | 远程 | 2026-07 之后的飞轮/炮台电机控制 | 角色 |
|---|---|---|---|
| `Team19725_DECODE` | `git@github.com:BluePowerRobotics/Team19725_DECODE.git` | **无（0 提交）** | 2025-26 基线（BEFORE） |
| `19725_DECODE_OffSeasonGame` | `git@github.com:BluePowerRobotics/19725_DECODE_OffSeasonGame.git` | 密集（2026-07-01 ~ 2026-08-18） | 过渡期（BRIDGE） |
| `BIOBUZZ` | `git@github.com:BluePowerRobotics/BIOBUZZ.git` | 密集（2026-09-13 ~ 2026-10-15） | 新赛季（AFTER） |

**证据**：
- `Team19725_DECODE`：`git log --all --since=2026-06-01` 返回**空**；GitHub API `/branches` 只返回一个分支 `master @ 94d386c`，该提交 authored 2026-02-02；本地 `master @ 328a6c3` 日期 2026-02-04。即该仓库 2026-02-04 之后完全冻结。
- `19725_DECODE_OffSeasonGame`：`git log --all --since=2026-06-01` 有 40+ 条提交，`db2f117 2026-07-01 炮台限位` 起，`f8bda9c 2026-08-18 UPDATE` 止；另有 stash `5ac7b1d`/`0bdc92f` 日期 2026-09-24。
- `BIOBUZZ`：本体就是 `StarterKit`（upstream `git@github.com:BluePowerRobotics/StarterKit.git`）派生的新赛季工程。

### 1.4 证据等级标注约定

本报告对每条结论标注来源等级，请读者据以判断可信度：

- **【源码】**：逐行读过的代码，附文件路径与行号，最可靠；
- **【Git】**：`git log/show/diff` 直接输出，可靠；
- **【API】**：GitHub REST API 返回，可靠；
- **【文档】**：仓库内设计文档/指南所述，代表**设计意图**而非实测；
- **【实测】**：`tools/tuning/out/` 报告中的量化数据；
- **【推断】**：由上述证据推导的结论，非直接记载。

**注意**：所有参数默认值在代码注释中被反复标注为"占位默认值，必须实机标定"。设计意图 ≠ 已实现 ≠ 已标定，本报告严格区分。

---

## 二、仓库与提交时间线（取证事实）

### 2.1 BIOBUZZ 分支事实

`git ls-remote origin` 显示 `refs/heads`：`26.10.11`、`New-Codes`、`docs/meeting-log-1012`、`fix/sweeper-compile`、`flywheelControlSystemChange`、`master`；另有 `refs/pull/1,2,7,8,9,10`。

GitHub API `/pulls?state=all` 实测结果（【API】）：

| PR | 标题 | 状态 | 创建 | 合并 | head 分支 | 作者 |
|---|---|---|---|---|---|---|
| 10 | docs: 补充 10/12 压敏电阻阈值测试记录 | open | 2026-10-14 04:48 | — | docs/meeting-log-1012 | 2910ZiQiang |
| 9 | 4 | closed | 2026-10-11 15:45 | 2026-10-11 15:53 | 26.10.11 | TerryLee667 |
| 8 | fix: 修正十字键 API 名称，修复 TeamCode 编译失败 | closed | 2026-10-11 08:16 | 2026-10-11 13:44 | fix/sweeper-compile | 2910ZiQiang |
| 7 | 2 | closed | 2026-10-10 07:23 | 2026-10-10 07:23 | master | TerryLee667 |
| 2 | 1 | closed | 2026-10-07 08:06 | 2026-10-07 08:08 | master | TerryLee667 |
| 1 | Update README.md | closed | 2026-10-04 10:50 | — | New-Codes | TerryLee667 |

**关键事实**：飞轮控制系统改造（commit `1937409`）是**在 `flywheelControlSystemChange` 分支上直接开发的，没有走 PR**。其进入主干是通过 `f4333ac 2026-10-11 Merge branch 'flywheelControlSystemChange' into 26.10.11`（【Git】）。这意味着该子系统在合并前**没有经过 code review 流程**。

分支 — 最近提交时间（`git for-each-ref --sort=-committerdate`）：

| ref | 最近提交 | SHA |
|---|---|---|
| master / origin/master | 2026-10-15 | 272398a |
| flywheelControlSystemChange | 2026-10-15 | 202cd58 |
| origin/docs/meeting-log-1012 | 2026-10-14 | 58a1e57 |
| origin/26.10.11 | 2026-10-11 | f4333ac |
| origin/New-Codes | 2026-10-11 | b376856 |
| origin/fix/sweeper-compile | 2026-10-11 | 85f27d8 |
| upstream/master | 2026-10-01 | 5996db3 |
| upstream/26.08.19 | 2026-08-19 | fbb61aa |

### 2.2 与飞轮/炮台电机控制直接相关的提交（BIOBUZZ）

`git log --all --since=2026-09-01` 过滤到炮台/飞轮/PID/Tuning/Parameter 路径（【Git】）：

```
202cd58 | 2026-10-15 | zihengtangtzh25 | Merge branch 'flywheelControlSystemChange' ... into flywheelControlSystemChange
b182459 | 2026-10-15 | zihengtangtzh25 | 小修
f4333ac | 2026-10-11 | Lee            | Merge branch 'flywheelControlSystemChange' into 26.10.11
1937409 | 2026-10-11 | zihengtangtzh25 | 飞轮控制系统修改
d354201 | 2026-10-09 | Suanni-135     | 自动修正
f16a83a | 2026-10-03 | Lee            | 左右互换
5996db3 | 2026-10-01 | Lee            | 线性二元统计，PID小修，状态记录
09d86c2 | 2026-10-01 | Lee            | Update Shooter.md
550ba4d | 2026-10-01 | Lee            | Turret测试
ef4b120 | 2026-09-30 | Lee            | Turret手动
52ad315 | 2026-09-29 | Suanni-135     | 自动迭代（第二代）
9f05ad2 | 2026-09-28 | Suanni-135     | 自动迭代（第二代）+上次的pidsva统一测试代码
06a34df | 2026-09-27 | Suanni-135     | 自动初代
0debb47 | 2026-09-26 | Lee            | delete reference
2936c21 | 2026-09-22 | Lee            | 上赛季数据接入+shooter原型机
0b6f5db | 2026-09-21 | Lee            | 定位接入与底盘瞄准
4ea6e08 | 2026-09-17 | Lee            | 定位
2adec68 | 2026-09-13 | Lee            | 开题
```

### 2.3 两个"里程碑"提交的量化规模

**`1937409`「飞轮控制系统修改」（zihengtangtzh25, 2026-10-11 22:36:09 +0800）**【Git】：

```
10 files changed, 1915 insertions(+)
```
全部为新增（A）：
- `.trae/documents/飞轮控制系统重构方案.md`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Shooter/PrototypeShooter/PrototypeShooter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/PrototypeShooterTest.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/` 全套 7 文件（`DropDetectionTuner.java`、`FeedforwardTuner.java`、`Guide.md`、`RelayFeedbackTuner.java`、`StepResponseTuner.java`、`TuningLogger.java`、`ZieglerNicholsTuner.java`）

**`b182459`「小修」（zihengtangtzh25, 2026-10-15 09:56:55 +0800）**【Git】：

```
13 files changed, 1760 insertions(+), 71 deletions(-)
```
- 修改（M）：`PrototypeShooter.java`、7 个 Tuning 文件、`Parameter/HypParams.java`
- 新增（A）：`tools/tuning/README.md`、`analyze_tuning.m`、`analyze_tuning.py`、`make_samples.py`

`master` 对比 `flywheelControlSystemChange` 的完整差异（`git diff --stat origin/master origin/flywheelControlSystemChange`）**【Git】**：

```
飞轮控制系统重构方案.md                     |  200 ++++++
Controllers/Shooter/PrototypeShooter/PrototypeShooter.java | 244 +++++---
OpModes/BreakBeamTestOp.java               | 109 ----   (删除)
OpModes/IntakeTest.java                    | 280 -------- (删除)
OpModes/MotorTest.java                     | 274 -------- (删除)
OpModes/PrototypeShooterTest.java          |   4 +-
OpModes/ServoTest.java                     | 108 ----   (删除)
OpModes/Tuning/DropDetectionTuner.java     | 221 +++++++
OpModes/Tuning/FeedforwardTuner.java       | 199 +++++++
OpModes/Tuning/Guide.md                    | 232 +++++++
OpModes/Tuning/RelayFeedbackTuner.java     | 215 +++++++
OpModes/Tuning/StepResponseTuner.java      | 201 +++++++
OpModes/Tuning/TuningLogger.java           | 102 +++
OpModes/Tuning/ZieglerNicholsTuner.java    | 234 +++++++
OpModes/开题会议.md                         |   5 +-
Parameter/HypParams.java                   |   7 +
Processors/Sensors/BreakBeamSensor.java    | 204 ----   (删除)
Processors/Sensors/BreakBeamSensor.md      | 161 ----   (删除)
tools/tuning/README.md                     | 112 ++++
tools/tuning/analyze_tuning.m              | 679 ++++++++++++++++
tools/tuning/analyze_tuning.py             | 687 +++++++++++++++++++++++
tools/tuning/make_samples.py               | 119 +++
22 files changed, 3383 insertions(+), 1214 deletions(-)
```

**只取飞轮相关路径**：`8 files changed, 1576 insertions(+), 72 deletions(-)`（【Git】）。

### 2.4 尚未提交的工作（working tree，2026-10-16 检查时点）

**【Git】`git status --short`**：
- 已修改未提交：` M .../Parameter/HypParams.java`（删除了 7 行 `chassisHalfLengthIn`，见 2.5）
- 未跟踪（untracked）：
  - `TeamCode/.../OpModes/Flywheel/`：`Flywheel1OpenLoopLimit.java`、`Flywheel2PidVsOpen.java`、`Flywheel3ControllerVsPid.java`、`FlywheelBench.java`、`FlywheelLog.java`、`Guide.md`
  - `tools/tuning/`：`analyze_dashboard_log.py`、`analyze_launch_events.py`、`check_plot_data.py`、`export_clean_table.py`、`plot_flywheel.py`、`plot_speed.m`、`转速图分析指南.md`

即：**台架测量 OpMode 全套与后处理脚本链仍只在本地工作区，尚未进入任何分支**。文件的 mtime（【Git】文件系统）显示 `OpModes/Flywheel/*` 最新，`FlywheelLog.java` 2026-10-15 19:11、`FlywheelBench.java` 19:29、`Guide.md` 19:33、`Flywheel1/2/3` 均 19:39 —— 即调查时点（2026-10-16 02:16）前 7 小时仍在活跃编写。

对照：`OpModes/Tuning/*` 的 mtime 是 2026-10-13 19:33~19:41，`utility/PID/*` 是 2026-10-11 17:11，`Controllers/Shooter/PrototypeShooter/PrototypeShooter.java` 是 2026-10-13 19:33。

### 2.5 `HypParams.java` 的未提交改动

**【Git】`git diff`**：工作区删除了 7 行，正是 `b182459` 新增的内容：

```java
-    /**
-     * 底盘半长（英寸）：底盘中心到最后端 / 最前端中点的距离。
-     * 用于在「最后端中点坐标」与「底盘中心坐标」之间换算，以及由 intake 目标点反推中心位姿。
-     * todo:实机测量标定（取值取自 master 分支 52ad315）
-     */
-    public static double chassisHalfLengthIn = 9.0;
```

即 `b182459` 加了它，之后又被本地撤销（`+0 -7`）。**该字段与飞轮/炮台无关**（属底盘），但说明 `b182459` 之后工作区仍在变动。

**【源码】`HypParams.java` 中与炮台相关的字段搜索**：只有一条，且是视觉相关（`hivePitchCheckTolDeg`，第 189 行，用于 HIVE 观测的 z′ 退化帧判据），**没有飞轮/炮台的转速、PID 或射程参数**。这是一个重要结构性事实：**新赛季的超参数没有集中到 `Parameter/HypParams.java`，而是分散在各控制器类内（`Turret.shooterBig/small` 的 `Shooter.BIG_PARAMS/SMALL_PARAMS`、`YawController` 的静态字段、`PrototypeShooter` 的静态字段）**。

### 2.6 `OpModes` 目录在各分支的差异

**【Git】`git ls-tree` 对比**：
- `master`/`origin/master`：21 个文件，**无 `Flywheel/`、无 `Tuning/` 子目录**；有 `FlywheelTest.java`、`PrototypeShooterTest.java`、`ShooterTuningTester.java`，另有 `BreakBeamTestOp.java`、`IntakeTest.java`、`MotorTest.java`、`ServoTest.java`
- `origin/26.10.11` 与 `flywheelControlSystemChange`：24 个文件，**含 `Tuning/` 全套 6 个 java + `Guide.md`**，但 `Flywheel/` 仍不存在于任何 ref
- `origin/New-Codes`：18 个（有 `BallCounterTestOp.java`）
- `origin/docs/meeting-log-1012`：17 个（最精简）

---

## 三、DECODE 基线：2026 年 7 月之前的飞轮发射电机控制

这一节是"变化前"的参照物。所有内容来自 `G:\19725\code_part\project\Team19725_DECODE\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\`。

### 3.1 相关文件清单

| 文件 | 行数 | 职责 |
|---|---|---|
| `controllers\shooter\Shooter.java` | 127 | 单个弹射飞轮的 PID 控制器 |
| `controllers\shooter\ShooterAction.java` | 149 | 双飞轮动作编排（roadrunner Action） |
| `controllers\Trigger.java` | 50 | 扳机舵机（左右各一个 180° 舵机） |
| `controllers\turret\TurretController.java` | 169 | **空类** + 弹道解算 `TurretCalculator` |
| `controllers\turret\model\TurretInfo.java` | 28 | 解算结果 DTO |
| `utility\PIDController.java` | 51 | PID 内核 |
| `utility\solvepoint\SolveShootPoint.java` | — | 射程—转速线性回归 + 射点几何 |

### 3.2 `controllers\shooter\Shooter.java`（127 行）全文要点【源码】

类注释（第 17 行）逐字：`//单个弹射飞轮的PID控制器`

**静态参数（全部硬编码在类内，分两套）**：

```java
public static int SpeedTolerance = 35;
public static double BlockPower = -0.3;
public static double degreePertick = 0;
public static double MinPower = 0.1;
public static double k_p = 0.01;   public static double k_i = 0.2;   public static double k_d = 0.05;
public static double max_i = 1;
//这里的small指的是小三角发射，而不是更小的PID
public static double MinPower_small = 0.3;
public static double k_p_small = 0.011; public static double k_d_small = 0.08; public static double k_i_small = 0.05;
public static double max_i_small = 0.6;
```

**构造**（`Shooter(HardwareMap, Telemetry, String motorName, boolean ifReverse)`）：

```java
setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
direction 由 ifReverse 决定;
pidController = new PIDController(k_p, k_i, k_d, max_i);
previous_time = System.currentTimeMillis();
```

**核心闭环 `boolean shoot(int targetSpeed)`（第 69 行起）**：

```java
if(targetSpeed < 750)  pidController.setPID(k_p, k_i, k_d);            else pidController.setPID(k_p_small, k_i_small, k_d_small);
// 注释说 small 是小三角，实际代码是 >=750 用 small
if(targetSpeed == 0){ setPower(0); pidController.reset(); return true; }
current_time = System.currentTimeMillis();
current_encoder = shooterMotor.getCurrentPosition();
current_speed = shooterMotor.getVelocity(AngleUnit.DEGREES);
double dt = (current_time - previous_time);
if (dt <= 0) dt = 1;                       // 防止除零
Power = pidController.calculate(targetSpeed, current_speed, dt);
if(targetSpeed>0 && targetSpeed<750)      Power = Range.clip(Power, MinPower, 1);
else if(targetSpeed>0 && targetSpeed>=750) Power = Range.clip(Power, MinPower_small, 1);
shooterMotor.setPower(Power);
return Math.abs(targetSpeed - current_speed) < SpeedTolerance;
```

`void block()` → `shooterMotor.setPower(BlockPower)`（−0.3，反转压住球）。

**该实现的全部结构性缺陷（后续所有改造正是针对这些）**：

1. **`dt` 单位是毫秒，而速度单位是度/秒** —— 微分项 `(error - previousError)/dt` 与积分项 `error*dt` 都被放大了约 1000 倍；`if (dt <= 0) dt = 1` 在真正 dt≈16ms 时并不触发，所以这不是"保护"而是**隐含的标度约定**，使 `k_i=0.2` 这类数字具有某种一致但不可移植的意义。
2. **误差肢解式双参数集**：以 750 °/s 为硬开关切换整套增益，且**注释与代码语义相反**（注释说 small 指小三角发射，代码却是"目标转速 ≥750 用 small 参数"）。
3. **无前馈项**：稳态转速靠积分项硬撑，因此 `k_i=0.2` 相对 `k_p=0.01` 大 20 倍 —— 典型的"用积分顶住稳态、牺牲响应"结构。
4. **积分项无 I 分离、无抗饱和回算**：`PIDController` 只做 `clamp(integral, ±maxI)`，在误差大时仍一直积分；`setPID` 里还有个特例 `if(kI==0) integral=0;`。
5. **PID 参数只在构造时传入一次，运行中不刷新 slot**（DECODE 版本没有 slot 机制）。
6. **`BRAKE` 零功率行为**：飞轮惯性大，急停时电机反向制动，对机械与电流都有冲击。
7. **速度单位度/秒**：编码器为 tick，每 tick 对应的度数被丢掉了（`degreePertick` 声明了但未使用于换算）。
8. **无掉速检测、无补偿**：发球瞬间负载突变只能靠积分慢慢补。
9. **`SpeedTolerance=35` 到位判据过于宽松**（相对 700~835 °/s 的工作点约 4~5%），且只在 `shoot()` 里用一次快照速度判断，无滞回、无稳定时间要求。
10. **无遥测分量分解**：只有 `InstanceTelemetry` 打印 "current PID" 为 "BIG"/"SMALL" 字符串。

### 3.3 `controllers\shooter\ShooterAction.java`（149 行）【源码】

双飞轮（左右各一），关键常量：

```java
public static int speed_block = -400;
public static int speed2_2 = 700;   public static int speed25_25 = 725;
public static int speed3_3 = 750;   public static int speed25_55 = 835;  public static int speed35_55 = 875;
public static int targetSpeed_low = 665;  public static int targetSpeed_high = 835;
public static int speedDescend = 150;
public int low_speed_threshold = 350;
public static long waitTime = 200;  public static long ShootTime = 2800;
public static int WindowSize = 3;
```

构造：`shooter_Left = new Shooter(hardwareMap, telemetry, "shooterMotor1", true); shooter_Right = new Shooter(hardwareMap, telemetry, "shooterMotor2", false);` 并对两侧速度各套一个长度 3 的 `MeanFilter`。

`setShootSpeed(int)` 用**滤波后均值**判断到位：

```java
left  = (Math.abs(FilterLeft.getMean()  - TargetSpeedRC) < Shooter.SpeedTolerance);
right = (Math.abs(FilterRight.getMean() - TargetSpeedRC) < Shooter.SpeedTolerance);
```

**要点**：到位判据加了 3 点均值滤波，是一种土法去抖；但转速表（目标值）是硬编码的离散档位（665/700/725/750/835/875），**不是按距离连续解算的**；`ShootTime=2800ms` 是固定发球时长。

### 3.4 `controllers\Trigger.java`（50 行）【源码】

```java
public static double TRIGGER_OPEN_POS  = 0.405;
public static double TRIGGER_CLOSE_POS = 0.17;
public static double TRIGGER_PUSH_POS  = 0.45;
public static double TRIGGER_PUSH_TIME_MS = 200;
public static double TRIGGER_OPEN_TIME_MS = 200;
Servo triggerServo_LEFT  = hardwareMap.get(Servo.class, "trigger_left");   // REVERSE
Servo triggerServo_RIGHT = hardwareMap.get(Servo.class, "trigger_right");  // FORWARD
```

`open()` 用一个 `PUSH` 布尔与 `System.currentTimeMillis()` 做 **PUSH_POS ⇄ OPEN_POS 的 200ms 交替**，即"推球—回位"的往复动作靠主循环反复调用来产生；`close()` 直接置 CLOSE_POS。类注释保留了 `//todo 确定一个能平滑摩擦的扳机值`。

**要点**：扳机是纯开环舵机位置 + 毫秒计时器状态机，**没有到位反馈、没有与飞轮/俯仰的联锁**。

### 3.5 `controllers\turret\TurretController.java`（169 行）【源码】

```java
public class TurretController { }          // ← 第 13-14 行：完全空的类

class TurretCalculator {
    public static class Param {
        public static double g = 9.8;
        public static Point3D target = new Point3D(0,0,0);
        public static double turretHeight = 0;
    }
    public List<TurretInfo> solveSpeed(double theta, Pose2d, PoseVelocity2d) { ... }
    public List<TurretInfo> solveTheta(double speed, ...) { ... }
}
```

`solveSpeed` 的四次方程系数：

```java
double A = Math.cos(theta)/Math.sin(theta);      // cot θ
a4 = (A*A*g*g)/4.0;  a3 = 0;
a2 = (A*A*h*g - vt_sq);
a1 = -2*d;           a0 = (A*A*h*h - R0_sq);
// R0_sq = x0*x0+y0*y0;  d = x0*vtx+y0*vty;  vt_sq = vtx*vtx+vty*vty
MathSolver.solve4(a4, a3, a2, a1, a0);
// 每个 t>0 的根：v = (h + 0.5*g*t*t)/(t*sinθ);  phi = atan2(yt, xt)
```

`solveTheta`：

```java
a4 = g*g/4.0; a3 = 0.0; a2 = vtx*vtx+vty*vty-speed*speed+g*h; a1 = 2*(x0*vtx+y0*vty); a0 = x0*x0+y0*y0+h*h;
Vx = x0/t+vtx;  Vy = y0/t+vty;  Vz = h/t + 0.5*g*t;
phi   = Math.atan2(Vy, Vx);
theta = Math.atan2(Vz, Math.sqrt(Vx*Vx+Vy*Vy));
```

**决定性结论**：DECODE 的 `turret` 包中**没有任何电机控制**——`TurretController` 是空类，其余是纯弹道学解算（给定 θ 求 (v,φ,t)，或给定 v 求 (θ,φ)）。**没有炮台平转电机、没有俯仰执行器、没有扳机**。整个 DECODE 赛季的发射电机控制**只有 `Shooter.java` 里的速度 PID 一个环节**。

### 3.6 `utility\PIDController.java`（51 行）【源码】

```java
public class PIDController {
    private double kP, kI, kD;
    private double integral, previousError;
    private double maxI = 1;
    public double calculate(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        integral += error * dt;
        if (integral > maxI) integral = maxI; else if (integral < -maxI) integral = -maxI;
        double derivative = (error - previousError) / dt;
        previousError = error;
        return kP * error + kI * integral + kD * derivative;
    }
    public void reset() { integral = 0; previousError = 0; }
    public void setPID(double kP, double kI, double kD) { ...; if(kI==0) integral=0; }
    public void setMaxI(double maxI) { ... }
}
```

**特征**：无 I 分离、无饱和回算、无输出限幅（限幅在调用方 Shooter 里做）、无前馈、无分量诊断、`@Config` 注解在第 4 行但静态字段并不存在（`kP/kI/kD` 是实例私有，FTC Dashboard 无法直接整定，只能通过 `Shooter` 的静态字段间接改）。

### 3.7 瞄准与射程—转速映射【源码】

`utility\solvepoint\SolveShootPoint.java`（部分）：

```java
public static double r1 = 48*Math.sqrt(2);  r2 = 60*Math.sqrt(2);  r3 = 72*Math.sqrt(2);
public static double r4 = 100;  r5 = 200;
public static double k = 50/(r3 - r1);
public static int solveShootSpeed(double distance){
    // 分段线性回归
    speed = Math.toIntExact(Math.round(1.081183337 * distance + 586.1640982));   // 一段
    speed = Math.toIntExact(Math.round(2.010562318 * distance + 540.4157073));   // 另一段
}
public static double solveBLUEShootHeading(Pose2d poseRC){ ... atan2(y+70.47, x+70.47); }
public static double solveBLUEShootDistance(Pose2d poseRC){ ... hypot(y+72, x+72); }
public static Pose2d solveBLUEShootPoint(Pose2d poseRC, double r) { ... }
```

**要点**：两条直线 `1.0812·d + 586.16` 与 `2.0106·d + 540.42`，是**第一代线性回归**；目标点写死 `+72`/`+70.47`（蓝方球门），分段阈值与挡位（`r4=100`、`r5=200`）硬编码。这些数字在 BIOBUZZ 的 `ShooterTuningTester` 中被**自动化回归取代**（见第七节）。

### 3.8 DECODE 基线的总体评价

DECODE 的发射系统是**"一个速度 PID + 硬编码转速档位 + 开环舵机扳机 + 离线弹道公式"**：

- 电机控制层面只有 1 个闭环（飞轮速度），1 个控制器类，1 个 PID 内核；
- 参数以 750 °/s 为界分两套，且注释与代码矛盾；
- 时间基准混乱（毫秒当秒用）；
- 无前馈、无抗饱和回算、无掉速处理、无零功率策略考虑、无分量遥测；
- 炮台（平转/俯仰）根本没有电机控制代码。

这正是后续所有改造的起点。

---

## 四、BIOBUZZ 新架构：从"单速度环"到"炮台子系统"

BIOBUZZ 在 `Controllers/Turret/` 下把炮台拆成明确的三个部件，并在 `utility/PID/` 下建立了一套新的控制器库。

### 4.1 目录与文件（含行数）【Git】

```
Controllers/Turret/
  Turret.java              129   ← 炮台整体编排
  YawController.java       187   ← 平转机构位置闭环（PIDSVA）
  Shooter.java             275   ← 单个发射器：飞轮速度环 + 俯仰舵机 + 扳机舵机
  AutoAimPrompt.md           6   ← 自动瞄准需求（未实现）
  TurretPrompt.md           29   ← 炮台模块需求
  TurretTestPrompt.md       14   ← 测试 OpMode 需求
Controllers/Shooter/
  Shooter.md                 1   ← 空
  PrototypeShooter/PrototypeShooter.java  303  ← 台架原型控制器（自包含占空比控制）
utility/PID/
  PIDSVAController.java    197
  SlotConfig.java          117
  MotorPIDCore.java        182
  PIDController.java       129
  SVAController.java        49
  Guide.md                 182
OpModes/Tuning/            (6 java + Guide.md)
OpModes/Flywheel/          (5 java + Guide.md, 未跟踪)
```

### 4.2 `Controllers/Turret/Turret.java`（129 行）全文结构【源码】

类注释（第 12-14 行）逐字定义了机械结构：

> **机械结构**：一个大球发射器、一个小球发射器，以及二者共用的平转机构。每个发射器含一个飞轮、一个俯仰舵机和一个扳机舵机；两发射器朝向始终平行，由同一个平转电机驱动，因此共享同一个平转角 yaw。

职责划分（第 16-21 行）：
- `YawController` —— 平转机构的位置闭环
- `Shooter` —— 单个发射器的飞轮速度环、俯仰与扳机
- `Turret` 本类 —— 把上层下达的目标分发给上述两个部件，并汇总到位状态

**器件名（第 50-56 行）**：

```java
private static final String BIG_MOTOR         = "shooterBig";
private static final String BIG_PITCH_SERVO   = "pitchBig";
private static final String BIG_TRIGGER_SERVO = "triggerBig";
private static final String SMALL_MOTOR         = "shooterSmall";
private static final String SMALL_PITCH_SERVO   = "pitchSmall";
private static final String SMALL_TRIGGER_SERVO = "triggerSmall";
```

**超参数（第 40-46 行）**：

```java
public static Vector2D RelPosBig   = new Vector2D(0, 0);   // 大球发射器中心在地面投影（英寸），"当前尚未使用"
public static Vector2D RelPosSmall = new Vector2D(0, 0);   // 小球同
```

**构造（第 66-74 行）**：一次构造 `YawController` 与两个 `Shooter`，GPS 参数分别传 `Shooter.BIG_PARAMS` / `Shooter.SMALL_PARAMS`，末尾 `stop()`。

**统一的每帧入口（第 106-114 行）**：

```java
public FireReady update(double yaw, double pitchBig, double pitchSmall,
                        int speedBig, int speedSmall,
                        boolean triggerBig, boolean triggerSmall) {
    boolean yawReady   = yawController.update(yaw);
    boolean bigReady   = shooterBig.update(speedBig, pitchBig, triggerBig);
    boolean smallReady = shooterSmall.update(speedSmall, pitchSmall, triggerSmall);
    return new FireReady(yawReady && bigReady, yawReady && smallReady);
}
```

**关键的联锁设计**：`FireReady(yawReady && bigReady, yawReady && smallReady)` —— **朝向不对时即便飞轮与俯仰就绪也不允许发射**。DECODE 完全没有这一层。

`FireReady` 是静态内部类（第 81-91 行），两个 `public final boolean big/small`。

第 23-24 行引用工程规范：`按 FTC_Format.md 的约定，update 会各调用一次其直接依赖的部件的 update 函数；上层每帧只需调用本类的 update。` 该规范在 `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\FTC_Format.md`（185 行），第 247-249 行逐字：

> ### 4. update函数嵌套
> 特别地，若一个功能需要每帧调用，则应当提供一个update函数，这个函数不仅需要更新这个功能，还应当调用**一次**该功能直接依赖的每个功能的update函数。最终，在主程序和actions中，应当**每帧调用**直接依赖的所有功能的update函数，从而按依赖关系层层更新。

第 26-28 行保留自动瞄准 todo：

> **todo**：未来实现自动瞄准——重载 update，输入车辆位姿与是否允许发射，由两个发射器的相对位置反解平转角与射程，再按标定的射程-转速模型解出飞轮速度，平转角取两个发射器所需角度的平均（指向劣角），并在两机构均到位时自动释放扳机。

### 4.3 `Controllers/Turret/YawController.java`（187 行）全文要点【源码】

```java
public class YawController {
    private static final String MOTOR_NAME = "yawMotor";
    public static double yawRange   = Math.PI / 2;   // 平转机构最大角度绝对值（弧度）
    public static double tickPerRad = 1000.0;         // 每弧度对应 tick 数（占位）
    public static double yawZeroTicks = 0.0;          // "机械上 yaw = 0 时"的编码器读数
    public static double kP = 0.005;  kI = 0.0;  kD = 0.0;
    public static double kS = 0.05;   kV = 0.0002;  kA = 0.0;
    public static double maxI = 1.0;  iZone = 200.0;
    public static double outputMin = -1.0;  outputMax = 1.0;
    public static double onTargetTolerance = 0.02;    // 单位：弧度
    public final DcMotorEx motor;
    private final PIDSVAController controller = new PIDSVAController();
    private final SlotConfig slot = new SlotConfig();
}
```

第 20-22 行注释点明了该仓库的核心设计动机（见 4.5 的对比）：`复用同一个 slot：每帧用 withXxx 刷新参数后 resetSlot（见 SlotConfig 类注释）`。

第 22 行附近注释：`零位标定：yawZeroTicks 为"机械上 yaw = 0 时"的编码器读数`。

时间基准（第 107-108 行）：

```java
private static double now() { return System.nanoTime() / 1e9; }   // ← 秒
```

`update(double yaw)`（第 117 行起）：

```java
double currentTime = now();  double dt = currentTime - lastTime;
if (dt <= 0 || dt > 1.0) dt = ...;          // 异常 dt 兜底
targetYaw = Math.max(-yawRange, Math.min(yawRange, yaw));       // 软限位钳制
double targetTicks  = yawZeroTicks + targetYaw * tickPerRad;    // 角度 → 编码器位置
double currentTicks = motor.getCurrentPosition();
// 每帧刷新参数，使 Dashboard 上的改动即时生效
slot.withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI).withIZone(iZone)
    .withKS(kS).withKV(kV).withKA(kA).withOutputLimits(outputMin, outputMax);
controller.resetSlot(slot);
power = controller.calculate(targetTicks, currentTicks, dt, false);   // false = 位置闭环
motor.setPower(power);
onTarget = Math.abs(targetTicks - currentTicks) < onTargetTolerance * Math.abs(tickPerRad);
```

第 139 行注释解释 `false` 的含义：

> PIDSVA 位置闭环：SVA 前馈的参考速度/加速度由控制器内部按 setpoint 变化率推导（见 PID/Guide.md）

其余 API：`stop()`（`motor.setPower(0)`）、`getCurrentYaw()` → `(motor.getCurrentPosition() - yawZeroTicks) / tickPerRad`、`getTargetYaw()`、`isOnTarget()`、`getPower()`、`setTelemetry()`（打印 Yaw target/current/error，单位度，第 178-181 行）。

**相对 DECODE 的跨越**：DECODE 的炮台**根本没有电机**；这里从零建立了「弧度域目标 → 软限位 → tick 域位置 → PIDSVA 位置闭环 → 到位判据（0.02 rad ≈ 1.15°）」的完整链路。

### 4.4 `Controllers/Turret/Shooter.java`（275 行）全文要点【源码】

文件头 javadoc 逐字：`单个发射器控制器：飞轮速度闭环 + 俯仰舵机 + 扳机舵机`。

**参数从静态字段改为实例类**——这是本文件最重要的一个结构改动。类内 javadoc 明确给出理由：

> 把 BIG/SMALL 机械参数放进 `Params` 实例类 `BIG_PARAMS`/`SMALL_PARAMS` 而不是静态裸字段，**否则两个实例会共用同一份参数**。

```java
public static class Params {
    public double kP = 0.0004, kI = 0.0002, kD = 0.0;
    public double maxI = 1.0, iZone = 3000.0;
    public double kS = 0.0, kV = 0.0002, kA = 0.0;
    public double outputMin = -1.0, outputMax = 1.0;
    public double pitchMin = 0.0, pitchMax = 1.0;
    public int    speedMax = 3000;
    public double triggerOnPos = 1.0, triggerOffPos = 0.0;
    public boolean motorReversed = false;
    public double speedTolerance = 100.0;    // 单位 tick/s
    public double pitchSettleTime = 0.25;    // 单位 s
}
public static Params BIG_PARAMS   = new Params();
public static Params SMALL_PARAMS = new Params();
```

全部字段注释均标注"占位默认值，必须实机标定"。

**单位约定（javadoc）**：飞轮速度用 `DcMotorEx#getVelocity()` 默认单位 **tick/s**；俯仰与扳机用舵机位置 **[0,1]**。

**字段与构造（第 133 行）**：

```java
public final DcMotorEx motor;
public final Servo pitchServo;
public final Servo triggerServo;
private final PIDSVAController controller = new PIDSVAController();
private final SlotConfig slot = new SlotConfig();

Shooter(HardwareMap, Telemetry, String motorName, String pitchServoName, String triggerServoName, Params params) {
    motor.setDirection(params.motorReversed ? REVERSE : FORWARD);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // 软件速度闭环：关闭固件速度环
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // 飞轮惯性大，停转后让其自然滑行
    controller.withSlot0(new SlotConfig().withKP(...).withKI(...).withKD(...)
        .withMaxI(...).withIZone(...).withKS(...).withKV(...).withKA(...).withOutputLimits(...));
}
```

`name`（motor 设备名）被用作**遥测标题前缀**，javadoc 说明理由：`避免大球/小球两个实例的条目互相覆盖`。

时间基准与 DECODE 一致改为秒：

```java
private static double now() { return System.nanoTime() / 1e9; }
```

`update(int speed, double pitch, boolean release)`（第 170 行起）：

```java
double dt = ...;
if (dt <= 0 || dt > 1.0) dt = 0.02;      // 标称环周期兜底
targetSpeed = Math.max(0, Math.min(params.speedMax, speed));
targetPitch = Math.max(params.pitchMin, Math.min(params.pitchMax, pitch));
pitchServo.setPosition(targetPitch);
triggerServo.setPosition(release ? params.triggerOnPos : params.triggerOffPos);
// 俯仰目标变化 > 1e-6 时重启 pitchCommandTime（用于俯仰稳定计时）
slot.withKP(...) ... ;  controller.resetSlot(slot);   // 每帧刷新，Dashboard 改动即时生效
double currentVelocity = motor.getVelocity();
double power = controller.calculate(targetSpeed, currentVelocity, dt, true);  // true = 速度闭环
motor.setPower(power);
```

javadoc 解释 `true`：`目标转速作为前馈速度项，由 kS/kV 换算成基础功率`。

`stop()`：`targetSpeed=0`、扳机置 off、`setPower(0)`、`controller.reset()`。

`isReady()`（第 230 行）：

```java
boolean speedReady = targetSpeed > 0 && Math.abs(targetSpeed - motor.getVelocity()) < params.speedTolerance;
boolean pitchReady = now() - pitchCommandTime >= params.pitchSettleTime;
return speedReady && pitchReady;
```

**相对 DECODE 的关键改动**：
1. 一实例一参数集（`Params`），彻底消除 DECODE 那种"两个飞轮共用静态字段"的隐患；
2. 除速度环外**接管了俯仰舵机与扳机舵机**，形成"飞轮 + 俯仰 + 扳机"三位一体的发射器；
3. 到位判据从"瞬时速度差 < 35 °/s"升级为"**速度到位 AND 俯仰稳定时间 ≥ 0.25s**"；
4. `BRAKE` → `FLOAT`（飞轮惯性大，自然滑行）；
5. 输出单位从"功率，带 MinPower 下限钳制"变为"功率，`outputMin/outputMax` 对称限幅"（`outputMin=-1.0`，即允许反转制动）；
6. 遥测按实例前缀区分，分量可分解。

### 4.5 `Controllers/Shooter/PrototypeShooter/PrototypeShooter.java`（303 行）【源码】

这是**第二个、也是更新的**飞轮控制实现，面向台架原型机，电机名固定 `private static final String MOTOR_NAME = "shooter";`。

**它放弃了 PIDSVAController，改为自包含控制器，且输出从"功率"改为"占空比"**。javadoc 给出控制律：

```
duty = Kff·ω* + Kp·err + Ki·∫err + Vboost
err  = ω* - ω
duty = sat(duty, dutyMin, dutyMax)
```

并说明：`所有增益的量纲均为「占空比」，因此不读取电压传感器，calculate 直接返回占空比`；单位统一为 `DcMotorEx#getVelocity()` 的 tick/s。

**静态可调参数（全部含单位注释）**：

```java
public static double kff  = 0.0002;    // (占空比)/(tick/s)，≈目标转速下的稳态占空比/目标转速
public static double kp   = 0.00005;   // (占空比)/(tick/s)
public static double ki   = 0.00002;   // (占空比)/(tick/s·s)
public static double maxI = 0.3;       // I 项上限（占空比）
public static double iZone = 500.0;    // I 项生效节点 (tick/s)
public static double dropThreshold = 300.0;      // 掉速判定阈值 (tick/s)
public static double dropRateThreshold = 8000.0; // 掉速速率阈值 (tick/s²)
public static double kBoost = 0.0002;  // 掉速补偿增益
public static double boostMax = 0.4;   // 补偿上限（占空比）
public static double boostDurationSec = 0.3;  // 补偿持续时长
public static double dutyMin = 0.0;  public static double dutyMax = 1.0;
public static boolean motorReversed = false;
public static int targetVelocity = 0;
```

**I 项语义在 javadoc 中被显式形式化**（这是 DECODE 完全没有的工程化表述）：
- 上限：对积分项 `Ki·∫err` 限幅到 ±`maxI`，**饱和时把内部积分回算为 ±maxI/Ki**（抗饱和）；
- 生效节点：仅当 `targetVelocity != 0` 且 `|err| < iZone` 时才累加积分；
- 清零节点：① 停机（targetVelocity == 0）；② 误差带外（|err| ≥ iZone）；③ 触发掉速补偿时。

**`calculate(double targetVel, double currentVel)`（第 153 行）**：

```java
double error = targetVel - currentVel;
double dVelocity = (currentVel - previousVelocity) / dt;
previousVelocity = currentVel;
// ---- 掉速检测（带 arming）----
boolean dropDetected = dropArmed && targetVel != 0 && (error > dropThreshold || dVelocity < -dropRateThreshold);
if (dropDetected) { boostActive = true; boostTimer = 0.0; integral = 0.0; }
if (boostActive) {
    boostTimer += dt;
    if (boostTimer >= boostDurationSec || error <= 0) boostActive = false;   // 恢复到位或超时退出
}
// ---- 积分 ----
boolean iActive = targetVel != 0 && Math.abs(error) < iZone;
if (iActive) integral += error * dt; else integral = 0.0;
double iTerm = ki * integral;
if (iTerm > maxI) { iTerm = maxI; if (ki != 0) integral = maxI / ki; }
else if (iTerm < -maxI) { iTerm = -maxI; if (ki != 0) integral = -maxI / ki; }
// ---- 前馈 + PI + 补偿 ----
double boost = boostActive ? Math.max(0.0, Math.min(kBoost * error, boostMax)) : 0.0;
lastFFTerm = kff * targetVel;  lastPTerm = kp * error;  lastITerm = iTerm;  lastBoostTerm = boost;
double out = lastFFTerm + lastPTerm + lastITerm + lastBoostTerm;
if (out > dutyMax) out = dutyMax;  if (out < dutyMin) out = dutyMin;
this.duty = out;  return out;
```

**掉速检测的"arming"机制**（javadoc 逐字要点）：补偿只有在误差曾回到 `dropThreshold` 以内（进入稳态）之后才允许触发；目标一变立即重新解除 arming，因此 **0→转速的启动斜坡不会被误判为掉速**（否则 boost 会一直挂着并造成超调）。

`update()`：`motor.setPower(calculate(targetVelocity, motor.getVelocity()));`

标定辅助：`setOpenLoopPower(double power)` 绕过闭环直接给功率（供 ShooterTuningTester 标定 kS/kV 用），`resetController()` 重置积分/微分状态（开环切回闭环前调用）。

遥测打印 FF/P/I/boost 四项分解。

javadoc 引用《飞轮系统控制理论研究报告》第 6 章的**黑箱辨识**方法：阶跃响应辨识 `kff/kp/ki`，稳态速度波动定 `dropThreshold`，正常工况最大减速度定 `dropRateThreshold`，`kBoost` 先取 2~5 倍 `kp`。

### 4.6 `utility/PID/PIDSVAController.java`（197 行）【源码】

```java
public double calculate(double setpoint, double measurement, double dt, boolean velocityLoop) {
    SlotConfig cfg = slots.get(currentSlot);
    if (cfg == null) throw new IllegalArgumentException("Slot not configured: " + currentSlot);
    double refVelocity;
    if (velocityLoop) refVelocity = setpoint;                       // 速度闭环：setpoint 即参考速度
    else refVelocity = Double.isNaN(previousSetpoint) ? 0.0 : (setpoint - previousSetpoint) / dt;  // 位置闭环：由 setpoint 变化率推导
    double refAcceleration = Double.isNaN(previousRefVelocity) ? 0.0 : (refVelocity - previousRefVelocity) / dt;
    previousSetpoint = setpoint;  previousRefVelocity = refVelocity;
    double error = setpoint - measurement;
    if (Math.abs(error) < cfg.iZone) {          // I 分离
        integral += error * dt;
        if (integral > cfg.maxI) integral = cfg.maxI;
        if (integral < -cfg.maxI) integral = -cfg.maxI;
    } else integral = 0;
    double derivative = (error - previousError) / dt;
    previousError = error;
    double pid = cfg.kP*error + cfg.kI*integral + cfg.kD*derivative;
    double sva = cfg.kS*Math.signum(refVelocity) + cfg.kV*refVelocity + cfg.kA*refAcceleration;
    double output = pid + sva;
    if (output > cfg.outputMax) output = cfg.outputMax;
    if (output < cfg.outputMin) output = cfg.outputMin;
    // 分量留存供遥测
    lastOutput = output; lastPTerm = cfg.kP*error; lastITerm = cfg.kI*integral; lastDTerm = cfg.kD*derivative;
    lastSTerm = cfg.kS*Math.signum(refVelocity); lastVTerm = cfg.kV*refVelocity; lastATerm = cfg.kA*refAcceleration;
    return output;
}
```

javadoc 里最关键的一句设计声明（第 107-108 行）：

> SVA 前馈所需的参考速度/参考加速度由控制器内部根据 setpoint 的变化率推导，**不使用实测速度，避免前馈与微分项相互抵消**。

其他 API：`withSlot0(SlotConfig)`、`withSlot(int, SlotConfig)`、`setSlot(int)`（切换 slot 并 `reset()`）、`resetSlot(SlotConfig)`（**仅更新 slot 0，不重置状态**）、`resetSlot(int, SlotConfig)`、`reset()`（清积分/微分/setpoint 历史）、以及 7 个分量 getter。

**注意 `resetSlot` 的语义**：它明确不 reset 积分与微分（控制器状态保存在 `controller` 内，不受影响），这正是 `Shooter`/`YawController` 能"每帧刷新参数"的前提。

### 4.7 相对 DECODE 的架构性改进清单

| # | 维度 | DECODE（2026-07 前） | BIOBUZZ（2026-10） |
|---|---|---|---|
| 1 | 炮台电机数 | 0（只有飞轮速度环） | 3 类执行器：yaw 电机 + 2 俯仰舵机 + 2 扳机舵机 |
| 2 | 参数存储 | 类内静态裸字段，两实例共用 | `Params` 实例 / `SlotConfig`，一实例一份 |
| 3 | 时间基准 | `currentTimeMillis`，毫秒当秒 | `System.nanoTime()/1e9`，秒 |
| 4 | 速度单位 | `getVelocity(AngleUnit.DEGREES)` | `getVelocity()` tick/s |
| 5 | 前馈 | 无 | S/V/A 前馈，参考速度可由 setpoint 变化率推导 |
| 6 | 积分抗饱和 | 仅 `clamp(±maxI)` | I 分离 + 饱和回算（`integral = maxI/ki`） |
| 7 | 控制器结构 | 1 个 PID 内核 | 双形态：PIDSVA（速度/位置双模）+ 自包含占空比控制器 |
| 8 | 零功率行为 | `BRAKE` | `FLOAT`（飞轮） |
| 9 | 到位判据 | 瞬时 \|Δv\| < 35 °/s | 速度容差 100 tick/s **AND** 俯仰稳定 0.25 s **AND** yaw 到位（联锁） |
| 10 | 掉速处理 | 无 | 带 arming 的掉速检测 + 限时 boost 补偿 + 触发时清积分 |
| 11 | 参数集切换 | 750 °/s 硬开关两套，注释与代码矛盾 | 独立实例参数，无运行中切换 |
| 12 | 输出限幅 | 分段 `MinPower` ∈ {0.1, 0.3}，上限 1 | `outputMin/outputMax` 对称可配（−1..1）；占空比版 `dutyMin=0` |
| 13 | 遥测 | "BIG"/"SMALL" 字符串 | 分量分解 P/I/D/S/V/A 或 FF/P/I/boost，前缀区分实例 |
| 14 | 每帧 update 约定 | 无统一约定 | `FTC_Format.md` 第 247-249 行规定逐层调用一次 |
| 15 | 整定方式 | 手改静态字段 + 重下 | slot 每帧刷新 → Dashboard 实时生效 |

---
## 五、2026 年 7 月：OffSeasonGame 期间的过渡期电机控制演进

> 本文档分析对象：`G:\19725\code_part\project\19725_DECODE_OffSeasonGame`（下称 **OffSeasonGame**，DECODE 2025-26 赛季的休赛期延续分支）。
> 时间窗：`git log --all --since=2026-06-20 --until=2026-09-01`，共 41 个提交（含 merge），作者为 Lee / Suanni-135 / JIMMYZ / zihengtangtzh25。
> 全部数据来源为仓库内 `git log` / `git show` / `git diff` 与工作区文件实际内容；行号以工作区当前文件（含未提交改动）为准，标注 `HEAD` 处则指已提交版本。

---

### 5.1 7 月窗口的提交编年（哪些真的动了炮台/飞轮电机控制）

本节先按「是否真的改到发射/炮台电机控制」把 7 月这段密集提交分成**真实电机控制改动**与**无关 churn**。过滤路径：
`Controllers/Turret/**`、`Controllers/MotorExamples/**`、`rubbishbin/**`、`OpModes/Actions/**`、`utility/HypParams.java`（炮台限位参数所在）、`utility/LinearInterpolation/**`（7-24 的线性回归标定）。

#### 5.1.1 真实改动炮台/飞轮电机控制的提交

| sha | date | author | subject | 触摸到的电机控制文件（+/-） |
|---|---|---|---|---|
| `db2f117` | 2026-07-01 | Lee | 炮台限位 | `Controllers/Turret/Turret.java` (+37/-10)；`Controllers/Turret/turner/TurretDegreeController.java` (+30/-53)；`OpModes/Actions/GoToShootingAreaAction.java` (+48/-1)；`OpModes/OffseasonDECODE.java` (+1)；`utility/HypParams.java` (+?/-?) |
| `5fd10fb` | 2026-07-16 | Lee | 0: 40 AM | `Controllers/Turret/Shooter/Shooter.java` (+5/-5)；`Controllers/Turret/Shooter/ShooterSmallTester.java` **新增 50 行**；`Controllers/Turret/Shooter/Shootertester.java` (-1)；`utility/HypParams.java` (+/-) |
| `c1fbe6e` | 2026-07-16 | Lee | trigger | `Controllers/Turret/Turret.java` (+8/-2) |
| `f21fe62` | 2026-07-19 | Lee | 0：40AM | `Controllers/Turret/Shooter/Shooter.java` (+6/-5)；`Controllers/Turret/Turret.java` (+26)；`Controllers/MotorExamples/MotorPIDTuning.java` (+2/-2)；`Controllers/MotorExamples/MotorSVATuning.java` (+2/-2)；新增 `OpModes/ShooterIntaketester.java` (+107) |
| `6e818e3` | 2026-07-19 | Lee | debug | `Controllers/Turret/Shooter/Shooter.java` (+3/-3)；`OpModes/ShooterIntaketester.java` (+3/-3) |
| `5b51033` | 2026-07-19 | Lee | 大修1 | `Controllers/Turret/TurretTest.java` (+8/-3)；`Controllers/Turret/TurretTickTest.java` (+1/-1)；`Controllers/Turret/turner/TurretDegreeController.java` (+7/-7) |
| `a03bfa3` | 2026-07-19 | zihengtangtzh25 | 调参 | `Controllers/Turret/Turret.java` (+1/-1)；`TurretTest.java` (+3/-1)；`TurretTickTest.java` (+5/-2)；`TurretDegreeController.java` (+11/-9) |
| `3bac6fb` | 2026-07-21 | Lee | 大修3 | `Controllers/Turret/Turret.java` (**+97/-14**)；`Controllers/Turret/turner/TurretDegreeController.java` (+5/-5)；`Controllers/Turret/Shooter/Shooter.java` (+3)；`utility/HypParams.java` (+25/-25 附近) |
| `81ed236` | 2026-07-21 | Lee | debug | `Controllers/Turret/Turret.java` (+8/-14) |
| `1cbb291` | 2026-07-21 | Lee | 大修4 | `Controllers/Turret/Shooter/Shooter.java` (+2/-2)；`Controllers/Turret/Turret.java` (+9/-6) |
| `53b595b` | 2026-07-21 | Lee | 大修4 | `Controllers/Turret/Shooter/Shooter.java` (+2/-2)（同提交还含两个 CSV 标定文件） |
| `f70c7eb` | 2026-07-22 | Lee | 大修5 | `Controllers/Turret/Turret.java` (+3/-2) |
| `d716183` | 2026-07-22 | Lee | 大修6 | `Controllers/Turret/Turret.java` (+2/-3) |
| `2d05336` | 2026-07-22 | Lee | 大修7 | `Controllers/Turret/Turret.java` (+22/-6)；`Controllers/Turret/Shooter/Shooter.java` (+3) |
| `2b790c4` | 2026-07-23 | Lee | 大修8 | `Controllers/Turret/turner/TurretDegreeController.java` (+2/-2) |
| `b03818b` | 2026-07-23 | Lee | 大修9 | `Controllers/Turret/Turret.java` (+13) |
| `a8d86a5` | 2026-07-24 | Suanni-135 | 大修10 | `Controllers/Turret/Shooter/Shooter.java` (+1/-1) |
| `9d3c8d0` | 2026-07-24 | Suanni-135 | 设置小三角自动发射 | **新增 `Controllers/Turret/Turret_2.java`（+549 行，全新文件）** |
| `431f220` | 2026-07-24 | Lee | 线性回归 | `Controllers/Turret/Turret.java` (**+46/-98**)；新增 `OpModes/DataTester.java` (+255)；新增 `utility/LinearInterpolation/LinearInterpolator.java` (+108) |
| `f8bda9c` | 2026-08-18 | Suanni-135 | UPDATE | `Controllers/Turret/Turret.java` (+3/-3)；`Controllers/Turret/Turret_2.java` (+1/-1)；`OpModes/Actions/ShootAction_Far.java` (+7/-1)；`OpModes/AutoActionFar_Blue_1.java` (+46/-62) |

**关键观察**：炮台/飞轮电机控制的主战场是 `Controllers/Turret/`。`Turret.java` 在 7-01 到 7-24 之间被反复修改 14 次，其中 `大修3`（+97）和 `线性回归`（+46/-98）是两次结构性改动；`TurretDegreeController.java` 在 7-01、7-19（两次）、7-23 各有改动，但自 7-23 后仅剩 `stash@{0}`（`5ac7b1d`，2026-09-24）中的参数改动（见 5.5）。

#### 5.1.2 与该窗口相关但**不碰电机控制**的提交（churn / 其他子系统）

- **底盘与定位**：`e58a2c8`、`f3e0e9c`、`5dcc5df`、`7129cd8`、`2403e49`、`a03bfa3`（同时含 RoadRunner 调参器 `ManualFeedbackTuner*.java`）、`5247f71`、`1cbb291`、`f70c7eb`、`d716183` 的多数文件。
- **视觉/帧率/主程序**：`31be96c`（帧率检测&耗时分析，+53）、`f8a4949`（帧率计算）、`d5205a1`（debug主程序）、`bc579e3`（add 主程序）、`320139d`（Berthing Procedure）。
- **颜色传感器**：`937617f`（2026-06-30 ColorSensor，+268）、`b546c15`（07-13 colorsensor2，+74），与发射电机无关。
- **Auto 动作组合**：`9d3c8d0`、`431f220`、`f8bda9c` 同时改 `OpModes/Actions/**` 和 `AutoActionFar_Blue_1.java`，属于「自动发射流程」而非电机控制律本身。
- 纯 merge 提交（无内容改动）：`8d938af`、`6dcef8c`、`a94c2e0`、`36f77b4`、`b88b327`、`16c1524`。

#### 5.1.3 工作区未提交状态（截至本次分析）

`git status --porcelain` 在 OffSeasonGame 工作区有 **3 个已修改未提交文件**，全部属于电机控制且都是**调试性降级**（把双电机改成单电机、注释掉右轮）：

- `Controllers/MotorExamples/MotorSVATuning.java` (+7/-7)：`motorName` 由 `"motorL"` 改为 `"motor"`，`followerMotor` 相关 6 处全部被注释。
- `Controllers/Turret/Shooter/Shooter.java` (+11/-11)：`shooterR` 字段、`hardwareMap.get(..., "motorR")`、配置、`setPower`、`getPowerR()/getSpeedR()` 全部注释；`shooterL` 的硬件名由 `"motorL"` 改为 `"motor"`。
- `Controllers/Turret/Shooter/Shootertester.java` (+1/-1)：`telemetry.addData("rightV",shooter.getSpeedR())` 被注释。

> 判定：这是**单电机台架调试**状态（硬件层只有一台 `motor`），不是设计变更；已提交历史里 Shooter 始终是双电机 L/R 版本。
>
> ⚠ 修正：截至本次分析的复核（`git status --porcelain` + `git diff --stat HEAD`），工作区未提交文件为 3 个，`git diff --stat HEAD` 汇总为 `3 files changed, 19 insertions(+), 19 deletions(-)`，与上表逐文件一致（MotorSVATuning 7+/7-、Shooter 11+/11-、Shootertester 1+/1-）。

#### 5.1.4 `Turret.java`（497 行）vs `Turret_2.java`（549 行）：不是新旧版本，而是两条并行路线

`git diff --no-index Turret.java Turret_2.java` = `1 file changed, 103 insertions(+), 51 deletions(-)`，两者 1-17 行 import 完全相同（仅 `Turret_2` 多一行 `import org.firstinspires.ftc.teamcode.utility.BST.BSTsolver;` `Turret_2.java:17`），但**类名不同**（`public class Turret` `Turret.java:19` / `public class Turret_2` `Turret_2.java:21`），构造器分别在 `Turret.java:104` 与 `Turret_2.java:97`。

关键区别在**弹道解算方式**：
- `Turret.java`：线性回归射速 `SPEED_A = 10; SPEED_B = 800;`（`:30-31`）→ `int speed = (int)(SPEED_A * distance + SPEED_B);`（`:266`），仰角**硬固定** `private static final double FIXED_YAW = 45;`（`:33`，`shoot()` 里 `rotate_to(targetRoll, FIXED_YAW)` `:282`）。
- `Turret_2.java`：改用 **BST 二元搜索弹道求解器** `private BSTsolver bstSolver;`（`:30`），构造时 `bstSolver = new BSTsolver();`（`:105`），发射时按机器人速度与相对位移求解 `solution = bstSolver.predict(vx, vy, relativeDx, relativeDy);`（`:263`、`:314`），带初速版本 `bstSolver.predict(...)`（`:247`、`:299`）。全仓 `grep BSTsolver` 只在这两个位置族出现。

使用情况（`grep` 全 `TeamCode\src\main\java`）：`Turret_2` 的实例化/引用集中在 `OpModes/Actions/AutoActionFar_Blue_1.java`、`ShootAction_Far.java`、`GoToStopPose_Far.java` 等**远距离自动**动作；`Turret` 被 `TurretTest`、`OffseasonDECODE`、`OffseasonDECODE_Blue` 等**手动/近距**路径使用。
⇒ 判定：**两者并存、按距离分线，而不是迭代替换**；`Turret_2.java` 是 2026-07-24 **新建**（`9d3c8d0`，+549/-0，提交主题「设置小三角自动发射」），`Turret.java` 在同一天仍有 `a8d86a5` 的 1+/1- 改动，8-18 的 `f8bda9c` 两者**同时**被改（`Turret.java` 3+/3-、`Turret_2.java` 1+/1-）——若其中一个是废弃件，不可能在收官提交里同时维护。

#### 5.2.4 `TurretDegreeController.java` 完整 API 与「舵机不可回读」这一硬约束

接 5.2.2 前文，该文件 301 行（HEAD `f8bda9c` 为 263 行，**多出的 38 行尚未提交**）的尾部方法（`:265-301`）补齐了对外 API：

```java
265: public double[] get_angle()      { return new double[]{currentRoll, currentYaw}; }
273: public double[] getTarget()      { return new double[]{targetRoll, targetYaw}; }
281: public double[] getAimError()    {
282:     double rollError = Math.abs(normalizeAngle(targetRoll - currentRoll));
283:     double yawError  = Math.abs(targetYaw - currentYaw);
284:     return new double[]{rollError, yawError};
285: }
292: private double normalizeAngle(double angle) {
293:     angle = angle % 360.0;
294:     if (angle > 180.0) angle -= 360.0; else if (angle < -180.0) angle += 360.0;
299:     return angle;
300: }
```

`TurretDegreeController.md:56-80` 列出的 `rotateTo / rotateRollTo / rotateYawTo / update / setTargetRoll / setTargetYaw / setTarget / resetAngles / reachedTarget / get_angle / getTarget / getAimError / stop` 现已全部在代码中落地（`stop` 未在 1-301 行内检到公开实现，标记**未核实**）。

**核心工程约束（源码注释原文，`:214-215`）**：
```java
//这里的yawServo.getPosition()读取到的就是你给它set的targetposition。
//舵机不会以任何电气方式实时读取自己的位置并返回。所以currentYaw即yawTargetServo
```
即 yaw 分支 `currentYaw = yawServo.getPosition() / YAW_SERVO_POSITION_PER_DEGREE + YAW_ANGLE_MIN;`（`:216`）**在数学上恒等于目标值**，`reachedTarget()` 的 yaw 判据（`ANGLE_TOLERANCE = 0.5`，`:30`）因此**永远为真**——yaw 是纯开环，`getAimError()` 的 `yawError` 恒为 0。这是 7 月炮台的真实能力边界，也是 BIOBUZZ 另立 `Controllers\Turret\YawController.java`（`import utility.PID.*` `:9-10`）的直接动因。

#### 5.2.5 7 月的瞄准数学与垃圾箱死代码

`utility\TurretAimingCalculator.java`（177 行）：角度域、输入输出全为度，走最短路径。核心 `:74-98`：

```java
74: turretWorldX = robotX + turretOffsetX*cos(thetaRad) - turretOffsetY*sin(thetaRad);
75: turretWorldY = robotY + turretOffsetX*sin(thetaRad) + turretOffsetY*cos(thetaRad);
78: deltaX = tarX - turretWorldX;   79: deltaY = tarY - turretWorldY;
83: aimAngleRad = Math.atan2(deltaY, deltaX);
86: currentTurretAngleRad = thetaRad + thetaTurretRad;
89: rotateRad = aimAngleRad - currentTurretAngleRad;
92: normalizeRadian(rotateRad);
95: deltaTheta = Math.toDegrees(rotateRad);
98: return Math.round(deltaTheta * 1000000.0) / 1000000.0;   // 6 位小数
```

**仰角拟合是明确的占位实现，不可引用为真实标定值**：
```java
101: //TODO:机播豆包给我把这玩意写了。。。。。
103: public static double ToBoardDegree(double targetDistance){
104:     return 10086*targetDistance;//todo角度制，实际测试后修改拟合函数;
105: }
107: public static double calculateTurretAxisAngle(double theta, double targetHeight){
110:     return TurretAimingCalculator.ToBoardDegree(targetHeight / Math.tan(Math.toRadians(theta)));
111: }
```
`10086` 是占位常数（对比 5.1 里 `431f220` 真实拟合出的 `speed = 12.7*distance + 627`，以及类默认 `SPEED_A=10/SPEED_B=800`）——**三者互不相同**，说明线性回归结果**没有回写到类字段默认值**，Dashboard 下发的值才是生效值。

死代码核对（`grep` 全 `TeamCode\src\main\java`，共 17 处命中）：`TurretAimingCalculator` 只被 `rubbishbin\TurretAimController.java`（`:7` import、`:11` 字段、`:31` 构造、`:33` `new TurretAimingCalculator(0.0, 0.0)`）与 `rubbishbin\TurretBoardController.java`（`:7`、`:12`、`:37`、`:39`）引用；后者又只被 `OpModes\GearTester.java` 实例化（`:9` import、`:13` 字段、`:21` `board = new TurretBoardController(hardwareMap, "servo")`）。
`git log -- rubbishbin/*` 最后几次触及为 `f65e1c5`(2026-04-02 `Move legacy Limelight files to rubbishbin`)、`7d0c915`(04-21)、`ecd1a3f`(04-28 `修改并重构Turret`)、`af7fab6`(04-10 `搬运VoltageOut及相关示例；移动PID SVA Controller`)、`54f2754`(05-27)。
⇒ `rubbishbin\TurretAimController.java`（160 行）与 `rubbishbin\TurretBoardController.java`（174 行）是 **2026-04~05 就被退役的早期版本，7 月窗口内零改动**，不是 7 月的"被取代版本"；`TurretAimingCalculator` 因此**不在任何存活发射链路上**（存活链路 = `Controllers\Turret\Turret.java` + `TurretDegreeController.java` + `Shooter\Shooter.java`）。

---

### 5.2 飞轮控制器：从「两套静态增益 + 毫秒 dt」到「单一 PIDSVA + 秒 dt + 电压前馈」

#### 5.2.1 DECODE 基线（BEFORE）：`Team19725_DECODE\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\controllers\shooter\Shooter.java`（127 行）

结构特征：
- 参数写在类字段上，**两套增益按目标转速硬切换**，切换阈值是 750（`Shooter.java:70`）：
  ```java
  public static int SpeedTolerance = 35;          // :20
  public static double BlockPower = -0.3;         // :22
  public static double MinPower = 0.1;            // :36
  public static double k_p = 0.01, k_i = 0.2, k_d = 0.05;   // :37-39
  public static double max_i = 1;                 // :41
  public static double MinPower_small = 0.3;      // :44
  public static double k_p_small = 0.011, k_d_small = 0.08, k_i_small = 0.05;  // :45-47
  public static double max_i_small = 0.6;         // :48
  ```
  ```java
  if(targetSpeed < 750){ ... pidController.setPID(k_p,k_i,k_d); ... }   // :70-74
  else { ... pidController.setPID(k_p_small,k_i_small,k_d_small); ... } // :75-79
  ```
- **dt 用毫秒**：`current_time = System.currentTimeMillis();`（`:88`），`double dt = (current_time - previous_time);`（`:91`），`if (dt <= 0) dt = 1;`（`:92`）。dt 单位是 **ms** 且下限被强制为 1。
- 反馈量是 `getVelocity(AngleUnit.DEGREES)`（`:90`），即 **°/s**。
- 输出直接是 **power**，用 `Range.clip(Power, MinPower, 1)` / `Range.clip(Power, MinPower_small, 1)` 限制（`:96`、`:99`），然后 `shooterMotor.setPower(Power)`（`:101`）。**没有电压换算、没有 S/V/A 前馈**。
- 零速处理是「`targetSpeed == 0` 直接 `setPower(0)` + `pidController.reset()`」（`:81-86`）；另有 `block()` 输出 `BlockPower = -0.3`（`:109-112`）。
- 零功率行为 `BRAKE`（`:56`），编码器 `STOP_AND_RESET_ENCODER` 后 `RUN_WITHOUT_ENCODER`（`:54-55`）。
- PID 内核 `utility\PIDController.java`（51 行）：`calculate(setpoint, measurement, dt)` 里 `integral += error * dt;`（`:24`）、`double derivative = (error - previousError) / dt;`（`:31`），**不做任何 dt 归一化/单位换算**——dt 传 ms 进去，则积分项被放大 1000 倍、微分项被缩小 1000 倍，全靠 k_i/k_d 手调补偿。`maxI` 默认 1（`:9`）。

#### 5.2.2 July 版本：`OffSeasonGame\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\Controllers\Turret\Shooter\Shooter.java`（147 行，工作区已含未提交改动）

逐条对照——**变化点**：

1. **控制器替换**：不再自研 PID 内核，改用 `Controllers.MotorExamples.PIDSVAControllers.PIDSVAController` + `SlotConfig` + `VoltageOut`（`:10-12`、`:42-44`）。
2. **增益只剩一套**，且代码里明确写了原因（这是 7 月最重要的设计判断，原文保留）：
   ```java
   //两套PID是不对的，当时是因为没有速度前馈项，所以强行用了两套PID参数，一套针对低速，一套针对高速。
   // 现在有了速度前馈项，理论上不需要两套了，所以统一。
   config.withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
           .withKS(kS).withKV(kV).withKA(kA)
           .withOutputLimits(outputMin, outputMax);   // Shooter.java:104-108
   ```
   参数为 `kP=0.08, kI=0, kD=0, maxI=1.0, kS=1.23, kV=0.0049, kA=0.0`（`:17-29`），另有 `outputMin=-14.0 / outputMax=14.0`（`:31-33`）、`VelocityTolerance=20`（`:35`，DECODE 是 35）。
3. **dt 改为秒**（`/1000.0`），且首帧固定 0.02 s：
   ```java
   long now = System.currentTimeMillis();
   double dt = lastUpdateTime == 0 ? 0.02 : (now - lastUpdateTime) / 1000.0;  // :111-112
   ```
4. **闭环为速度环**：`outputVoltage = controller.calculate(targetVelocity, currentVelocity, dt, true);`（`:115`，末参 `true` = `VelCycle`）。`currentVelocity = shooterL.getVelocity()`（`:114`）不再显式指定 `AngleUnit.DEGREES`（默认 tick/s）。
5. **输出经电压换算**：`power = voltageOut.getVoltageOutPower(outputVoltage);`（`:116`）再 `setPower`（`:118`）——控制器输出量纲是**伏特**（-14…14），而非 power。
6. **限幅与钳位交给 SlotConfig 的 outputLimits**，不再 `Range.clip(Power, MinPower, 1)`；**`BlockPower` / `block()` / `MinPower*` / `SpeedTolerance` 全部消失**。
7. **零速短路**同样保留为显式分支：`if(targetVelocity == 0){ shooterL.setPower(0); return; }`（`:97-103`）。
8. **零功率行为由 BRAKE 改成 FLOAT**：`setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT)`（`:64`）；DECODE 是 `BRAKE`。
9. 硬件名：工作区未提交版本为**双电机 `motorL`/`motorR`，现被整体注释、只留单电机 `"motor"`**（`:37-38`、`:57-58`、`:66-72`、`:99-100`、`:119`、`:126-129`）。已提交历史（`HEAD`）里仍是 `motorL`/`motorR` 双电机、`shooterR.setDirection(REVERSE)`。
10. 缺少的能力：**没有 drop detection、没有 boost、没有 lastPTerm/lastITerm 分量诊断**（该版本 `PIDSVAController` 本身也没有，见 5.3.2）。

> 结论：July 的关键跃迁是「**毫秒 dt → 秒 dt**」「**power 域 → 电压域 + SVA 前馈**」「**按转速切换两套 PID → 单一 PIDSVA**」三点；前馈项 `kS=1.23 V`、`kV=0.0049 V/(tick/s)` 即 7 月台架拟合出来的值（`kS` 量级 1.23 V 符合 FTC 飞轮静摩擦的常见范围）。

#### 5.2.3 台架测试 OpMode（硬件名与测试流程）

- `Shooter\Shootertester.java`（工作区 44 行，`@TeleOp(name = "Shootertester", group = "Tests")`，`:8`）：直接 `new Shooter(hardwareMap, telemetry)`（`:18`），`targetVelocity` 初值 **1000**（`:12`），dpad↑/↓ 每次 **±100**（`:21-26`），dpad←/→ 每次 **∓/+500**（`:27-32`），循环里 `shooter.update()`（`:35`）并显示 `leftV`（`:37`）；`rightV` 一行在未提交改动中被注释（`:38`）。遥测同时送 Driver Station 与 FTC Dashboard（`MultipleTelemetry`，`:15`）。
- `Shooter\ShooterSmallTester.java`（50 行，**7-16 新增**，`@TeleOp(name = "Shooterstester", group = "Tests")`，`:10`）：**不经过 PIDSVA**，纯开环——硬件 `motorL` / `motorR`（`:18-19`），`STOP_AND_RESET_ENCODER` → `RUN_WITHOUT_ENCODER`（`:23-29`），`shooterL` FORWARD / `shooterR` REVERSE（`:31-33`），dpad↑ 双电机 `setPower(0.7)`、dpad↓ `setPower(0)`（`:38-45`）。用途是**验证机械/接线/双轮反向**，与"小三角"无关（类名 `Small` 指小三角发射口径）。
- `Turret\TurretTest.java`（工作区 118 行，`@TeleOp(name = "TurretTest", group = "Tests")`，`:9`）：手动调参台。步长 `YAW_STEP = 5.0°`、`SPEED_STEP = 50`，扣扳机（`left_trigger > 0.5`）时减半为 `2.5°` / `25`（`:22-25`、`:54-56`）；角度限幅 `ROLL_MIN/MAX = ∓180.0`、`YAW_MIN = 45.0 / YAW_MAX = 65.0`、`SPEED_MIN/MAX = 0 / 800`（`:28-33`）；左摇杆 x 控 roll，灵敏度 `roll += stickX * 2.0`（`:61`）；dpad↑↓ 控 yaw、←→ 控转速；`right_trigger` 控预载 `preSpeed = (int)(gamepad1.right_trigger * HypParams.maxPreSpeed)`（`:87`）；A 键发射（`:90`）；主循环调用 `turret.update(roll, yaw, speed, shouldShoot, preSpeed)`（`:93`）。遥测含 `turret.turretDegreeController.yawServo.getPosition()`（`:101`）——说明 `TurretTest` 直接反射内部舵机位置。
- `Turret\TurretTickTest.java`（工作区 89 行，`@TeleOp(name = "TurretTickTestS", group = "Tests")`，`:10`）：**标定硬件换算系数**用。A 键在「电机 tick 模式」/「舵机 position 模式」间切换（`:34-36`）；电机侧 dpad↑↓ 每次 `±80` tick、←→ 每次 `∓1°`，靠 `roll.setTargetPosition((int) targetPosition_r); roll.setMode(RUN_TO_POSITION); roll.setPower(0.25);`（`:50-52`）走 REV 内置位置模式，B 键按下时算 `MOTOR_TICK_PER_DEGREE = roll.getCurrentPosition() / currentDegree_r`（`:53`）；舵机侧 dpad↑↓ 每次 `±0.05` position，B 键松开时算 `SERVO_POSITION_PER_DEGREE = targetPosition_y / currentDegree_y`（`:71`）。初值 `SERVO_POSITION_PER_DEGREE = 0.0385`（`:15`）—— 该数字后来固化为 `TurretDegreeController.YAW_SERVO_POSITION_PER_DEGREE = 0.0385`（`turner\TurretDegreeController.java:55`）。

---

### 5.3 PIDSVA 的「前史」：OffSeasonGame 的 `PIDSVAControllers`（复数包）vs BIOBUZZ 的 `utility\PID`

> 这是回答「BIOBUZZ 的 PIDSVA 是不是凭空冒出来的」的关键证据。结论：**不是**。PIDSVA 这一整套（`PIDSVAController` + `SlotConfig` + `SVAController` + `VoltageOut`）在 OffSeasonGame 里**先于 7 月就已存在**，7 月只是把它接到了炮台上。

#### 5.3.1 时间线（git 证据）

`git log --since=2026-06-01 -- '*PIDSVA*' '*SlotConfig*' '*PIDController*' '*MotorExamples*'` 在 OffSeasonGame 得到：

| sha | 日期 | 主题 | 改动文件 |
|---|---|---|---|
| `7657b48` | 2026-06-10 | 修改了 SlotConfig 中一个可能导致 bug 的错误；添加 Izone 概念 | `PIDSVAControllers\SlotConfig.java` 2+/3- |
| `40d0a6b` | 2026-06-11 | 忘记传了 | `PIDSVAControllers\SlotConfig.java` 36+/1- |
| `aee0ffb` | 2026-06-11 | debug | `PIDSVAControllers\PIDSVAController.java` 10+/6- + `SlotConfig.java` 1+/1- |
| `1965bb4` | 2026-06-16 | 底盘 pid | `PIDSVAControllers\PIDController.java` 7+/0- |
| `0e076d7` | 2026-06-16 | debug | `PIDSVAControllers\PIDController.java` 4+/0- |
| `f21fe62` | 2026-07-19 | 0: 40 AM | 仅 `MotorExamples\MotorPIDTuning.java` 2+/2-、`MotorSVATuning.java` 2+/2- |

⇒ **`PIDSVAControllers` 包的诞生在 2026-06-10/11，比 7 月窗口早一个月**；7 月内该包**一行都未被修改**，7 月改的只是 `MotorExamples` 下两个调参 OpMode。故「PIDSVA 出现在 BIOBUZZ 之前」这一点在 OffSeasonGame 有明确落点，路径是 `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\Controllers\MotorExamples\PIDSVAControllers\`（复数）。

#### 5.3.2 两个版本的逐项差异（OffSeasonGame HEAD `f8bda9c` vs BIOBUZZ 工作区）

| 项目 | OffSeasonGame `Controllers\MotorExamples\PIDSVAControllers\` | BIOBUZZ `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\utility\PID\` |
|---|---|---|
| 包名 | `Controllers.MotorExamples.PIDSVAControllers` | `utility.PID` |
| `PIDSVAController.java` 行数 | 142 | 197 |
| `SlotConfig.java` 行数 | 118 | 117 |
| `SVAController.java` | 49 行，同源 | 49 行，同源（`utility\PID\SVAController.java:7/21/33/44` 与 OffSeasonGame 完全同构） |
| 速度/位置环开关 | `VelCycle`（`:83` 注释「是否为速度闭环」） | `velocityLoop`（`:112` 注释「true=速度闭环…false=位置闭环」） |
| 位置环的 SVA 前馈 | **无**。`calculate(setpoint,measurement,dt,VelCycle)` 在 `false` 分支直接 `return calculate(setpoint, measurement, 0.0, 0.0, dt);`（`PIDSVAController.java:91`）→ velocity=0、acceleration=0，`kS*Math.signum(0)=0`，**位置环实际退化为纯 PID** | **有**。`refVelocity = Double.isNaN(previousSetpoint) ? 0.0 : (setpoint - previousSetpoint) / dt;`（`:128`）、`refAcceleration = Double.isNaN(previousRefVelocity) ? 0.0 : (refVelocity - previousRefVelocity) / dt;`（`:131`）→ 由**设定值差分**推导参考速度/加速度（用 `previousSetpoint`/`previousRefVelocity`，BIOBUZZ 独有字段） |
| 速度环前馈来源 | `calculate(setpoint,measurement,dt,true)` → `return calculate(setpoint, measurement, setpoint, 0.0, dt);`（`:88`）：velocity 直接取 **setpoint**，acceleration 恒 0 → **只有 kS + kV 起作用，kA 恒为 0** | 同思路但更完整：velocityLoop 时 `refVelocity = setpoint`（`:125`），且再差分一次得到 `refAcceleration` → **kS/kV/kA 三项全部可用** |
| SVA 计算式（速度环） | `double sva = cfg.kS * Math.signum(velocity) + cfg.kV * velocity + cfg.kA * acceleration;`（`:125`） | `double sva = cfg.kS * Math.signum(refVelocity) + cfg.kV * refVelocity + cfg.kA * refAcceleration;`（`:153`） |
| 分量诊断 `lastPTerm/lastITerm/lastDTerm/lastSTerm/lastVTerm/lastATerm/lastOutput` | **没有**（142 行内无任何 last* 字段） | **有**：字段 `:28-38`，赋值 `:160-166`（`lastPTerm = cfg.kP*error;` … `lastATerm = cfg.kA*refAcceleration;`），getter `:184-196`（`getLastOutput/getLastPTerm/.../getLastATerm`） |
| slot 未配置时的行为 | `cfg` 可能为 `null`，仅积分分支做了 `cfg != null` 判断（`:110`），随后 `:123` 直接 `cfg.kP` → **会 NPE** | 快速失败：`if (cfg == null) throw new IllegalArgumentException("Slot not configured: " + currentSlot);`（`:119`） |
| `SlotConfig` 默认输出限幅 | `outputMin = -14` / `outputMax = 14`（`SlotConfig.java:50/52`，**电压域**） | `outputMin = -1` / `outputMax = 1`（`SlotConfig.java:50/52`，**归一化 power 域**） |
| `SlotConfig` 其余字段 | `kP/kI/kD/maxI/iZone/kS/kV/kA` 与 builder `withKP…withOutputLimits` 完全一致（`:34-116`） | 同（`:34-116`），**两文件除默认限幅外逐行同构** |
| 内置 `PIDController.java` | 116 行；构造只有 `(kP,kI,kD)`（`:27`）与 `(kP,kI,kD,maxI)`（`:38`），**无 iZone 参数、无 getIZone** | `utility\PID\PIDController.java`：构造 `(kP,kI,kD)`（`:29`）与 **`(kP,kI,kD,maxI,iZone)`（`:41`）**，并有 `setIZone`（`:118`）/`getIZone`（`:126`） ⇒ **BIOBUZZ 的内核更新** |
| SVA 计算的重复实现 | `PIDSVAController.calculate` 内联（`:125`） | 同样内联（`:153`），未调用同包的 `SVAController.calculate(velocity, acceleration)`（`:33`）⇒ 两版都有「`SVAController` 存在但被绕过」的重复实现 |

**小结（可直接引用的判断）**：OffSeasonGame 的 PIDSVA 已经具备 **(a) PIDSVA 结构本身、(b) 多 slot 参数集、(c) Izone、(d) 秒制 dt、(e) 速度环 SVA 前馈**；缺失的是 **位置环的 SVA（setpoint 差分前馈）、三项分量诊断、以及 slot 未配置的快速失败**——这三项是 BIOBUZZ 的增量，其中「位置环前馈」正是让 `TurretDegreeController` 那种「走到角度」能真正吃到 kV/kA 的前提。BIOBUZZ 的 `SlotConfig` 把默认限幅从 `±14`（伏特）改成 `±1`（power），说明 BIOBUZZ 的调用方不再统一经 `VoltageOut` 折算。

#### 5.3.3 谁在用这套控制器（两个仓库的实际接线）

- OffSeasonGame（HEAD）：`Controllers\Turret\Shooter\Shooter.java` 与 `Controllers\Turret\turner\TurretDegreeController.java` 都 import 复数包 `Controllers.MotorExamples.PIDSVAControllers.*`；`TurretDegreeController.update()` 以 `controller.calculate(targetRoll, currentRoll, dt, false)`（`:208`）走**位置环**——即上文所述「位置环无前馈」的组合，其 `kS/kV/kA` 在代码里也确实是 `0.0`（`:45-47`），二者自洽。
- BIOBUZZ：`TeamCode\src\main\java\org\firstinspires\ftc\teamcode\Controllers\Turret\YawController.java:9-10` 与 `...\Controllers\Turret\Shooter.java:10-11` 改为 import `utility.PID.PIDSVAController` / `utility.PID.SlotConfig`，`YawController.java:76` 与 `Shooter.java:104` 均为 `new PIDSVAController()`。

---

### 5.4 三个仓库的能力轨迹表（炮台/飞轮电机控制）

行 = 能力；列 = 首次出现位置 / OffSeasonGame 截至 2026-08-18 的状态 / BIOBUZZ 状态。

| # | 能力 | 首次出现（仓库 : 文件 : 关键行） | OffSeasonGame 截至 2026-08-18 | BIOBUZZ（2026-09/10） |
|---|---|---|---|---|
| a | PIDSVA 结构（PID + S/V/A 前馈） | **OffSeasonGame** `Controllers\MotorExamples\PIDSVAControllers\PIDSVAController.java:104-133`（包诞生 2026-06-10/11，见 5.3.1） | ✅ 已实现，且**已在飞轮与炮台位置环上使用**（`Controllers\Turret\Shooter\Shooter.java:115`、`turner\TurretDegreeController.java:208`） | ✅ 迁移至 `utility\PID\PIDSVAController.java:116-169`，并新增位置环 setpoint 差分前馈（`:128`、`:131`） |
| b | 多 slot 参数集 | **OffSeasonGame** `PIDSVAController.java:12` `private final Map<Integer, SlotConfig> slots = new HashMap<>();` + `withSlot0`/`withSlot`/`setSlot`/`resetSlot`（`:25-84`） | ✅ 已有（`Map` + `currentSlot`） | ✅ 保留（`withSlot0` `:45`、`withSlot` `:56`、`resetSlot` `:78/89`） |
| c | dt 以**秒**为单位 | **OffSeasonGame** `PIDSVAController.java:82`（`@param dt 时间间隔（秒）`）；调用点 `TurretDegreeController.java:199` `(now - lastUpdateTime) / 1000.0` | ✅ 已实现（`Shooter.java:111-112` 同法） | ✅ 沿用；BIOBUZZ 另在飞轮 `PrototypeShooter` 中保留秒制 dt |
| d | 单位：**tick/s** vs **°/s** | DECODE = °/s（`controllers\shooter\Shooter.java:90` `getVelocity(AngleUnit.DEGREES)`）；OffSeasonGame 飞轮改为 **tick/s**（`Controllers\Turret\Shooter\Shooter.java:114` 不指定 AngleUnit）；炮台角度维度用 **°**（`TurretDegreeController.java:203` `getCurrentPosition() / ROLL_TICKS_PER_DEGREE`） | ⚠ **混用**：飞轮 tick/s、炮台 °、校准常数 `ROLL_TICKS_PER_DEGREE = 1.5455`（`:29`）与 `YAW_SERVO_POSITION_PER_DEGREE = 0.0385`（`:55`）桥接 | 同构延续（BIOBUZZ `YawController` 走 `utility.PID`，飞轮走 `PrototypeShooter`；未发现统一的单位抽象层） |
| e | 速度环 vs 位置环开关 | **OffSeasonGame** `PIDSVAController.java:86` `calculate(setpoint,measurement,dt,boolean VelCycle)` | ✅ 有开关，但**位置环无前馈**（`:91` 传 0.0/0.0） | ✅ 改为 `velocityLoop`（`:116`），位置环**有**前馈（`:128/131`）；另有无开关的 3 参重载 `:101` |
| f | 角度域炮台位置控制 | **OffSeasonGame** `Controllers\Turret\turner\TurretDegreeController.java`（!）`TurretDegreeController.java:203-210`：`currentRoll = rollMotor.getCurrentPosition() / ROLL_TICKS_PER_DEGREE` → `normalizeAngle` → `controller.calculate(targetRoll, currentRoll, dt, false)` → `voltageOut.getVoltageOutPower` → `rollMotor.setPower` | ✅ 已实现（`ANGLE_TOLERANCE = 0.5` `:30`，`setTargetRoll` 用 `HypParams.maxRoll` 钳位 `:120-129`，`reachedTarget()` `:232-236`）；yaw 为**开环舵机**，`currentYaw` 直接由 `yawServo.getPosition()` 反算（`:212-216`，注释 `:214-215` 明确「舵机不会以任何电气方式实时读取自己的位置并返回」） | ✅ 延续为 `Controllers\Turret\YawController.java`（`import utility.PID.*` `:9-10`，`PIDSVAController` 字段 `:76`）⇒ 从「电机 + 舵机混搭」走向独立 yaw 控制器；具体控制律见 BIOBUZZ 章节 |
| g | 掉速检测 / boost 补偿 | **BIOBUZZ**（OffSeasonGame 未找到）`Controllers\Shooter\PrototypeShooter\PrototypeShooter.java:16` `duty = Kff·ω* + Kp·err + Ki·∫err + Vboost`；`:71 kBoost = 0.0002`、`:73 boostMax = 0.4`、`:75 boostDurationSec = 0.3`、`:94 lastFFTerm/lastPTerm/lastITerm/lastBoostTerm`、`:104 boostActive`、`:106 boostTimer`、`:177-187` 触发与退出（`integral = 0.0` 防叠加 `:182`）；调参工具 `OpModes\Tuning\DropDetectionTuner.java:186 kBoost = kBoostRatio * PrototypeShooter.kp` | ❌ **未找到**。检索范围：OffSeasonGame 全 `TeamCode\src\main\java` 下 `grep -i 'dropdetect|boost'`（Turret/Shooter/MotorExamples/OpModes 全覆盖），只有与发射无关的底盘/视觉 Q 自适应命中共 0 处炮台用途。7 月飞轮的过冲/掉速只能靠 `VelocityTolerance = 20`（`Shooter.java:35`）判到速，无补偿动作 | ✅ **BIOBUZZ 独有新增**，且有配套标定 OpMode（`DropDetectionTuner`：`kSigma 3~5`、`kRate 2~3`、`kBoostRatio 2.0`、`suggestBoostDurationSec 0.3`） |
| h | 前馈调参工具（台架拟合 kS/kV/kA） | **OffSeasonGame** `Controllers\MotorExamples\MotorSVATuning.java:31-33`（`@TeleOp(name="Motor SVA Tuning")`），状态机 `enum State { kS_ASSESSING, kS_kV_FITTING, WAITING, kA_ASSESSING, FINISHED }` `:60-66`，`KS_VELOCITY_THRESHOLD = 20` `:92`，`testPoints = 5` `:92`，`kA_TestVoltages = {1..6}` `:96`，输出 `public static double kS/kV/kA` `:78-82`，逐帧写 CSV `:286` + `sleep(10)` `:291` | ✅ 已实现，且**产出的数字被直接写死在飞轮里**：`Shooter.java:17-29` 的 `kS=1.23, kV=0.0049, kA=0.0` | ✅ 保留并扩展：新增 `OpModes\Tuning\DropDetectionTuner.java`（掉速阈值/boost 标定）与 `PrototypeShooterTest.java:22`（Dashboard 在线调 `kff/kp/ki/iZone/maxI/dropThreshold/kBoost`） |

**明确结论**：
1. **OffSeasonGame 在 2026-08-18 之前已经具备**：(a) PIDSVA 结构、(b) 多 slot 参数集、(c) 秒制 dt、(e) 速度/位置环开关、(f) 角度域炮台位置闭环（电机侧）、(h) SVA 前馈台架调参工具，并且**已把 PIDSVA 接进飞轮速度环和炮台 roll 位置环**——这是 DECODE→BIOBUZZ 之间真正的过渡件。
2. **BIOBUZZ 新增（OffSeasonGame 中未找到）**：(g) **掉速检测 + boost 补偿**（`PrototypeShooter` + `DropDetectionTuner`）；以及把 PIDSVA 从 `Controllers\MotorExamples\` 提升为通用 `utility\PID\` 包、补上**位置环 setpoint 差分 SVA 前馈**（`PIDSVAController.java:128/131`）、**六项分量诊断 getter**（`:184-196`）、**slot 未配置快速失败**（`:119`）、**PIDController 的 iZone 构造与 getter**（`utility\PID\PIDController.java:41/118/126`）、以及 `SlotConfig` 默认限幅由 `±14` 改为 `±1`（`SlotConfig.java:50/52`）。
3. DECODE 的「两套增益按 750 °/s 切换」在 OffSeasonGame **被显式否定**，理由写进了源码注释（`Controllers\Turret\Shooter\Shooter.java:104-105`：`//两套PID是不对的…现在有了速度前馈项，理论上不需要两套了，所以统一。`）——这是三个仓库之间最清晰的一次设计决策断点。

---

### 5.5 stash / 悬挂引用核对（任务要求第 4 项）

`git stash list --format='%gd|%h|%cd|%s' --date=short` 在 OffSeasonGame 只返回 **1 条**：

```
stash@{2026-09-24}|5ac7b1d|2026-09-24|On master: !!GitHub_Desktop<master>
```

- `git rev-parse 'stash@{0}'` = `5ac7b1d6a24a5b2a7159719121cfc4f46c0b6edb`；`git log -1` 显示它是 `zihengtangtzh25` 于 **2026-09-24** 创建的 WIP stash。
- `git stash show --stat 'stash@{0}'`（**必须给花括号加引号**，PowerShell 会吞掉裸 `stash@{0}`，报 `fatal: ambiguous argument 'stash@'`）：
  ```
  .../Turret/turner/TurretDegreeController.java      |   5 +-
  .../ftc/teamcode/OpModes/AutoActionBlue.java       | 228 +++++++++----------
  .../ftc/teamcode/OpModes/AutoActionRed.java        |   6 +-
  .../OpModes/AutoActionWithoutVisionBlue.java       | 252 ++++++++++-----------
  .../OpModes/AutoActionWithoutVisionRed.java        | 252 ++++++++++-----------
  .../ftc/teamcode/OpModes/EatTest.java              | 166 +++++++-------
  6 files changed, 455 insertions(+), 454 deletions(-)
  ```
  ⇒ stash 里**确实含炮台电机控制**：`Controllers/Turret/turner/TurretDegreeController.java`（5 行改动）；其余 5 个文件全是 `OpModes/AutoAction*.java` / `EatTest.java` 的自动流程。以 `455 insertions / 454 deletions` 近乎 1:1 的比例看，主体是**缩进/换行风格**级别的改写，不是新控制律。**本次未逐行展开该 5 行 diff**（`git stash show -p` 与 pathspec 同用会报 `Too many revisions specified`，需拆成两次调用）——标记为**未完全核实**，如需引用请另行展开。
- `0bdc92f` 在 `git rev-parse --all` 中**不是一个 stash**，而是一个 **dangling commit**：`git cat-file -t 0bdc92f` = `commit`，其自身 subject 为 `index on master: d716183 大修6`（即 `git stash create` 产生的 index 提交，日期同为 2026-09-24）。它**不在** `git stash list` 中，只能靠 reflog/悬空对象访问 ⇒ 任务简报里把 `5ac7b1d` 与 `0bdc92f` 并列为「7 月 stash refs」不成立：两者都是 **2026-09-24** 的产物（远晚于 7 月窗口），且 `0bdc92f` 甚至不是一条 stash 记录。
- 未找到任何**日期落在 2026 年 7 月**的 stash。已检索：`git stash list`（1 条，09-24）、`git reflog show stash`（同上）、`git log --all --format=…` 全量匹配 `5ac7b1d|0bdc92f`（各 1 处，均 09-24）。

> 结论：**7 月窗口内没有 stash**；`5ac7b1d` / `0bdc92f` 是 2026-09-24 的休赛期收尾悬挂引用，其中只有 `5ac7b1d` 触及炮台（`TurretDegreeController.java` 5 行）。

---

## 六、BIOBUZZ 调参方法学与实测证据

> 本章回答一个问题：DECODE 赛季那个「凭手感改两个静态增益、在 750°/s 处硬切换」的经验式调参流程，在 BIOBUZZ 里被换成了什么，以及换完之后**实测到底好不好**。
>
> 结论先行：BIOBUZZ 建立了一套**无电气/机械参数标定**的闭环调参体系——端上 5 个自动/半自动 OpMode 采数据，离线两套脚本（Python + MATLAB）复算参数，再用 4 组实机日志做验收。这套体系在 2026-10-11 的 `1937409` 提交中一次性落地（+1915/−0 纯新增），2026-10-15 的 `b182459` 补上关键修正。**实测同时暴露了当前参数的严重偏差**：标定值与代码默认值差 10~36 倍，掉速补偿在全部 13 次真实发射事件中顶死在 `boostMax` 限幅。

---

### 6.1 对照基线：DECODE 的调参方式

DECODE 赛季（`Team19725_DECODE`）飞轮速度环的整定完全是手工的，没有任何工具链支撑：

| 维度 | DECODE 的做法 | 证据 |
|---|---|---|
| 参数载体 | `Shooter.java` 里的**静态字段**，改完重新编译部署 | `TeamCode\...\controllers\shooter\Shooter.java` |
| 增益组数 | **2 组**（大球/小球），按实测转速在 `750 °/s` 处硬切换 | 同上，「大球增益 / 小球增益」分支 |
| 切换阈值 | `750`（单位 °/s） | 同上 |
| 大球增益 | `KP=0.01, KI=0.2, KD=0.05`（注释写「小球参数」）| 同上 |
| 小球增益 | `KP_small=0.011, KI_small=0.05, KD_small=0.08`（注释写「大球参数」）| 同上 |
| PID 内核 | `utility\PIDController.java` (51 行)，**不做 dt 归一化**，积分项为 `integral += error * dt`，`dt` 由 `System.currentTimeMillis()` 算出且为**毫秒** | `TeamCode\...\utility\PIDController.java` |
| 到位判据 | `SpeedTolerance = 35`（°/s）瞬时比较 + `MeanFilter` 3 点均值滤波 | `Shooter.java` |
| 前馈 | 无。只有 `MinPower = 0.1/0.3` 的经验底功率与 `BlockPower = -0.3` 的堵转反转 | 同上 |
| 零功率行为 | `BRAKE` | 同上 |
| 数据记录 | 无 CSV、无日志、无绘图 | 全仓检索无相关文件 |
| 结论 | **纯试错**：改代码 → 编译 → 上场试 → 凭手感再改 | — |

对比之下，BIOBUZZ 的调参体系在三个层次上都做了替换：**参数语义**（功率量纲 → 占空比量纲）、**获取方式**（手工试错 → 无模型辨识）、**验收方式**（肉眼观察 → 量化指标表）。

---

### 6.2 端上：五套无模型标定 OpMode

全部位于 `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\OpModes\Tuning\`，由 `1937409` 新增、`b182459` 修订。Driver Station 分组统一为 `Tuning`，名称前缀 `Tune6.x`。理论依据是《飞轮系统控制理论研究报告》第 6 章「无电气/机械参数标定控制参数的方法」——**全部方法只利用「占空比输入 + 速度输出」两组可测量**，不需要知道 `R / L / Ke / Kt / J / B`。

标定目标对象是 `Controllers\Shooter\PrototypeShooter\PrototypeShooter.java` 的 9 个参数：
`kff / kp / ki / maxI / iZone / dropThreshold / dropRateThreshold / kBoost / boostDurationSec`。

#### 6.2.1 数据记录器 `TuningLogger.java` (94 行)

所有五个 OpMode 共用同一个轻量 CSV 记录器：

- 目录：`new File(context.getExternalFilesDir(null), "tuning")`（`TuningLogger.java:47`）
- 文件名：`<name>_<System.currentTimeMillis()>.csv`（`:52`）
- 注释头：`# flywheel tuning data (<name>)`（`:58`）、`# t(s),duty,velocity(tick/s)`（`:59`）
- 表头：`t,duty,velocity`（`:60`）
- 行格式：`String.format(Locale.US, "%.4f,%.4f,%.2f", t, duty, velocity)`（`:83-85`）
- 落盘节流：`FLUSH_EVERY = 20`（`:32`）——每 20 行 flush 一次，避免高频写盘拖慢控制循环
- 关键设计：**记录失败绝不打断控制循环**（`:77-79`），掉盘只丢数据、不停车

这套格式被离线脚本、MATLAB 脚本、`numpy.loadtxt(..., comments='#', skiprows=1)` 三方共同约定，是整个体系的数据契约。

#### 6.2.2 方法一：阶跃响应辨识 `Tune6.1 StepResponse`（`StepResponseTuner.java` 203 行）

报告 6.1，官方推荐的首选方法，**一次给出 `Kd / τ / kff / kp / ki`**。

原理：先以 `d0` 稳定运行，阶跃到 `dStep`，速度响应近似
`ω(t) = ω0 + Kd·Δd·(1 − e^(−t/τ))`，
据此读 `Kd = Δω/Δd`、`τ = t63.2%`，再按零极点对消：
`kff = 1/Kd`，`kp = τ/(Kd·τc)`，`ki = 1/(Kd·τc)`，其中 `τc = tRecover/ratio`。

代码实现（`StepResponseTuner.java`）：

- `logger.start(..., "step01")`（`:102`）
- 阶段 1：`shooter.motor.setPower(d0)`（`:109`），稳定 `settleSec`，取 `steadyWindowSec` 窗口均值得 `ω0`（`:117-125`）
- 阶段 2：`setPower(dStep)`（`:128`），最多 `MAX_SAMPLES = 4000`（`:135-136`）
- 有效性：`sampleCount < 5` 判 invalid（`:144`）
- `kd = deltaW / deltaD`（`:165`）
- `targetW = omega0 + 0.632 * deltaW`（`:167`）
- `tau` = 首个满足 `velArr[j] >= targetW` 的时刻（`:169-174`）
- `kff = 1.0/kd`（`:177`）、`tc = tRecover/ratio`（`:178`）、`sugKp = tau/(kd*tc)`（`:179`）、`sugKi = 1.0/(kd*tc)`（`:180`）
- 结论写回 CSV 注释行：`# omega0=... Kd=%.6f tau=%.4f Kff=%.8f Kp=%.8f Ki=%.8f`（`:182-184`）

默认参数：`d0=0.15`、`dStep=0.25`、`settleSec=2`、`recordSec=3`（≥4τ）、`steadyWindowSec=0.5`、`tRecover=0.5`、`ratio=4`（3~5）。按 A 重跑，X 立即停转。

#### 6.2.3 方法二：Ziegler-Nichols 临界比例度法 `Tune6.2 ZieglerNichols`（221 行）

原理：P-only 控制 `d = Kp·(ω*−ω)`，手动增大 Kp 直到**等幅振荡**，当前 Kp 即 `Ku`，振荡周期为 `Tu`。ZN 表：`Kp = 0.45·Ku`，`Ki = 0.54·Ku/Tu`。

关键实现（`ZieglerNicholsTuner.java`）：

- 进入时保存原值 `origKff / origKi / origDropThreshold / origDropRateThreshold`（`:79-82`）
- 每帧强制 `PrototypeShooter.kff = 0.0; ki = 0.0; dropThreshold = 1e9; dropRateThreshold = 1e9`（`:97-100`）——即**打掉前馈、打掉积分、关闭掉速补偿**，只留纯比例环节，隔离出真正的临界点
- DPad 上/下把 `kp` 乘 / 除 1.1（`:103-111`）
- kp 一旦变化 ⇒ `bufN = 0; kpChangeTime = timer.seconds()`（`:114-119`），**清空采样缓冲并重新等待 `warmupSec`**，避免把上一组 kp 的暂态和启动峰值算进幅值与周期
- 只采 `t − kpChangeTime >= warmupSec` 之后的样本（`:125-127`）
- A 键锁定：`ku = PrototypeShooter.kp; tuLocked = estTu; sugKp = 0.45*ku; sugKi = 0.54*ku/tuLocked`（`:135-145`）
- 退出时恢复原有静态量（`:158-161`）——**唯一例外是 `kp` 不还原**，因为那是用户主动整定行为

默认：`targetVelocity=3000`、`minZeroCrossings=4`、`warmupSec=3`。遥测显示 `(settling)` 时不得读振荡结果；只有幅值近似恒定（等幅）时当前 Kp 才是 Ku。

#### 6.2.4 方法三：继电反馈自整定 `Tune6.3 RelayFeedback`（218 行）

原理：用继电器代替控制器——`err > hysteresis → dHigh`，`err < −hysteresis → dLow`。系统自动进入极限环，测幅值 `a` 与周期 `Tu`，则
`Ku = 4·Δd/(π·a)`（`Δd = dHigh − dLow`），再 `Kp = 0.45·Ku`、`Ki = 0.54·Ku/Tu`。

实现（`RelayFeedbackTuner.java`）：

- `if (err > hysteresis) lastDuty = dHigh; else if (err < -hysteresis) lastDuty = dLow;` 然后 `setPower(lastDuty)`；初始 `lastDuty = (dHigh+dLow)/2.0`（`:106`）
- 有效条件 `bufN < 10` 判 invalid（`:135`）
- `ku = 4.0 * deltaD / (Math.PI * amp)`（`:146`）、`sugKp = 0.45*ku`、`sugKi = 0.54*ku/tu`（`:147-148`）

默认：`dHigh=0.7`、`dLow=0.5`、`transientSec=2`、`measureSec=4`、`hysteresis=0`。相比方法二（手动把增益推到临界点），继电法**由机器自己产生极限环，更安全**，是 Guide 推荐的替代路径。

#### 6.2.5 方法四：掉速检测与补偿参数标定 `Tune6.4 DropDetection`（190+59 行）

标定 `dropThreshold / dropRateThreshold / kBoost / boostDurationSec` 四个参数。

原理：让系统闭环稳定运行，但**关闭掉速补偿以测量纯正常波动**，然后统计：

- `Δω_th = kSigma·σ_ω`（`σ_ω` 为**样本标准差**，÷(N−1)）
- `k_th = kRate·max(−dω/dt)`（仅取**速度下降**的分量，即最大减速度）
- `kBoost = kBoostRatio·kp`
- `boostDurationSec` 建议 0.1~0.5 s，由发射实测微调

关键实现（`DropDetectionTuner.java`）：进入时强制 `dropThreshold=1e9; dropRateThreshold=1e9` 每帧生效（`:98-99`），退出前先 `shooter.motor.setPower(0)` 再恢复原值（`:97`）；`stdVel = sqrt(sq/(sampleCount-1))`；`quantStep` = 把非零速度取整后求**最大公约数**；EMA_ALPHA=0.3、MAX_SAMPLES=4000；`maxDecel` **先对速度做 EMA 平滑再差分**（`:168` 的注释说明了原因——避免相邻样本读数噪声把阈值抬得过高）。

遥测给出：`mean / sigma / step`、`sigma (steps)`（σ 相当于几个量化台阶）、`max decel (tick/s²)`、`dropThreshold` + `(N steps)`、`dropRateThreshold`、`kBoost (2~5 kp)`、`boostDurationSec`。

默认：`targetVelocity=3000`、`settleSec=2`、`measureSec=5`、`kSigma=5`、`kRate=2.5`、`kBoostRatio=2`、`suggestBoostDurationSec=0.3`。

> **Guide 第 165-167 行的判读铁律**：遥测同时给出量化台阶与 σ 相当于几个台阶。若 `dropThreshold` 只相当于 1~2 个台阶，说明阈值被量化噪声主导，需保守取值或提高目标转速——「**任何稳态精度/阈值的讨论都必须先报出台阶**」。

#### 6.2.6 方法五：无模型前馈增益标定 `Tune6.5 Feedforward`（195 行）

标定 `kff`。原理：闭环稳定运行时记录稳态占空比 `d_ss`，则 `Kff = d_ss/ω*`。

关键实现（`FeedforwardTuner.java`）：

- 持续强制 `kff = 0`，**由 PI 提供全部稳态占空比**，避免前馈饱和导致读数失真（`:124` 退出时恢复 `origKff`）
- `lastKff = lastOmega > 1e-6 ? lastDss/lastOmega : 0`（`:156-158`）
- `lastSaturated = lastDss >= 0.95`（`:160`）——饱和点标记为无效
- 记录多点：`points.add(...)`（`:163`）
- CSV 注释：`# point omega=%.1f dss=%.4f kff=%.8f%s`，饱和点追加 ` SATURATED(invalid)`（`:164-165`）
- 独立的 `measureTimer` 保证 CSV 时间轴单调（`:130-131`）

默认：`targetVelocity=典型工作转速`、`velocityStep=200 tick/s`、`settleSec=2`、`windowSec=1.5`。遥测同时显示 `Kff (through origin)`（过原点斜率，推荐）与 `Kff (linear fit slope)`（含截距拟合，可反映静摩擦）。

> **重要缺口**：`ff05` 的线性拟合截距明显非 0 说明存在静摩擦，但**当前控制律（前馈 + PI + 掉速补偿）没有 `kS` 项**——Guide 第 202 行自己承认了这一点：「可考虑用截距补偿 `kS`（当前控制器未含该项）」。

#### 6.2.7 推荐标定顺序（Guide 6.6）与参数速查

1. **准备**：取下异物，确保飞轮可安全低速运行
2. **阶跃辨识**（方法一）→ 得 `Kd / τ` 及初版 `kff / kp / ki`
3. **PI 微调**：方法二（ZN）或方法三（继电）复核 `kp / ki`，**取较保守者**
4. **前馈标定**（方法五）→ 用实测稳态占空比修正 `kff`
5. **掉速参数**（方法四）→ 得 `dropThreshold / dropRateThreshold / kBoost`，`boostDurationSec` 先取 0.3 s
6. **发射实测**：真实发射，观察掉速与恢复，微调 `kBoost / boostDurationSec`

| PrototypeShooter 参数 | 单位 | 来源方法 |
|---|---|---|
| `kff` | 占空比/(tick/s) | 6.1（=1/Kd）、6.5（=d_ss/ω*） |
| `kp` | 占空比/(tick/s) | 6.1、6.2、6.3 |
| `ki` | 占空比/(tick/s·s) | 6.1、6.2、6.3 |
| `maxI` | 占空比 | 经验值（防积分饱和） |
| `iZone` | tick/s | 经验值（积分生效误差带） |
| `dropThreshold` | tick/s | 6.4 |
| `dropRateThreshold` | tick/s² | 6.4 |
| `kBoost` | 占空比/(tick/s) | 6.4（=倍数·kp） |
| `boostDurationSec` | s | 6.4 + 发射实测 |

**量纲一致性**：队列中所有增益单位均为「占空比」，与 `PrototypeShooter` 不读取电压传感器的设计一致。若后续改为电压补偿（`d = V/Vdc`），需把增益全部乘以 `Vdc` 换成电压量纲。

---

### 6.3 离线：`tools/tuning/` 双语言分析链路

#### 6.3.1 已提交部分（`b182459` 新增）

| 文件 | 行数 | 作用 |
|---|---|---|
| `tools\tuning\README.md` | 112 | 端上↔离线的方法/文件名映射表、CSV 格式、判读要点、合成样例自检 |
| `tools\tuning\analyze_tuning.py` | 687 | Python 主分析器（依赖 numpy + matplotlib，**不需要 scipy**） |
| `tools\tuning\analyze_tuning.m` | 679 | MATLAB 对照实现，同一套公式，输出同名文件 |
| `tools\tuning\make_samples.py` | 119 | 生成 5 组真值已知的合成 CSV，用于脚本自检 |

端上↔离线映射表（README §方法表）：

| 方法 | OpMode | 文件名前缀 | 输出 |
|---|---|---|---|
| 6.1 阶跃响应辨识 | `Tune6.1 StepResponse` | `step01_*.csv` | Kd、τ、kff、kp、ki |
| 6.2 Ziegler-Nichols | `Tune6.2 ZieglerNichols` | `zn02_*.csv` | Ku、Tu、kp、ki |
| 6.3 继电反馈 | `Tune6.3 RelayFeedback` | `relay03_*.csv` | a、Tu、Ku、kp、ki |
| 6.4 掉速检测 | `Tune6.4 DropDetection` | `drop04_*.csv` | σ、量化台阶、maxDecel、Δω_th、k_th、kBoost |
| 6.5 前馈标定 | `Tune6.5 Feedforward` | `ff05_*.csv` | 多点回归 Kff |

**离线比端上多做的两件事**（README 开头明确说明）：

1. **最小二乘拟合 τ**（端上只能读 t63.2% 判据，离散采样下偏大）
2. **多点回归 Kff**（端上只能取单点）

调用方式：

```bash
python tools/tuning/analyze_tuning.py ./tuning_data                 # 分析目录下全部 CSV
python tools/tuning/analyze_tuning.py step01_1700.csv --out ./out   # 单文件 + 指定输出目录
python tools/tuning/analyze_tuning.py ./tuning_data --no-plot       # 只出报告
python tools/tuning/analyze_tuning.py zn02_1.csv --ku 9e-5          # 手工指定 Ku
python tools/tuning/analyze_tuning.py drop04_1.csv --kp 5e-5        # 计算 kBoost 需要 kp
```

每份输入产出 `<名字>_report.txt`（结论文本，同时打印到终端）+ `<名字>_analysis.png`（转速与占空比曲线；`ff05` 第二幅画成 (ω, d_ss) 回归图）。MATLAB 版选项：`-out DIR`、`-noplot`、`-window S`、`-ku K`、`-kp K`、`-alpha A`。

> **环境约束记录**：MATLAB 需要写 `%APPDATA%\MathWorks` 与临时目录，因此运行它的沙箱必须是 `danger-full-access`，否则 MATLAB 直接启动失败，退出码 `-1073741824` 附近的 `0xC0000142`（README 第 56-58 行明确记录）。

#### 6.3.2 脚本自检（`make_samples.py` 真值还原）

README §6 给出 5 组合成样例的「真值 → 复算值」对照，这是**验证分析脚本本身正确性**的手段（没有机器人时也能跑）：

| 方法 | 真值 | 复算值 |
|---|---|---|
| step01 | Kd=8000、τ=0.30 s | Kd=7984.2、τ(最小二乘)=0.2999 s、τ(t63.2%)=0.32 s |
| zn02 | Tu=0.42 s、Ku=9e-5 | Tu=0.4382 s、Ku 取自锁定行 → kp=4.05e-5、ki=1.11e-4 |
| relay03 | Δd=0.2、a≈180、Tu=0.55 s | Δd=0.2000、a=165.0、Tu=0.5967 s → Ku=1.54e-3 |
| drop04 | σ≈15、台阶=1 | σ=15.51、台阶=1.0、maxDecel=600.6、Δω_th=77.57 |
| ff05 | kff=1.2e-4、截距 0.012 | 过原点 1.2533e-4；线性拟合斜率 1.2000e-4、截距 0.0120 |

注意 `τ(t63.2%)=0.32` vs `τ(最小二乘)=0.2999`——**离散采样下 t63.2% 判据系统性偏大 6.7%**，这正是离线必须做最小二乘的原因。

#### 6.3.3 判读要点（README §5 六条「踩过的坑」）

1. **先看量化台阶**：`drop04` 会报出速度反馈的量化台阶（实测速度的最大公约数）与 σ 相当于几个台阶。若 `Δω_th` 只相当于 1~2 个台阶，说明阈值被量化噪声主导，报告里会给出 `⚠` 警告。
2. **τ 取哪个**：端上用 t63.2% 判据，离散采样下偏大；离线最小二乘更准，两者差距大时以最小二乘为准。
3. **`ff05` 看截距**：线性拟合截距明显非 0 → 存在静摩擦，当前控制律没有 `kS` 项，可考虑加常数偏置，否则**低转速段误差会偏大**。
4. **`zn02` 必须近似等幅**：只有等幅振荡时当前 Kp 才是 Ku；幅值还在增长说明还没到临界点。
5. **一致性核对**：`drop04` 报告里会列出端上给出的 `sigma/maxDecel/dropThreshold/...`，与离线复算值应基本一致；不一致说明端上那段代码或采样口径变了。
6. **单位**：增益单位统一为「占空比」。

#### 6.3.4 未提交的扩展工具（工作区独有）

以下 7 个文件在**任何 git 提交中都不存在**，是纯工作区产物：

| 文件 | 行数 | 作用 |
|---|---|---|
| `tools\tuning\analyze_dashboard_log.py` | 533 | 分析 FTC Dashboard 导出的 `.jsonl` 运行日志 |
| `tools\tuning\analyze_launch_events.py` | 272 | 自动识别发射掉速事件并统计恢复性能 |
| `tools\tuning\plot_flywheel.py` | 364 | 画 Flywheel 台架 CSV（`fly1/fly2/fly3_*.csv`） |
| `tools\tuning\plot_speed.m` | 415 | 纯 MATLAB 画转速图 + 诊断图 |
| `tools\tuning\check_plot_data.py` | 138 | 绘图数据一致性自检 |
| `tools\tuning\export_clean_table.py` | 119 | 清洗时间列、重建均匀时间轴 |
| `tools\tuning\转速图分析指南.md` | 133 | 读图方法学与验收阈值表 |

其中 **`转速图分析指南.md` 是整个体系里方法论密度最高的一份文档**，其核心贡献是识别出一个数据陷阱并给出量化判据（详见 6.5 节）。

---

### 6.4 实测：四组真实日志

`tools\tuning\out\` 下有 4 组完整实测数据（`log2` / `log3` / `log4` / `shooter_log`），每组含清洗后 CSV、`_report.txt` 结论文本与多张 PNG。以下数字全部来自这些报告的原文。

#### 6.4.1 `shooter_log.csv`——**增益严重标错的直接证据**

- 规模：883 样本，8.20 s，重建采样间隔 9.3 ms（107.5 Hz），时间列毫秒倒退 16 处
- 量化台阶（速度最大公约数）：**20 tick/s**，86 个不同速度值，范围 [−0, 2060]
- 目标 2000 段：v 均值 1885，**sd 334.2**，误差均值 114.5，duty 0.783，**饱和 23.2%**，电流 1.84 A
- 稳态末窗（最后 1.0 s）：**v 2008，v_sd 12.5（0.62 台阶），误差 −8.3（0.42%），duty 0.724±0.204，duty 极差 0.853，饱和 5.6%，|P| 振幅 0.762**
- **增益反演**：`kff ≈ 0.00038925`；**`kp ≈ 0.01904500`**（区间 [−0.019045, 0.038090]）；`ki ≈ 0.00019283`
- 量化台阶换算：**一个 20 tick/s 的台阶会让 P 项跳变 0.3809 占空比**——这已经接近满量程的 40%
- 掉速补偿：**触发 0.0%**（本次没有掉速扰动）
- duty：均值 0.616，饱和 19.4%，贴底 21.9%
- **极限环**：速度过零周期 ≈ **73 ms**，幅值 ≈ 30.0 tick/s = **1.50 台阶**
- 报告原文诊断：
  - `[严重] kp 相对量化台阶过大：误差只要跳 1 个台阶(20 tick/s)，P 项就跳 0.381 占空比。稳态必然在限幅之间来回撞，表现为转速 2000↔2020 反复、占空比 0.5↔1.0 横跳。`
  - `[正常] 稳态误差 0.42%（-8.3 tick/s），在 ±2% 带内。`
  - `[提示] 全程未触发掉速补偿：本次没有掉速扰动（正常），但也没有验证过 boost 是否有效——需要投球实测。`
  - `[警告] 占空比饱和占比 19.4%：前馈+反馈已把输出顶到上限，此时 Kp 再大也只会加深饱和，抗饱和与积分上限要一起看。`

**与代码默认值的偏差**：`PrototypeShooter` 默认 `kp = 0.00005`，实测反演 `kp ≈ 0.019045`——**差 381 倍**。这份日志是「默认参数从未真正标定过」的铁证。

#### 6.4.2 `log2` vs `log3`——**同一组增益，4.4 倍噪声差异**（这是 `b182459` 修正的直接依据）

两份日志用**完全相同的参数**，反演出的 kp **完全相同**：`kp = 0.00182000`。但表现差异巨大：

| 指标 | `log2` | `log3` | 差异 |
|---|---|---|---|
| 样本数 / 时长 | 4895 / 46.26 s | 2712 / 26.59 s | — |
| 采样间隔 / 频率 | 9.5 ms / 105.8 Hz | 9.8 ms / 102.0 Hz | — |
| 时间列毫秒倒退 | **87 处** | **49 处** | — |
| 量化台阶 | 20 tick/s | 20 tick/s | 相同 |
| 反演 `kp` | **0.00182000** | **0.00182000** | **相同** |
| 稳态末窗 duty sd | **0.0790** | **0.0182** | **log3 小 4.4×** |
| 极限环周期 | **51 ms** | **28 ms** | — |
| 极限环幅值 | **30.0 tick/s = 1.50 台阶** | **10.0 tick/s = 0.50 台阶** | **log2 大 3×** |
| 稳态误差 | 21.9（1.25%） | −1.8（**0.10%**） | — |
| **掉速补偿触发率** | **21.1%** | **0.0%** | **决定性差异** |
| duty 饱和占比 | 0.4% | 1.5% | — |

触发时的误差中位数只有 **30 tick/s**（触发帧误差最小 −10），而未触发帧的最大减速度达 −4232 tick/s²。**误差只有 30 tick/s 却触发了「掉速」判定**——这说明 `log2` 里 boost 的触发跟真实掉速无关，而是**从 0 拉起转速的正常加速段被误判成了掉速**。

这正是 `b182459`（2026-10-15，+22/−1）加的那个修正所针对的缺陷。加的是「武装（arming）」规则，代码为 `PrototypeShooter.java`：

```java
private boolean dropArmed = false;                      // :109
private double lastTargetVel = Double.NaN;              // :111

if (targetVel != lastTargetVel) {                       // :168-171
    dropArmed = false;
    lastTargetVel = targetVel;
}
if (Math.abs(error) < dropThreshold) {                  // :172-174
    dropArmed = true;
}
boolean dropDetected = dropArmed && targetVel != 0
        && (error > dropThreshold || dVelocity < -dropRateThreshold);   // :177-178
```

即：**只有误差已经回到 `dropThreshold` 以内（系统真正进入稳态）之后，才允许触发掉速补偿；目标转速一旦改变立即重新解除武装。** `resetController()` 里也补了 `dropArmed = false; lastTargetVel = Double.NaN;`（`:244-248`）。

`OpModes\Tuning\Guide.md` 第 23-25 行的原文解释了动机：

> 掉速检测的「武装」规则：`PrototypeShooter` 只有在误差回到 `dropThreshold` 以内（即已进入稳态）之后才允许触发掉速补偿，目标转速一旦改变立刻重新解除武装——**否则从 0 拉起时的正常加速段会被误判为掉速，使 boost 全程挂着并在到速后造成过冲**。

参数上还有一处旁证：`log2` 反演 `kff = 0.00043222`，而当时使用的值是 `0.00032400`（偏差 −25.0%）；`log3` 反演 `0.00043702`，使用 `0.00043206`（偏差 −1.1%）。两组反演值高度一致（0.000432 vs 0.000437），说明**真实的 kff ≈ 0.00043**，而代码里继承的默认 `kff = 0.0002` 只有真实值的一半不到。

#### 6.4.3 `log4`——**13 次真实发射的掉速数据（当前最重要的一组）**

规模：13643 样本，132.73 s，9.7 ms（102.8 Hz），时间列毫秒倒退 242 处；量化台阶 20 tick/s，121 个不同速度值，范围 [−0, 2400]。这是**唯一一组包含真实投球发射的日志**。

**13 次发射事件的完整统计**（`log4_launch_report.txt` 原文）：

| 指标 | 数值 |
|---|---|
| 事件数 | **13** |
| 跌幅中位数 | **270 tick/s（13.17%）** |
| 跌幅范围 | 最大 290，最小 110 |
| 恢复时间中位数 | **0.428 s**（最长 0.642 s，**<0.3 s 的比例仅 15%**） |
| 恢复后过冲中位数 | **50 tick/s**（最大 90） |
| boost 参与率 | **100% 的事件里 boost 触发**；boost term 峰值 **0.400** |
| 恢复期占空比 | **100% 的事件顶到饱和（≥0.999）** |
| 事件间隔中位数 | 8.18 s |

报告原文诊断（三条全部是警告）：

- `[警告] boost term 已顶到 boostMax 限幅（0.400）→ 补偿强度被限住，可考虑提高 boostMax 或 kBoost。`
- `[警告] 过半事件恢复期占空比顶到 100% → 恢复速度受母线电压/电机能力限制，加大增益没有用。`
- `[警告] 恢复时间中位 0.43 s 偏长（比赛要求通常 <0.3~0.5 s）。`

**最优稳态**出现在 1750 tick/s 段：sd 11.0（0.55 台阶），误差 −0.7（**0.04%**），duty 0.691±0.020，饱和 0%。其余稳态段（1950 / 2050 / 2250 tick/s）同样优秀：sd 10.2~10.8（0.51~0.54 台阶），误差 −2.2 ~ −0.1（0.00~0.11%）。

**但 2250 tick/s 高转速起步段暴露了能力边界**：`v 均值 2211，sd 161.8（8.09 台阶），误差 37.9（1.69%），duty 0.978，饱和 74.9%`——电机已经接近满占空比，再往上没有余量。

#### 6.4.4 独立复算验证（`plot_speed_stats.txt`）

MATLAB `plot_speed.m` 对同一份 `log4` 做了**独立复算**，用于交叉验证 Python 链路：

- 分段表与 Python 版基本一致（如 1750 段：v 1750，sd 10.9，0.55 台阶，误差 −0.3 / 0.02%，duty 0.691）
- **掉速事件数 14** vs Python 版 13——差异来自两者用了略不同的带宽/最小目标门槛（MATLAB 版多识别出 `2250|92.67|130|5.8%|0.409 s` 这一条浅跌落事件）
- 汇总：`跌幅中位 270 tick/s (12.6%) 恢复中位 0.418 s (最长 0.652) 过冲中位 50`
- `占空比：均值 0.655 饱和(>=0.999) 占比 7.9% boost 触发占比 5.8%`

两份独立实现的结论互相印证，说明数据链路本身可信；差异点已被解释（判定门槛口径不同），不是 bug。

---

### 6.5 方法论亮点：`转速图分析指南.md` 的量化判据体系

这份 133 行的文档不是脚本说明，而是一套**读图方法学 + 验收阈值表**，是整个体系里最有复用价值的部分。

#### 6.5.1 识别的数据陷阱：时间列的毫秒字段是坏的

原始日志的 `time` 列（`HH:MM:SS.mmm`）**毫秒字段有缺陷**（在一秒内会从 ~900 跳回 ~100）。后果：若按该列排序，会**凭空造出锯齿状的假掉速**。解决方案是 `export_clean_table.py`：丢弃毫秒、**按行序重建均匀时间轴**（`dt = span/(n−1)`），并重算 `error = target − velocity`。

这解释了三份报告中出现的「时间列毫秒倒退 87 / 49 / 242 处」——这不是采集错误，而是日志格式缺陷，必须在清洗阶段处理。

#### 6.5.2 三把「尺子」——任何结论之前必须先量

1. **量化台阶**：速度实测值的最大公约数。台阶 = 20 tick/s 时，在目标 340 处是 5.9%，在 2250 处只有 0.9%。**同一个台阶在不同转速下的相对意义完全不同。**
2. **±2% 带 vs 一个台阶**：在目标 250 处，±2% = 5 tick/s，**比一个台阶还窄**——即「±2% 判据在该转速下根本不可达」。
3. **采样率 ≈ 100 Hz（dt ≈ 10 ms）**：比 10 ms 更快的现象**看不见**；一个在一帧内完成的跌落是正常现象，不是数据丢失。

#### 6.5.3 验收阈值表（好 / 可接受 / 差）

| 指标 | 好 | 可接受 | 差 |
|---|---|---|---|
| 稳态误差 | ≤1 台阶 或 ≤1% | ≤2% 且 ≤2 台阶 | >2% 且 >2 台阶 |
| 稳态 v_sd | ≤0.5 台阶 | ≤1 台阶 | >2 台阶 |
| 稳态 duty sd | ≤0.02 | ≤0.05 | >0.1 |
| duty 饱和占比 | ≈0% | <5% | >10% |
| 加速过冲 | ≤3% | ≤5% | >10% |
| 发射后恢复（回到 ±2%） | <0.3 s | <0.5 s | >0.6 s |
| `kp × 一个台阶` | ≤0.02 | ≤0.05 | >0.1 |

按这套表回看 6.4 的实测：

- 稳态性能（1750~2250 tick/s 段）**达到「好」**：误差 0.00~0.11%，v_sd 0.51~0.55 台阶，duty sd ≈0.02
- `log4` 的 2250 段饱和 13.5%，`shooter_log` 饱和 19.4% → **落「差」档**
- 发射恢复中位 **0.428 s** → 仅「可接受」，且 <0.3 s 的比例只有 15%
- 唯一的**「严重」级问题**是 `shooter_log` 的 `kp × 台阶 = 0.381`，超「差」档门槛（0.1）的 **3.8 倍**

#### 6.5.4 症状→原因→修法对照表（文档内置）

- `kp × 台阶 > 0.05` → kp 相对量化台阶过大 → 减小 kp 或提高目标转速
- boost 退出过晚 → 补偿持续时间阈值偏大 → 减小 `boostDurationSec`
- `dropRateThreshold` 误触发 → 阈值低于正常最大减速度 → 取 ≥2~3 倍正常最大减速度
- `ff05` 截距明显非 0 → 存在静摩擦而控制律无 `kS` → 增加常数偏置项

---

### 6.6 本章结论

#### 6.6.1 与 DECODE 的方式对比

| 维度 | DECODE（2025-26） | BIOBUZZ（2026-27） |
|---|---|---|
| 参数来源 | 手工试错，改静态字段重编译 | 5 套无模型辨识 OpMode 自动给出建议值 |
| 需要知道的物理量 | 凭经验（无模型） | 明确**不需要** `R/L/Ke/Kt/J/B` |
| 数据记录 | 无 | CSV（`t,duty,velocity`），每 20 行 flush，记录失败不打断控制 |
| 离线分析 | 无 | Python 687 行 + MATLAB 679 行，同公式双实现 |
| 脚本自检 | 无 | `make_samples.py` 生成 5 组真值已知合成数据，复算值可还原真值 |
| 参数一致性核对 | 无 | `drop04` 报告列出端上值，与离线复算值交叉核对 |
| 判读标准 | 肉眼 | 量化台阶 + 三档验收阈值表 |
| 掉速补偿参数 | 不存在该机制 | 方法四专门标定 4 个参数 |

#### 6.6.2 实测暴露的问题（按严重度排序）

1. **【严重】`kp` 严重超标**：`shooter_log` 反演 `kp ≈ 0.019045`，是一个量化台阶对应 0.381 占空比跳变，报告直接判「严重」。而代码默认 `kp = 0.00005`——**默认值从未被真正标定过**（偏差 381 倍）。
2. **【高】`boostMax / kBoost` 顶死限幅**：13 次真实发射中 **100% 触发 boost，且 boost term 峰值恰好等于 `boostMax = 0.400`**。补偿强度被硬件上限锁死，继续加大 `kBoost` 也无效，只能提高 `boostMax`。
3. **【高】恢复期占空比饱和**：100% 的发射事件恢复期占空比顶到 ≥0.999，说明**恢复速度受母线电压/电机能力物理限制**，PID 增益再大也无用。这是机械/供电问题，不是控制律问题。
4. **【中】恢复时间偏长**：中位 0.428 s，仅 15% 的事件能在 0.3 s 内恢复，未达比赛要求。
5. **【中】`dropThreshold / dropRateThreshold` 与实测不匹配**：`log4` 显示触发帧误差中位 140 tick/s（应 ≥690），未触发帧最大减速度 **−9250 tick/s²**，而默认 `dropRateThreshold = 8000`——**低于正常工况的最大减速度，存在误触发风险**。
6. **【中】`kff` 默认值偏低一倍**：三份日志反演出来的 kff 高度一致（0.00039 / 0.00043 / 0.00044 / 0.00039），而代码默认 `kff = 0.0002`。
7. **【低-中】控制律缺 `kS` 项**：`ff05` 的线性拟合截距稳定非零（`log2` 0.1640、`log3` 0.1258、`log4` 0.2297），说明存在静摩擦，但当前律无常数偏置项，**低转速段误差会偏大**。Guide 第 202 行自己指出了这个缺口。

#### 6.6.3 一句话总结

BIOBUZZ 把 DECODE 的「改代码→编译→上场试→凭手感」流程，换成了「无模型辨识 OpMode 采数据 → 双语言离线复算 → 量化阈值表验收」的闭环；**这套方法学本身是本赛季最有价值的工程改进**。与此同时，实测数据也诚实地证明了：**截至 `202cd58`，`PrototypeShooter` 的参数仍处于「默认值从未按新方法标定」的状态**——`kp` 差 381 倍、`kff` 差一倍、掉速阈值低于实测噪声底、`boostMax` 每次发射都顶死。
## 七、BIOBUZZ 飞轮台架测量与自动整定工具链

> 本节对象为 BIOBUZZ（2026-27 赛季）在 `OpModes/Flywheel/` 下新增的一套**硬件在环（HIL）台架测量 + 跨 OpMode 自动对照整定**工具链，用于替代 2025-26 基线 `Team19725_DECODE` 中"凭经验给增益、靠肉眼看待测值"的整定方式。
>
> **版本状态**：该目录整体 **UNTRACKED in git**。`git status --porcelain` 输出为
> `?? TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Flywheel/`，
> 即下列 **5 个 Java 文件 + `Guide.md` 全部未提交**：
> `FlywheelBench.java`(109 行)、`FlywheelLog.java`(111 行)、`Flywheel1OpenLoopLimit.java`(316 行)、
> `Flywheel2PidVsOpen.java`(359 行)、`Flywheel3ControllerVsPid.java`(448 行)、`Guide.md`(262 行)。
> 当前 HEAD 为 `202cd58`（`Merge branch 'flywheelControlSystemChange' of github.com:BluePowerRobotics/BIOBUZZ`），
> 其上一条为 `b182459 小修`；任务描述中提到的另一提交号 `272398a` 未出现在当前 `git log --oneline -3` 中（未找到，查了 `git log --oneline -3`）。
>
> **共同硬件约定**（三个 OpMode 完全一致）：电机名默认 `"shooter"`，通过 `hardwareMap.get(DcMotorEx.class, motorName)` 绑定；
> 一律 `setDirection(DcMotor.Direction.FORWARD)` + `setMode(RunMode.RUN_WITHOUT_ENCODER)` + `setZeroPowerBehavior(ZeroPowerBehavior.FLOAT)`，
> 因此 **`setPower(p)` 就是占空比指令**，固件不再插一层速度环，`p=0` 是滑行而非刹车。
> 三角色分工：① 无控制器测物理极限 → ② 单 PID vs 开环 → ③ 现有控制律 vs 纯 PID。

### 7.1 数据记录层（两个工具类）

这三个 OpMode **不写 `.jsonl`**。全面检索 `jsonl`（`grep -r "jsonl"` 于仓库根、含 `tools/tuning/` 与 `TeamCode/src/`）**未找到任何 `.jsonl` 写入或解析代码**——日志格式只有 CSV。落盘路径在 Robot Controller 的**外部私有存储**（`context.getExternalFilesDir(null)`，实机为 `/sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/`），可 `adb pull` 取出。

#### `FlywheelLog.java`（111 行）—— 带双电压列的飞轮专用 CSV 记录器

即任务中所指的 file-writing utility（**不是** `TuningLogger`，也**不是**任何 Dashboard-telemetry 类）。

- 类 javadoc（`FlywheelLog.java:11-28`）明确与 `OpModes/Tuning/TuningLogger` 的区别：**多记两列电压**（母线实测值 + 由当前 power 估算的端电压），"因为飞轮极限测试与整定必须同时看电压"。
- 目录与文件命名：`<RC 外部存储>/flywheel/<name>_<System.currentTimeMillis()>.csv`（`FlywheelLog.java:48`、`FlywheelLog.java:53`）；目录创建失败或打开失败时把原因写进 `path` 字段而不是抛异常（`FlywheelLog.java:49-51`、`FlywheelLog.java:60-63`）。
- `FLUSH_EVERY = 20`（`FlywheelLog.java:32`）：每 20 行 flush 一次，"频繁 flush 会拖慢控制循环"。
- `start()` 固定写入的前置注释行（`FlywheelLog.java:57-59`）：
  - `# flywheel debug data (<name>)`
  - `# t(s), power(0~1), velocity(tick/s), busV(V), motorV(V)=busV*power`
  - 随后是调用方传入的表头字符串（实测为 `t,power,velocity,busV,motorV` 或 `...,mode`）。
- 数据行 schema（`row()`，`FlywheelLog.java:89-92`，格式串 `"%.4f,%.4f,%.2f,%.3f,%.3f"`）：

| 列序 | 字段 | 单位 | 精度 |
|---|---|---|---|
| 1 | `t` | s（自 `timer.reset()` 起） | `%.4f` |
| 2 | `power` | 占空比 0~1 | `%.4f` |
| 3 | `velocity` | tick/s（`DcMotorEx#getVelocity()`） | `%.2f` |
| 4 | `busV` | V（母线实测，多 Hub 取**最小值**；无传感器记 `nan`） | `%.3f` |
| 5 | `motorV` | V（**估算** `busV × power`，非实测） | `%.3f` |

- 2/3 号 OpMode 在此 schema 后追加第 6 列 `mode`（`0`=闭环/PID 类，`1`=对照类），见 `Flywheel2PidVsOpen.java:143-144`、`Flywheel3ControllerVsPid.java:172-173`。
- `note(text)` 自动加 `# ` 前缀写注释行（`FlywheelLog.java:84-86`），用于把本次实验参数与结论写进文件头/文件尾（如 `log.note("end")`、`log.note("X: stop")`、`log.note("mode -> PID")`）。
- 所有 `IOException` 被静默吞掉（`FlywheelLog.java:78-80`），"记录失败不应中断控制循环"。

#### `FlywheelBench.java`（109 行）—— 跨 OpMode 静态基准表（本工具链的核心创新）

**为什么能跨 OpMode**：javadoc（`FlywheelBench.java:22-25`）指出 FTC 在同一 App 进程内切换 OpMode **不会重新加载类**，因此 `static` 字段会保留（`PrototypeShooter` 的静态参数也是靠这一点）。所以在一次上电会话里依次跑 ①→②→③，就能累积出一张完整横向对照表；任何阶段都能在遥测里看到它。

- 容量 `MAX_ROWS = 12`（`FlywheelBench.java:34`），超出后丢弃最早一行（FIFO，`FlywheelBench.java:68-71`）。
- 每个基准点固定 6 个数（`FlywheelBench.java:10-18` 定义，`Row` 类在 `FlywheelBench.java:37-49`）：

| 字段 | 含义 | 单位 |
|---|---|---|
| `tag` | 标签（谁、在什么模式下测的） | — |
| `v` | 稳态转速均值 | tick/s |
| `p` | 稳态占空比均值 | 0~1 |
| `sd_v` | 转速标准差（波动大小） | tick/s |
| `sd_p` | 占空比标准差（控制量抖动） | 0~1 |
| `busV` | 母线电压（未知填 `NaN`） | V |

- API：`record(tag,v,p,sdV,sdP,busV)`（`FlywheelBench.java:66-72`，`synchronized`）、`clear()`（`:75`）、`size()`（`:79`）、`telemetryLines()`（`:84-93`）、`last()`（`:96-98`）、`csvLine(Row)`（`:101-108`）。
- 遥测行格式（`FlywheelBench.java:87-90`，`Locale.US`）：
  `%-15s v=%7.0f p=%.4f sd_v=%6.1f sd_p=%.4f busV=%s`，`busV` 为 NaN 时显示 `n/a`，否则 `%.2f`。
- CSV 注释行格式（`FlywheelBench.java:100-107`）：
  `# bench,<tag>,<v 2位小数>,<p 5位小数>,<sd_v 2位>,<sd_p 5位>,<busV 3位>`，空表时为 `# bench,(empty)`，`busV` 为 NaN 时字面量 `nan`。
  该行被写入当次 CSV，供 `tools/tuning/plot_flywheel.py` 解析（该脚本以 `BENCH_PREFIX = "# bench,"` 识别，见 `tools/tuning/plot_flywheel.py:36`、`:65-69`），产出 `flywheel_compare.png` / `flywheel_sweep.png` / `flywheel_bench.png` 三张图（`tools/tuning/plot_flywheel.py:11-13`）。
- 清空方式（`FlywheelBench.java:25`）：按 `Flywheel1` 的 **B** 键，或重启 App。
- 标签命名约定（javadoc `FlywheelBench.java:59`）：建议形如 `"2-PID@2000"` / `"1-limit"`。

#### 对照：`OpModes/Tuning/TuningLogger.java`（102 行，**不在本工具链的调用链上**）

| | `FlywheelLog` | `TuningLogger` |
|---|---|---|
| 目录 | `<外部存储>/flywheel/`（`:48`） | `<外部存储>/tuning/`（`:47`） |
| 表头 | `t,power,velocity,busV,motorV`（可选 `,mode`） | 固定 `t,duty,velocity`（`:60`） |
| 数据行格式 | `%.4f,%.4f,%.2f,%.3f,%.3f`（`:90`） | `%.4f,%.4f,%.2f`（`:84`） |
| 电压列 | 有（母线实测 + 端电压估算） | 无 |
| flush 周期 | 20 行（`:32`） | 20 行（`:32`） |

`TuningLogger` 只被 `OpModes/Tuning/` 下的 5 个整定器引用（`StepResponseTuner.java:65`、`ZieglerNicholsTuner.java:60`、`RelayFeedbackTuner.java:63`、`FeedforwardTuner.java:56`、`DropDetectionTuner.java:57`），**5 个 Flywheel OpMode 全部只用 `FlywheelLog` + `FlywheelBench`**（grep 确认）。

### 7.2 `Flywheel1OpenLoopLimit.java`（316 行）—— 阶段①：无控制器，测极限转速与对应电压

**Driver Station 名称 / 注解**：`@Config`（`Flywheel1OpenLoopLimit.java:42`）+ `@TeleOp(name = "Flywheel1 OpenLoopLimit", group = "Flywheel")`（`:43`）。Dashboard 参数分组即 `Flywheel1OpenLoopLimit`。

**硬件设备名（精确字符串）**：
- 电机：`public static String motorName = "shooter"`（`:47`），绑定处 `hardwareMap.get(DcMotorEx.class, motorName)`（`:276`）。
- 电压：不按名绑定，遍历 `hardwareMap.voltageSensor` 全部 `VoltageSensor` 取**最小正值**；一个都读不到或全 `<= 0` 时返回 `Double.NaN`（`:298-311`）。

**实验步骤**（`runSweep()`，`:123-236`）：
1. 清现场：清 `resultRows`，`maxVel/maxVelPower/maxVelBusV/currentPower/lastPower/lastVel` 全部置 0（`:124-130`）。
2. 开 CSV：`log.start(hardwareMap.appContext, "fly1_openloop", "t,power,velocity,busV,motorV")`（`:132`），随后 `log.note` 写入本次参数 `startPower/powerStep/maxPower/holdSec/settleSec`（`:133-135`）。
3. **起转等待**：先把占空比固定到 `clamp(startPower,0,1)`，持续 `settleSec` 秒，期间每帧记录一行 CSV（`:137-149`）。
4. **逐级扫描**：`for (double p = p0; p <= maxPower + 1e-9 && idx < MAX_STEPS; p += powerStep, idx++)`（`:162`）——即**等差数列占空比扫描，每级保持 `holdSec` 秒**（`:176`）。
5. 每级取**末段 `avgFrac` 比例**的样本做均值/标准差（判据 `if (t >= holdSec * (1.0 - avgFrac))`，`:182-186`），避开加速暂态。
6. 每级结束时与上一级比较算**局部增益** `dKd = (vAvg - lastV) / (power - lastP)`（`:216-218`），即占空比→转速的局部斜率（符号 `dKd` 与 `Kd` 同义）。
7. 全表求最大稳态转速 → `maxVel/maxVelPower/maxVelBusV/maxVelSd`（`:204-209`）。
8. 收尾：`motor.setPower(0)`（`:228`），`log.note` 写 `MAX velocity=... tick/s at power=... (busV=..., motorV≈...)`（`:229-231`），**自动**把极限点记入基准表 `recordBench("1-limit", maxVel, maxVelPower, maxVelSd, 0, maxVelBusV)`（`:233`），再 `log.note("csv=" + log.getPath())`（`:234`）并 `log.close()`（`:235`）。

**精确参数值**（全部 `public static`，Dashboard 实时生效，`:49-62`）：

| 参数 | 行号 | 默认值 | 含义 / 单位 |
|---|---|---|---|
| `motorName` | `:47` | `"shooter"` | 硬件配置电机名（**改后重新 Start 生效**） |
| `startPower` | `:52` | `0.10` | 起始占空比 |
| `powerStep` | `:54` | `0.10` | 每级递增占空比 |
| `maxPower` | `:56` | `1.00` | 扫描占空比上限 |
| `holdSec` | `:58` | `1.50` | 每级保持时长 (s) |
| `avgFrac` | `:60` | `0.40` | 取每级**末段 40%** 时间做平均 |
| `settleSec` | `:62` | `1.00` | 开始扫描前的起转等待 (s) |
| `MAX_STEPS`（`private static final`） | `:65` | `24` | 结果表容量，扫描级数硬上限 |

由默认值推出的扫描序列：`p = 0.10, 0.20, …, 1.00`，共 **10 级**，单级 `1.50 s`，整轮扫描约 `1.00 + 10×1.50 = 16.0 s`（不含后续手动操作时间）。**注意 `avgFrac`/`holdSec` 不参与级数计算**。

**操作**（`:37`、`:85-116`）：Init 后按 Start **自动跑一遍**；`A` 重跑（`runSweep()`，`:104-106`）；`X` 立即 `motor.setPower(0)` 停转（`:107-109`）；`B` 调 `FlywheelBench.clear()` 清空基准表（`:110-112`）；`Y` 把当前这一级的稳态统计写进基准表（`:188-192`，标签 `String.format("1-step@%.2f", power)`，`sd_p` 恒传 `0`）。

**遥测**（`showTelemetry()`，`:245-271`）：三件必输出量合并成一行 `"%.3f / %.0f / %s / %s"` 显示 `power / velocity / busV / motorV`（`:249-253`）；`step`（`stepIndex/stepTotal`，`:254`）；`max velocity`（`:255`）；`-- last steps --` 最近 6 行，每行 `p=%.2f v=%.0f sd=%.1f busV=%.2f dKd=%.1f`（`:222-223`、`:257-261`）；`-- bench: tag / v / p / sd_v / sd_p / busV --` 最近 5 行（`:262-269`）；`CSV` 当前文件路径（`:270`）。全部遥测经 `MultipleTelemetry` **同时推给 Driver Station 与 FTC Dashboard**（`:83`）。

**安全性**：javadoc `:39-40` 明确"全功率时飞轮转速很高，测试前请取下小球与异物；随时可按 X 停转"。电机绑定失败不崩溃，而是循环显示 `!! motor not found` + 失败原因（`:92-99`、`:289-293`），避免误驱动别的机构（`:273`）。

**CSV 数据 schema**：`t,power,velocity,busV,motorV`，其中 `busV` 来自 `readBusVoltage()`，`motorV = bus * power`（`:144`、`:181`）——**端电压是估算值不是实测值**（FTC 无每电机电压传感器）。

### 7.3 `Flywheel2PidVsOpen.java`（359 行）—— 阶段②：单 PID 整定，并与「不用 PID」对照

**Driver Station 名称 / 注解**：`@Config`（`Flywheel2PidVsOpen.java:43`）+ `@TeleOp(name = "Flywheel2 PidVsOpen", group = "Flywheel")`（`:44`）。Dashboard 分组 `Flywheel2PidVsOpen`。

**硬件设备名**：同阶段①——电机 `motorName = "shooter"`（`:48`，绑定 `:321`）；电压遍历 `hardwareMap.voltageSensor` 取最小正值（`:341-354`）。

**实验：两个工作模式的 A/B 对照**（javadoc `:16-25`）。本 OpMode **没有扫描/台阶序列**，是**连续闭环/开环运行 + 手动切换**的对照台：
- **PID 模式**（`usePid = true`）：`duty = kP·e + kI·∫e − kD·dω/dt`，误差 `e = ω* − ω`（`:20`、`pid()` 在 `:221-256`）。
- **OPENLOOP 模式**（`usePid = false`）：`duty = clamp(openLoopPower, dutyMin, dutyMax)` 恒定，完全不闭环——这就是「不用 PID」的对照组（`:21`、`:138-140`）。
- 对比读法（`:23-25`）：把 `openLoopPower` 设成阶段①里达到**同一转速**所需的占空比，两种模式间切换，看 1 秒窗口稳态值 `SS v / SS p` 与速度波动。预期：PID 把稳态误差压得更小，但稳态出现**占空比抖动**（增益相对编码器分辨率过大时尤其明显）。

**精确参数值**（`:47-81`，除 `AVG_N` 外全部 `public static`）：

| 参数 | 行号 | 默认值 | 含义 / 单位 |
|---|---|---|---|
| `motorName` | `:48` | `"shooter"` | 硬件配置电机名 |
| `AVG_N`（`private static final`） | `:51` | `50` | 稳态统计窗口长度（帧，约 **1 s @50 Hz**） |
| `targetVelocity` | `:56` | `2000` | 目标转速 (tick/s)，0=停转 |
| `velocityStep` | `:58` | `100` | 方向键单次调节步长 (tick/s) |
| `maxTargetVelocity` | `:60` | `6000` | 目标转速绝对值上限 (tick/s) |
| `usePid` | `:62` | `true` | `true`=PID 闭环；`false`=开环固定占空比 |
| `openLoopPower` | `:64` | `0.50` | 开环模式固定占空比（建议填阶段①同转速的 `d_ss`） |
| `kP` | `:69` | `0.00005` | 比例增益（占空比/(tick/s)） |
| `kI` | `:71` | `0.00002` | 积分增益（占空比/(tick/s·s)） |
| `kD` | `:73` | `0.0` | 微分增益（占空比·s/(tick/s)）——默认**不开** |
| `iZone` | `:75` | `500.0` | 积分生效误差带 (tick/s)：`\|e\|` 小于该值才积分 |
| `maxI` | `:77` | `0.3` | 积分项限幅（占空比） |
| `dutyMin` | `:79` | `0.0` | 输出下限 |
| `dutyMax` | `:81` | `1.0` | 输出上限 |

**控制律实现要点**（`pid()`，`:221-256`）：
- 微分**作用于测量值而非误差**：`dTerm = -kD * (v - prevVel) / dt`（`:225`），避免目标阶跃瞬间的微分冲击。
- 条件积分抗饱和：仅当 `target != 0 && Math.abs(error) < iZone` 才积分（`:229`），否则 `integral = 0`（`:233`）；输出已顶在饱和方向且误差仍推进时不积分 —— `saturating = (unsat > dutyMax && error > 0) || (unsat < dutyMin && error < 0)`（`:231`）。
- 积分项限幅 ±`maxI` 并**回算内部积分量**（`integral = maxI / kI`，`:239-249`）。
- `dt` 兜底：首帧或卡顿 `dt > 1.0 s` 时按标称 `0.02 s`（`:304-312`）。

**操作**（`:38`、`:110-111`、`handleInput()` `:158-194`）：DPad ↑/↓ 调目标转速（**按住连续调，重复周期 100 ms**，`:189-192`）；`B` 切换 PID ⇄ OPENLOOP（切换时清积分、重置 `prevVel`、清平均窗口，写注释 `mode -> PID`/`mode -> OPENLOOP`，`:165-172`）；`A` 清零积分并**重开 CSV**（`:173-178`）；`X` 立即停转并清积分，写注释 `X: stop`（`:159-164`）；`Y` 记录基准点（`:179-181`）。

**Y 键基准记录**（`recordBench()`，`:204-211`）：窗口样本数 `< 10` 时直接返回；标签格式 `String.format("%s@%d", usePid ? "2-PID" : "2-openloop", targetVelocity)` → 实测形如 `2-PID@2000` / `2-openloop@2000`；调用 `FlywheelBench.record(tag, ssV, ssP, ssSdV, ssSdP, readBusVoltage())` 并把 `FlywheelBench.csvLine(FlywheelBench.last())` 作为注释写进当前 CSV。

**1 秒窗口稳态统计**（`pushAvg()` `:258-265` + `showTelemetry()` `:277-290`）：50 帧环形缓冲分别存 `avgV[]`/`avgP[]`，按 `E[x]`/`E[x²]−E[x]²` 算：

| 遥测字段 | 含义 | 单位 | 格式 |
|---|---|---|---|
| `SS v` | 转速均值 | tick/s | `%.0f` |
| `SS sd_v` | 转速标准差（波动） | tick/s | `%.1f` |
| `SS power` | 占空比均值（控制代价） | 0~1 | `%.3f` |
| `SS sd_p` | 占空比标准差（控制量抖动） | 0~1 | `%.4f` |

显示为 `SS (1s window): v=%.0f±%.1f  power=%.3f±%.4f`（`:289`）。

**遥测**（`:267-301`）：`power / velocity / busV / motorV` 四合一（`:269-273`，含 NaN→`n/a` 保护）；`mode`+`target`+`error`（`:274-275`）；`SS (1s window)`；`PID terms`：`P=%.4f I=%.4f D=%.4f`（`:291`）；`-- bench: ...` 最近 5 行（`:292-299`）；`CSV` 路径（`:300`）。同样经 `MultipleTelemetry` 双推 Driver Station 与 FTC Dashboard（`:108`）。

**CSV**：`log.start(hardwareMap.appContext, "fly2_pid", "t,power,velocity,busV,motorV,mode")`（`:197`），注释头写入 `kP/kI/kD/iZone/maxI/openLoopPower/target`（`:198-200`）。运行中**每帧一行 6 列**（`:143-144`，格式 `"%.4f,%.4f,%.2f,%.3f,%.3f,%d"`），第 6 列 `mode`：`0`=PID、`1`=OPENLOOP（javadoc `:36`），便于离线把两条曲线叠在一起。结束写 `log.note("end")` 并 `log.close()`（`:153-154`）。

### 7.4 `Flywheel3ControllerVsPid.java`（448 行）—— 阶段③：现有控制律 vs 纯 PID

**Driver Station 名称 / 注解**：`@Config`（`Flywheel3ControllerVsPid.java:41`）+ `@TeleOp(name = "Flywheel3 ControllerVsPid", group = "Flywheel")`（`:42`）。Dashboard 分组 `Flywheel3ControllerVsPid`（javadoc `:33`）。

**本 OpMode 与生产代码的关系（关键）**：javadoc `:22-24` 明说 CONTROLLER 分支的实现与
`Controllers/Shooter/PrototypeShooter/PrototypeShooter` 的 `calculate()` **同参数同语义**，
"唯一差别：本 OpMode 通过 Dashboard 的 `motorName` 绑定电机，默认 `"shooter"`"；
并且警告 **"若那边改了控制律，这里要同步改，否则对照失效"**。即本 OpMode 是生产控制律的一具**镜面对照实装**（`calculateFull()` 在 `:256-310`），不是调用关系。

**硬件设备名**：电机 `motorName = "shooter"`（`:46`）；电压遍历 `hardwareMap.voltageSensor` 取最小正值（`:434-447`）。

**实验：两种控制律的同条件 A/B**（javadoc `:16-29`）。同样**没有扫描序列**，是连续运行 + 手动切模式：
- **CONTROLLER**（`useFullController = true`）：`duty = kff·ω* + kp·e + ki·∫e + boost`，boost 由**掉速检测**触发并带"先稳后触"武装规则（`:20-23`、`calculateFull()` `:256-310`）。
- **PID**（`useFullController = false`）：`duty = pidKp·e + pidKi·∫e − pidKd·dω/dt`，**无前馈、无掉速补偿**——即"只用 PID"的对照（`:25-26`、`calculatePid()` `:313-347`）。
- 对比读法（`:28-29`）：两模式整定到同一目标转速后比较 1 秒窗口 `SS v / SS p`、稳态误差与占空比抖动；再**用手指/小球模拟一次掉速**，比较恢复时间与最低点——这正是前馈+掉速补偿存在的意义。

**精确参数值**（`:45-103`）：

| 参数 | 行号 | 默认值 | 含义 / 单位 |
|---|---|---|---|
| `motorName` | `:46` | `"shooter"` | 硬件配置电机名 |
| `AVG_N`（`private static final`） | `:49` | `50` | 稳态窗口帧数（≈1 s @50 Hz） |
| `targetVelocity` | `:54` | `2000` | 目标转速 (tick/s) |
| `velocityStep` | `:56` | `100` | 方向键步长 (tick/s) |
| `maxTargetVelocity` | `:58` | `6000` | 目标转速绝对值上限 (tick/s) |
| `useFullController` | `:60` | `true` | `true`=CONTROLLER，`false`=纯 PID |
| **CONTROLLER 组** | | | |
| `kff` | `:65` | `0.0002` | 稳态前馈增益（占空比/(tick/s)） |
| `kp` | `:67` | `0.00005` | 比例 |
| `ki` | `:69` | `0.00002` | 积分 |
| `iZone` | `:75` | `500.0` | 积分误差带 (tick/s) |
| `maxI` | `:71` | `0.3` | 积分项限幅（占空比） |
| `dropThreshold` | `:75` | `300.0` | 掉速判定误差阈值 (tick/s)：武装条件与触发条件共用 |
| `dropRateThreshold` | `:77` | `8000.0` | 掉速判定加速度阈值 (tick/s²)，判据 `dVelocity < -dropRateThreshold` |
| `kBoost` | `:79` | `0.0002` | 补偿增益（占空比/(tick/s)） |
| `boostMax` | `:81` | `0.4` | 补偿项限幅（占空比） |
| `boostDurationSec` | `:83` | `0.3` | 单次补偿持续时间 (s) |
| **PID 组** | | | |
| `pidKp` | `:88` | `0.00005` | 比例 |
| `pidKi` | `:90` | `0.00002` | 积分 |
| `pidKd` | `:92` | `0.0` | 微分（默认不开） |
| `pidIZone` | `:94` | `500.0` | 积分误差带 (tick/s) |
| `pidMaxI` | `:96` | `0.3` | 积分项限幅（占空比） |
| `dutyMin` / `dutyMax` | `:101` / `:103` | `0.0` / `1.0` | 输出限幅 |

**掉速检测状态机**（`calculateFull()`，`:261-283`）——本节最核心的机制：
1. **武装（arm）**：`Math.abs(error) < dropThreshold` 时 `dropArmed = true`（`:267-269`）。即"只有进入稳态后才允许触发补偿"，否则从 0 拉起时的正常加速段会被误判成掉速（`:261-262`）。
2. **换目标清除武装**：`if (target != lastTargetVel) { dropArmed = false; lastTargetVel = target; }`（`:263-266`）。
3. **触发**：`dropDetected = dropArmed && target != 0 && (error > dropThreshold || dVelocity < -dropRateThreshold)`（`:271-272`）。两条判据是**或**关系：误差突增 **或** 加速度骤降。
4. **动作**：`boostActive = true; boostTimer = 0; integralFull = 0`（同时**清积分**）（`:273-277`）。
5. **退出**：`boostTimer += dt`，当 `boostTimer >= boostDurationSec || error <= 0` 时解除（`:278-283`）。
6. **补偿量**：`termBoost = boostActive ? clamp(kBoost*error, 0, boostMax) : 0`（`:307`）——注意**只正向、且以 0 为下界**（不产生反向补偿）。
7. 前馈/比例项：`termFF = kff * target`、`termP = kp * error`（`:304-305`）；输出 `clamp(termFF+termP+termI+termBoost, dutyMin, dutyMax)`（`:309`）。

`calculatePid()`（`:313-347`）与阶段②同构（微分作用于测量值 `:318`、条件积分+抗饱和 `:323-329`、积分限幅回算 `:332-342`），但增益用独立的 `pid*` 组，且 **`termFF`/`termBoost` 恒置 0**（`:315`、`:320`）。

**操作**（`:31-32`、`handleInput()` `:186-217`）：DPad ↑/↓ 调目标（**按住连续调，重复周期 100 ms**，`:212-215`）；`B` 切换 CONTROLLER ⇄ PID（`resetStates()` + 注释 `mode -> CONTROLLER`/`mode -> PID`，`:193-197`）；`A` 复位状态并重开 CSV（`:198-201`）；`X` 立即停转（`:187-192`，写 `X: stop`）；`Y` 记录基准点（`:202-204`）。`resetStates()`（`:219-230`）清 `integralFull`/`integralPid`、以当前实测转速重置两个 `prevVel`、清 `boostActive`/`boostTimer`/`dropArmed`、`lastTargetVel = NaN`、并清空平均窗口。

**Y 键基准记录**（`recordBench()`，`:242-250`）：`avgCount < 10` 直接返回；标签 `String.format(Locale.US, "%s@%d", useFullController ? "3-controller" : "3-PID", targetVelocity)`（`:246-247`）→ 实测形如 `3-controller@2000` / `3-PID@2000`；随后 `FlywheelBench.record(tag, ssV, ssP, ssSdV, ssSdP, readBusVoltage())` 并把 `FlywheelBench.csvLine(...)` 写为注释（`:248-249`）。

**遥测**（`showTelemetry()`，`:358-392`）：`power / velocity / busV / motorV` 四合一（`:360-364`）；`mode`：`%s   target=%d   error=%.0f   boost=%s`（`:365-366`）；`SS (1s window)`：`v=%.0f±%.1f  power=%.3f±%.4f`（`:380`）；**比阶段②多一项分量分解** `terms FF/P/I/boost`：`%.4f / %.4f / %.4f / %.4f`（`:382`），字段来源即 `termFF/termP/termI/termBoost`——用于直接看出掉速瞬间是谁在出力；`-- bench: tag / v / p / sd_v / sd_p / busV --` 最近 5 行（`:383-390`）；`CSV` 路径（`:391`）。初始化画面另打印 `Stage 3: full controller (FF+PI+boost) vs plain PID` 与操作说明（`:143-144`）。遥测同样经 `MultipleTelemetry` 双推 Driver Station 与 FTC Dashboard（`:141`）。未绑电机时循环显示 `!! motor not found` + 排查提示 + `bindError`（`:425-427`）。

**CSV**：`log.start(hardwareMap.appContext, "fly3_ctrl", "t,power,velocity,busV,motorV,mode")`（`:233`），参数注释行一次写全两套增益（`:234-238`）：
`CONTROLLER kff=%.8f kp=%.8f ki=%.8f maxI=%.3f iZone=%.1f dropTh=%.1f dropRate=%.1f kBoost=%.8f boostMax=%.3f boostDur=%.2f | PID pidKp=%.8f pidKi=%.8f pidKd=%.8f target=%d`。
每帧一行 6 列（`:172-173`），第 6 列 `mode`：**`0`=CONTROLLER、`1`=PID**（javadoc `:37`）——注意与阶段②的 `mode` 编码**语义不同但数值约定一致**（0=闭环主模式、1=对照组）。结束时 `motor.setPower(0)` + `log.note("end")` + `log.close()`（`:181-183`）。

### 7.5 关联测试台（非 Flywheel 包，但同属飞轮整定链路）

#### `OpModes/PrototypeShooterTest.java`（93 行）—— 生产控制律的手动遥测台

- **注解 / 名称**：`@Config`（`PrototypeShooterTest.java:27`）+ `@TeleOp(name = "PrototypeShooterTest", group = "Tests")`（`:28`）。
- **测试内容**：不跑任何自动序列，只做"**用手柄给目标转速、在 Dashboard 上看速度环跟随**"（javadoc `:12-13`）。硬件来自 `new PrototypeShooter(hardwareMap, telemetry)`（`:52`）——即**生产控制类**，电机名由 `PrototypeShooter` 内部决定（本文件不出现 `hardwareMap.get`）。
- **精确参数**（`:31-38`）：`MaxTargetVelocity = 3000` (tick/s)、`velocityStep = 100` (tick/s)、`repeatPeriodMs = 100` (ms)。
- **操作**：DPad ↑/↓ 调目标（首次按下立即走一步，按住按 `repeatPeriodMs` 连续调，`:69-79`）；`A` 立即停转（`:64-66`）。
- **整定指引**（javadoc `:21-23`）：在 Dashboard 的 `PrototypeShooter` 分组实时改 `kff / kp / ki / iZone / maxI / dropThreshold / kBoost`，观察上升时间、超调量、稳态误差——**参数集与 `Flywheel3ControllerVsPid` 的 CONTROLLER 组一一对应**。
- **日志**：**无文件写入**，只有 `MultipleTelemetry` 双推（`:50`）+ `shooter.setTelemetry()`（`:58`、`:82`）。

#### `OpModes/ShooterTuningTester.java`（464 行）—— 四模式自动整定台（本工具链的"全自动"变体）

- **注解 / 名称**：`@Config`（`ShooterTuningTester.java:44`）+ `@TeleOp(name = "ShooterTuningTester", group = "Tests")`（`:45`）。
- **定位**（javadoc `:17`）："飞轮 PID/SVA 整定测试台 —— 按『**先前馈、后反馈**』流程在实机上整定 `PrototypeShooter` 的速度环"。它**复用生产类**（`import ...Controllers.Shooter.PrototypeShooter.PrototypeShooter`，`:11`；`shooter = new PrototypeShooter(hardwareMap, telemetry)`，`:140`）。
- **四种模式**（`:82-93`，`MODE_KS=0 / MODE_KV=1 / MODE_STEP=2 / MODE_DISTURB=3`，`MODE_COUNT=4`）：

| 模式 | 名称与开环/闭环 | 做什么 | 产出 |
|---|---|---|---|
| 0 | `kS calib (open loop)` | 功率从 0 以 `rampRate` 缓升，转速首次 ≥ `spinDetectThreshold` 即记该功率为 `suggestedKS`（`:275-285`） | 建议 kS（`%.4f`） |
| 1 | `kV calib (open loop)` | 依次施加 `powerSteps` 各档功率，每档等平稳（相邻 0.5 s 窗口转速差 < `settleTolerance`）或超时 `maxStepWait`；累积 `{v,p}` 样本后**线性回归 `P = kS + kV·v`**（`:344-345`） | 建议 kV（`%.6f`）+ 拟合 kS |
| 2 | `step response (closed loop)` | 在 0 与 `stepTarget` 间自动往复，半周期 `cycleTime`；`trackStep()` 统计 | 90% 上升时间 / 超调量 % / 整定时间 |
| 3 | `disturbance (closed loop)` | 保持 `disturbTarget`，**人工投球**制造扰动；跌落超过 `dipDetect` 视为扰动发生（`:413-423`） | 跌落深度 tick/s + 恢复时间 s |

- **精确参数**（`:48-78`）：`rampRate = 0.01`（功率/秒）、`spinDetectThreshold = 60` (tick/s)、`rampPowerLimit = 1.0`、`powerSteps = {0.25, 0.40, 0.55, 0.70}`、`settleWindow = 0.5` (s)、`settleTolerance = 40` (tick/s)、`maxStepWait = 8.0` (s)、`stepTarget = 2500` (tick/s)、`cycleTime = 4.0` (s)、`disturbTarget = 2500` (tick/s)、`dipDetect = 300` (tick/s)。
- **操作**（javadoc `:19-25`、`handleButtons()` `:180-202`）：DPad ←/→ 切模式（切换时自动停车并**保留已得结论**，`:183-186`）；`A` 启动/停止当前模式流程；`B` 清数据与结论并停车；`Y` 紧急停转。模式 0/1 走 `PrototypeShooter#setOpenLoopPower(double)` **绕过闭环**，模式 2/3 为闭环（javadoc `:38-39`）。
- **遥测**（`setReportTelemetry()` `:430-453`）：`Mode`（模式提示串）；`>>> Suggested kS`（`%.4f`）；逐条 `Sample i`；`>>> Suggested kV`（`%.6f`）+ `>>> Suggested kS (fit)`（`%.4f`）；`Step rise(90%)`（`%.2f s`，未达显示 `-`）、`Step overshoot`（`%.1f %%`）、`Step settle`（`%.2f s`，未稳显示 `not settled`）；`Last disturbance`：`dip=%.0f tick/s, recovery=%.2f s`；`Battery`：`%.2f V`。随后叠加 `shooter.setTelemetry()`（`:170`）。
- **日志**：**无文件写入**（grep `TuningLogger`/`FlywheelLog`/`.jsonl` 在本文件**未找到**），结论只走遥测。
- **收尾规范**（javadoc `:42`）："整定完成后按规范将结果**硬编码回 `PrototypeShooter` 的静态字段**"——即测试台不自动写参数，人工回填。

### 7.6 依赖关系：三个 Flywheel OpMode 谁都不依赖

三个 Flywheel OpMode 的 **import 全表**（已逐一核对）只有 10 项：`FtcDashboard`、`dashboard.config.Config`、`MultipleTelemetry`、`LinearOpMode`、`TeleOp`、`DcMotor`、`DcMotorEx`、`VoltageSensor`、`ElapsedTime`、`java.util.Locale`（`Flywheel1`/`Flywheel2`/`Flywheel3` 的 `:3-15` 行区间）。
- **对 `PIDSVAController` / `SlotConfig` / `MotorPIDCore`：零依赖**。这三个类位于 `utility/PID/`（`PIDSVAController.java`、`SlotConfig.java`、`MotorPIDCore.java`），当前只被 `Controllers/Turret/YawController.java:9-10`、`Controllers/Turret/Shooter.java:10-11`、`Sweeper.java` 的注释引用（grep 全仓库确认）。**Flywheel 三个 OpMode 全部自写控制律，不走 `utility.PID` 的插槽式控制器**。
- **对 `PrototypeShooter`（生产类）：零依赖**。三个 Flywheel OpMode 都**不 import、不 new `PrototypeShooter`**，而是 `new MultipleTelemetry(...)` 后直接在 `runOpMode()` 里 `hardwareMap.get(DcMotorEx.class, motorName)` 自绑电机、自算占空比。反观 `PrototypeShooterTest.java:52` 与 `ShooterTuningTester.java:140` 是 `new PrototypeShooter(hardwareMap, telemetry)`——**两条链路互不调用**。
- **唯一的"语义耦合"**：`Flywheel3ControllerVsPid.calculateFull()`（`:256-310`）与 `PrototypeShooter.calculate()` **同参数同语义**，靠 javadoc `:22-24` 的**人工同步约定**维持，编译器不保证。这是本工具链的一个**显式风险点**（javadoc 原话："若那边改了控制律，这里要同步改，否则对照失效"）。
- **三个 Flywheel OpMode 之间的依赖**：只有 **`FlywheelLog` 与 `FlywheelBench` 两个静态工具类**（`120` 行 + `109` 行）。三者都 `new FlywheelLog(...)` 并调用 `FlywheelBench.record/telemetryLines/clear/csvLine`。

**运行流转（一次上电会话内）**：
`Flywheel1 OpenLoopLimit` → 自动扫 10 级得极限点，自动 `FlywheelBench.record("1-limit", …)` → 人工按需 `B` 清表或保留 → 切 OpMode 到 `Flywheel2 PidVsOpen`，把 `openLoopPower` 设为阶段①中同转速的占空比，`B` 键切 PID/OPENLOOP 对照，`Y` 记录 `2-PID@v` / `2-openloop@v` → 切到 `Flywheel3 ControllerVsPid`，`B` 键切 CONTROLLER/PID，`Y` 记录 `3-controller@v` / `3-PID@v`，再用手指/小球模拟掉速看 boost 行为（javadoc `:29`）。
**跨 OpMode 不丢数据的原因**：FTC 同一 App 进程内换 OpMode **不重载类**，`FlywheelBench` 的 `static` 表得以保留（`FlywheelBench.java:22-25`）；上电重启或按 `Flywheel1` 的 `B` 键才清空。
**与 Planned/speculative 的区分**：`Flywheel1` 的 10 级 × 1.5 s 扫描时长（约 16 s）是按默认参数**推算**，非文件中断言；`Guide.md` 中的预期结论未在本节引用，需以实测 CSV 为准。

### 7.7 配套文档与离线数据管线（同属未提交的工作集）

`OpModes/Flywheel/Guide.md`（262 行，**同样 UNTRACKED**）是这套台架的**操作主线**，其 git 未跟踪状态与 5 个 Java 文件一致（`git status --porcelain` 只报一行目录 `?? TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Flywheel/`，目录内 6 个文件全部未提交）。

- 文档结构（各节标题行号）：`# 飞轮调试与参数整定指南（按调试顺序）`（`:1`）、`## 0. 三阶段总览`（`:9`，含一张"阶段 / OpMode（Driver Station 名）/ 有没有控制器 / 要回答的问题 / 记入基准表的标签"五列对照表）、`## 1. 准备（每次上车前）`（`:34`）、`## 2. 阶段①`（`:50`）、`## 3. 阶段②`（`:77`，含 `3.1 SS v / SS p 是什么` `:81`、`3.2 PID 在控制什么` `:101`、`3.3 整定步骤` `:112`、`3.4 与「不用 PID」对比` `:128`）、`## 4. 阶段③`（`:139`，含 `4.1 控制律结构（为什么不是纯 PID）` `:143`、`4.2 整定顺序（严格按"先前馈、后反馈、最后抗扰"）` `:156`、`4.3 与纯 PID 对比` `:171`）、`## 5. 三阶段横向对比（基准表）`（`:182`）、`## 6. 参数速查表`（`:212`）、`## 7. 排查表`（`:235`）、`## 8. 与其它工具/文档的关系`（`:251`）。
- **阶段①的产出被下游复用的显式链条**（`Guide.md:73`）："把「设计发射转速 `ω*` 对应的稳态占空比 `d_ss`」记下来——**这是阶段③前馈增益 `kff` 的来源**，也是阶段②开环对照的 `openLoopPower`"。
- **前馈定值公式**（`Guide.md:160`）：`kff = d_ss / ω*`（阶段①的稳态占空比），或用 `Tune6.5 Feedforward` 多点回归取斜率；判据是"稳态时 I 项接近 0、P 项接近 0"。`Guide.md:223` 等价写作 `kff = d_ss/ω* = 1/Kd`。
- **`kp` 起点**（`Guide.md:161`）：用阶段②确定的 `kP`，前馈已承担稳态后 `kp` 可以比纯 PID 小。
- **`dropThreshold` 定值规则**（`Guide.md:164`）：`k_σ·σ_ω`，`σ_ω` 取阶段②稳定后的 `SS sd_v`，`k_σ = 3~5`；判据是"正常波动不误触发"。
- **实机教训（量化）**（`Guide.md:125`）：若 `Δduty/Δω ≳ 0.02`（转速只跳一格、占空比就跳 2% 以上），稳态必然抖，应减小 `kP`、加低通滤波、或接受更大稳态误差；并指出"这与交接说明里实机日志的结论一致（那里是 **20 RPM 台阶 × kP = 0.276**，占空比在 **0.6~1.0** 间横跳）"。注意这里 `kP = 0.276` 是**历史实机日志中的单位制**，与本节 `kP = 0.00005`（占空比/(tick/s)）**不是同一量纲**，不可直接比较。
- **离线出图**：`tools/tuning/plot_flywheel.py`（**UNTRACKED**，同批新增）以 `BENCH_PREFIX = "# bench,"` 识别基准行（`tools/tuning/plot_flywheel.py:36`、`:65-69`），扫描 `flywheel/` 目录下全部 `*.csv`（`:310`），产出三张图（`:11-13`、`:205-207`）：`flywheel_compare.png`（速度/占空比/电压时间序列，按 `mode` 分色跨文件叠加）、`flywheel_sweep.png`（阶段① 速度-功率 + 电压-功率，标出极限点）、`flywheel_bench.png`（基准表 `v / sd_v / p` 并排柱状图）。文件头（`:52`）说明其内部结构为 `bench: list = field(default_factory=list)  # [(tag, v, p, sdV, sdP, busV)]`——与 `FlywheelBench.Row` 的 6 字段一一对应，构成"Java 写入 → Python 读取"的**线上契约**。
- **同批新增的相邻脚本**（`tools/tuning/`，均 UNTRACKED）：`analyze_dashboard_log.py`（解析 Driver Station 导出 CSV，文档头 `:4-8` 记录了两条既有教训：① `logs` 列含多行引号文本须用 csv 模块解析；② **`time` 列毫秒字段是坏的**——同一秒内会从 ~900 跳回 ~100，按它排序会伪造锯齿暴跌，因此按文件行序重建均匀时间轴 `dt = span/(n-1)`；并报出速度**量化台阶**）、`analyze_launch_events.py`、`check_plot_data.py`、`export_clean_table.py`、`转速图分析指南.md`、`_hypdiff.txt`。

### 7.8 本节要点与风险标记

1. **本工具链的独创点**是 `FlywheelBench` 这张**跨 OpMode 静态表**——利用"同一 App 进程内换 OpMode 不重载类"的 FTC 运行时特性（`FlywheelBench.java:22-25`），把三个阶段、两种控制律的稳态指标累积到同一张表里，再用 `Y` 键逐点落入 CSV 注释行交给 `plot_flywheel.py` 出横向对比图。DECODE 基线没有对应机制（未找到——查了 `FlywheelBench`/`flywheel` 关键字在全仓库的引用）。
2. **量纲一致性是最大的使用陷阱**：三个阶段全部用 **tick/s** 与 **占空比 0~1**，但 `Guide.md:125` 引用的历史实机日志用 RPM 与另一套 kP 标度（`kP = 0.276`）。
3. **电压是"母线实测 + 端电压估算"**：`motorV = busV × power`（`FlywheelLog.java:23`、三个 OpMode 的记录行），**不是实测端电压**——FTC 无每电机电压传感器，做功率结论时须知悉这一拟合。
4. **`.jsonl` 不存在**：全仓库检索 `jsonl` 未找到任何写入或解析代码；本工具链的持久化格式**只有 CSV + `#` 注释行**，通过 RC 外部存储 `flywheel/` 目录 + `adb pull` 取出。
5. **参数生效时机**：`motorName` 等被 `@Config` 暴露的 `public static` 字段改后需**重新 Start**（三个 OpMode 的 `motorName` 行注释均如此声明）；而 PID 增益类字段在 Dashboard 上**实时生效**（因为每帧从静态字段重读）。
6. **未找到项汇总**：`272398a` 未出现在 `git log --oneline -3`；`.jsonl` 全仓库未找到；`FlywheelBench` 在 `DECODE` 基线中的对应物未找到；三个 Flywheel OpMode 调用 `TuningLogger`/`PIDSVAController`/`SlotConfig`/`MotorPIDCore`/`PrototypeShooter` 的证据**均未找到**（grep import 表 + 全仓库符号检索）。
## 八、总对比与结论建议

> 前七章按「范围 → 时间线 → DECODE 基线 → BIOBUZZ 新架构 → 7 月过渡期 → 调参方法学与实测 → 飞轮台架工具链」逐层展开。本章做横向收束：把六个阶段的控制律放在一条线上，把每一项能力的「首创者」标清楚，然后给出结论与按优先级排序的建议。

### 8.1 六个阶段的控制律演进主线

| 阶段 | 时间窗 | 代表提交 | 飞轮速度环 | yaw 控制 | 前馈 | 掉速补偿 | 调参手段 |
|---|---|---|---|---|---|---|---|
| **A. DECODE 正赛季** | 至 2026-06 前 | `328a6c3`（DECODE master） | 单一 PID，**两套静态增益按 750 °/s 硬切换**；dt 用毫秒 | 无 | 无 | 无 | 手改静态字段 + 重新部署 |
| **B. 休赛期 OffSeasonGame** | 2026-06 ~ 08-18 | `f8bda9c`（OffSeason master） | 单一 PIDSVA slot；**秒制 dt**；kS/kV/kA 前馈 | `TurretDegreeController` 角度域**位置闭环**，但 **yaw 纯开环**（源码自述舵机不回报位置） | S/V/A 电压前馈 | 无（仅 `VelocityTolerance = 20`） | 手改 + Dashboard |
| **C. BIOBUZZ 九月** | 2026-09-22 ~ 09-30 | `2936c21` → `9f05ad2` → `ef4b120` | `PrototypeShooter`（PIDSVA，功率量纲） | 无 | S/V/A | 无 | `ShooterTuningTester`（464 行，4 模式半自动） |
| **D. BIOBUZZ 反冲架构成形** | 2026-10-01 | `550ba4d` → `5996db3` | 不变 | **`YawController` 位置环首次带上到位判据** | S/V/A（位置环由 setpoint 差分推导） | 无 | PIDSVA 加 7 项分量诊断 |
| **E. 控制律断点** | **2026-10-11** | **`1937409`**（+1915/−0） | **`PrototypeShooter` 弃用 PIDSVA，改为自包含占空比律**<br>`duty = Kff·ω* + Kp·err + Ki·∫err + Vboost` | 不变 | 静态 `Kff·ω*`（占空比量纲，**不读电压传感器**） | **首次出现**：掉速检测 + 限时 boost + 触发清积分 | **5 套无模型标定 OpMode**（`OpModes/Tuning/`，共 1077 行 + Guide 232 行） |
| **F. arming 修正 + 工具链提交** | **2026-10-15** | **`b182459`**（+1760/−71） | 控制律不变，加**武装规则** `dropArmed` / `lastTargetVel` | 不变 | 不变 | 修正「0→转速加速段被误判为掉速」的缺陷 | `tools/tuning/` 三脚本 + README 进入版本库 |
| **G. 未提交工作区** | 2026-10-16 检查时点 | 无提交 | 不变 | 不变 | 不变 | 不变 | **三级飞轮台架**（`OpModes/Flywheel/`，5 Java 共 1343 行 + Guide 262 行）+ **7 个离线脚本** + **4 组实测日志与 25 个分析产物** |

**读法**：真正被改写的只有三处——**B 阶段的 dt/前馈改革**（把 DECODE 的毫秒当秒、无前馈修好）、**E 阶段的控制律替换**（把 PIDSVA 换成自包含占空比律并引入掉速补偿）、**F 阶段的 arming 修正**（补上前一版逻辑漏洞）。C、D 两阶段是**架构扩张**（从单速度环长成三执行器子系统），不是控制律变化。

### 8.2 每项能力的首创归属（谁先做的）

| # | 能力 | DECODE 正赛季 | OffSeasonGame（7 月） | BIOBUZZ | 首创 |
|---|---|---|---|---|---|
| 1 | 秒制 dt（而非毫秒） | ✗ | ✓ | ✓ | OffSeason |
| 2 | S/V/A 前馈项 | ✗ | ✓ | ✓ | OffSeason |
| 3 | PIDSVA + 多 slot + iZone | ✗ | ✓（`PIDSVAControllers` 复数包，2026-06-10/11 诞生，7 月零改动） | ✓（`utility/PID`） | **OffSeason**（BIOBUZZ 在 `MotorExamples\PIDSVAControllers\` 里继承） |
| 4 | 位置环由 setpoint 差分推导参考速度/加速度 | — | 有分支但喂 0.0（死代码） | ✓（`PIDSVAController.java:128/131`） | BIOBUZZ |
| 5 | 7 项 lastXTerm 分量诊断 | ✗ | ✗ | ✓（`:160-166`） | BIOBUZZ |
| 6 | slot 未配置快速失败（抛异常） | ✗ | ✗ | ✓（`:119`） | BIOBUZZ |
| 7 | 输出限幅语义从 ±14 V 改为 ±1（功率） | ✗ | ✗ | ✓（`SlotConfig.java:50/52`） | BIOBUZZ |
| 8 | 炮台三执行器架构（yaw 电机 + 俯仰舵机 + 扳机舵机） | ✗（只有飞轮） | ✗ | ✓（`Turret.java` 129 行） | BIOBUZZ |
| 9 | yaw 闭环 + 到位判据 | ✗ | ✗（yaw 恒等于目标，判据永真） | ✓（`YawController.java` 187 行） | BIOBUZZ |
| 10 | 到位联锁（yaw 未到位则禁止发射） | ✗ | ✗ | ✓（`Turret.java:106-114`） | BIOBUZZ |
| 11 | 实例级参数（`Params` / `SlotConfig`） | ✗（类内静态裸字段） | 部分 | ✓ | BIOBUZZ |
| 12 | 掉速检测 + 限时 boost 补偿 | ✗ | ✗ | ✓（`PrototypeShooter`） | **BIOBUZZ 独有** |
| 13 | 掉速检测 arming 规则 | ✗ | ✗ | ✓（`b182459`） | **BIOBUZZ 独有** |
| 14 | 无模型标定方法论（阶跃 / ZN / 继电 / 掉速 / 前馈） | ✗ | ✗ | ✓（5 OpMode） | **BIOBUZZ 独有** |
| 15 | CSV 数据记录 + 离线复算工具链 | ✗ | ✗ | ✓（双语言 687+679 行） | **BIOBUZZ 独有** |
| 16 | 量化台阶意识 + 三档验收阈值表 | ✗ | ✗ | ✓（`转速图分析指南.md`） | **BIOBUZZ 独有** |
| 17 | 飞轮台架三级对照实验 | ✗ | ✗ | ✓（`OpModes/Flywheel/`） | **BIOBUZZ 独有**（未提交） |
| 18 | 自动瞄准 / 射程—转速映射 | 有线性回归 `speed = 2.010562318·distance + 540.4157073`（另有 Dashboard 拟合 `12.7·distance + 627`，未回写默认值）；`TurretAimingCalculator` 是 TODO 占位且已随 `rubbishbin` 退役 | 有 `speed = SPEED_A·distance + SPEED_B`（默认 10/800）与 `FIXED_YAW = 45`；`Turret_2` 走 `BSTsolver.predict(vx,vy,dx,dy)` | `utility/BST` 提供「静止标定滞空时间 → 飞轮目标转速」模型（run-and-gun 简化变体，去掉炮台只做径向解析解）；`Turret.java` 的 `RelPosBig`/`RelPosSmall` 已定义但**当前未使用**；`AutoAimPrompt.md` 仍是需求文档 | 三条并行路线，尚未统一 |

### 8.3 结论

1. **控制律的真正断点是 `1937409`（2026-10-11），不是新赛季开始时。**
   9 月 22 日到 10 月 1 日这段时间，BIOBUZZ 虽然从零建起了炮台子系统，但飞轮速度环用的还是 OffSeason 那套 PIDSVA。`1937409` 一次性把 `PrototypeShooter` 从「PIDSVA + 功率输出」换成「自包含占空比律 + 掉速补偿」，并同时引入 5 套标定 OpMode——**控制律与方法学是在同一个提交里一起换的**。

2. **架构断点在 `ef4b120`（2026-09-30）。**
   `Controllers/Turret/{Shooter,Turret,YawController}.java` 三件套在这一天从零出现，把 DECODE 时代的「一个速度环」扩成「一个 yaw 电机 + 两个发射器（各含飞轮 + 俯仰舵机 + 扳机舵机）」共 5 个执行器。其中 `YawController.java` 修掉了 7 月 `TurretDegreeController` 的核心缺陷——那份代码的源码注释自己写着「舵机不会以任何电气方式实时读取自己的位置并返回，所以 `currentYaw` 即 `yawTargetServo`」，导致 yaw 到位判据**恒真**、`getAimError()` 的 yawError **恒为 0**。BIOBUZZ 用带编码器的电机 + PIDSVA 位置环替换了它。

3. **方法学断点是 `1937409` + `b182459` 的组合，这是本赛季最有复用价值的产出。**
   DECODE 的调参是「改代码 → 编译 → 上场试 → 凭手感再改」，没有一次数据记录。BIOBUZZ 建立了「端上 5 套无模型辨识 OpMode 采数据 → 离线 Python/MATLAB 双实现复算 → 量化台阶意识 + 三档验收阈值表判定」的闭环，并且用 `make_samples.py` 做了脚本自检（真值可还原）。**这条链路不依赖任何赛题硬件，可以整建制移植到下一个赛季。**

4. **但实测诚实地证明：截至 `202cd58`，`PrototypeShooter` 的参数从未按这套新方法标定过。**
   逐条对照（详见 6.4、6.6.2）：

   | 参数 | 代码默认值 | 实测反演/要求值 | 偏差 |
   |---|---|---|---|
   | `kp` | `0.00005` | **`0.00182`（log2/log3/log4 三次独立反演一致）**；`shooter_log` 更高达 `0.019045` | 默认值比实测低 **36 倍**（对 `shooter_log` 低 **381 倍**） |
   | `kff` | `0.0002` | `0.00039 ~ 0.00044`（三份日志高度一致） | 默认值低约 **一半** |
   | `ki` | `0.00002` | `0.0022 ~ 0.0030`（段内拟合中位数） | 低约 **2 个数量级** |
   | `dropThreshold` | `300` | `log4` 触发帧误差中位 **140**，建议 ≥ **690** | 偏低 |
   | `dropRateThreshold` | `8000` | `log4` **未触发帧最大减速度 −9250 tick/s²** | **低于正常工况噪声底，存在误触发风险** |
   | `boostMax` | `0.4` | 13 次真实发射中 boost 峰值**恰好 = 0.400，100% 顶死限幅** | 已饱和 |
   | `kS`（静摩擦项） | **不存在该项** | `ff05` 线性拟合截距稳定非零（0.126 / 0.164 / 0.230） | 控制律缺项 |

5. **有一个改进是纯粹靠数据对比才做出来的，值得单独记一笔。**
   `log2` 与 `log3` 用**完全相同的增益**（反演出的 `kp` 都是 `0.00182000`），但 `log2` 的 duty 标准差是 `0.0790`、极限环 51 ms / 1.50 台阶、**掉速补偿触发率 21.1%**；`log3` 是 `0.0182`、28 ms / 0.50 台阶、**触发率 0.0%**——**4.4 倍的稳态噪声差异**。触发时的误差中位数只有 30 tick/s，而与真实掉速无关。这就定位出了「从 0 拉起转速的正常加速段被误判为掉速」的缺陷，直接催生了 `b182459` 的 arming 规则。**如果没有这套日志体系，这个 bug 只会表现为「上场打不准」。**

6. **发射恢复的瓶颈已经被确认为物理瓶颈，不是控制问题。**
   `log4` 的 13 次真实发射中，**100% 的事件恢复期占空比顶到 ≥0.999**——即前馈 + 反馈已经把输出推到电机/母线电压能力的上限。报告原文的判定是：「恢复速度受母线电压/电机能力限制，**加大增益没有用**」。恢复时间中位 0.428 s、最长 0.642 s，仅 15% 的事件能在 0.3 s 内恢复。配合 `2250 tick/s` 段 74.9% 的饱和占比，**当前电机 + 供电组合的速度天花板已经摸到**。

7. **稳态精度本身是达标的。** 最好的稳态出现在 `log4` 的 1750 tick/s 段：速度 sd 11.0（**0.55 个量化台阶**）、误差 −0.7（**0.04%**）、duty sd 0.020、饱和 0%。1950 / 2050 / 2250 三段同样是 sd 10.2~10.8（0.51~0.54 台阶）、误差 ≤0.11%。**按 `转速图分析指南.md` 的三档验收表，稳态项落在「好」档。** 问题集中在「增益标定值错」与「发射恢复」两处，不在稳态。

8. **版本库与工作区存在明显脱节，是本报告必须提示的风险。**
   - `OpModes/Flywheel/`（三级台架，5 Java + Guide，共 1605 行）**不在任何提交里**
   - `tools/tuning/` 的 7 个离线脚本 **全部未提交**（`analyze_dashboard_log.py` 533 行、`analyze_launch_events.py` 272 行、`plot_flywheel.py` 364 行、`plot_speed.m` 415 行、`check_plot_data.py` 138 行、`export_clean_table.py` 119 行、`转速图分析指南.md` 133 行）——**只有 `README.md` / `analyze_tuning.py` / `analyze_tuning.m` / `make_samples.py` 四个被 `b182459` 提交**
   - `tools/tuning/out/` 的 4 组实测日志与 25 个分析产物（含全部报告与 PNG）**未提交**——这恰恰是证明参数标定偏差的唯一证据
   - 也就是说：**如果有人新克隆仓库，他拿到的是完整的标定方法论框架，但拿不到任何一条调参结论、也没有执行调参的台架程序。**

### 8.4 建议（按优先级）

**P0 — 按已建立的方法重新标定 `PrototypeShooter` 参数（半天工作量）**
1. 跑 `Tune6.1 StepResponse`，用离线 `analyze_tuning.py`（最小二乘 τ）取 `kff / kp / ki`，填入代码。
2. 用 `Tune6.2` 或 `Tune6.3` 复核，**取较保守者**（Guide 6.6 明确要求）。
3. 跑 `Tune6.5 Feedforward` 多点回归修正 `kff`。
4. 预期收益：`kp` 从默认值提高 36 倍后，才真正对得上实测反演；`kff` 提高一倍后，前馈承担更多稳态占空比，P 项的负担与量化跳变随之减小。
5. **验收**：稳态 duty sd ≤ 0.05（最好 ≤ 0.02），`kp × 一个台阶 ≤ 0.05`。

**P1 — 修正掉速判定阈值与补偿强度（依赖 P0）**
1. 跑 `Tune6.4 DropDetection`，先确认量化台阶与 σ 相当于几个台阶。
2. `dropThreshold` 取 ≥ **690 tick/s**（`log4` 触发帧误差中位 140 的 5 倍，按 `kSigma=5`）。
3. `dropRateThreshold` 必须 **显著大于 9250 tick/s²**（`log4` 未触发帧实测最大减速度），否则量化抖动会误触发。
4. **提高 `boostMax`**（当前 0.4 在 100% 的真实发射中顶死）或提高 `kBoost`，然后复测恢复时间。
5. **验收**：恢复时间中位 < 0.3 s（当前 0.428 s）。

**P2 — 给控制律补 `kS` 项**
`ff05` 的线性拟合截距在三份日志里稳定非零（0.126 / 0.164 / 0.230），说明存在静摩擦，而当前律 `duty = Kff·ω* + Kp·err + Ki·∫err + Vboost` 没有常数偏置项，**低转速段误差必然偏大**。Guide 第 202 行与 README 判读要点第 3 条都已自行指出这个缺口。建议先加静态 `kS` 再重测低转速稳态误差。

**P3 — 把工作区产物提交进版本库**
1. 提交 `OpModes/Flywheel/`（三级台架）。
2. 提交 `tools/tuning/` 的 7 个未跟踪脚本，特别是 `转速图分析指南.md`——它是整套判据体系的方法论载体。
3. 提交 `tools/tuning/out/` 的 4 组日志与报告（或至少提交 `_report.txt` 与 `_clean.csv`），作为标定基线的可追溯证据。
4. 如果 `doc/` 下的本报告分段文件也要保留，一并提交。

**P4 — 明确射程—转速映射的单一来源**
当前存在三条并行路线：DECODE 的线性回归（`2.010562318·distance + 540.4157073`）、OffSeason 的 `SPEED_A/SPEED_B`（默认 10/800，而 Dashboard 拟合值是 12.7/627，**从未回写**）、BIOBUZZ 的 `utility/BST`（静止标定滞空时间 → 目标转速）。同时 `Turret.java` 的 `RelPosBig` / `RelPosSmall` 已定义但**未被任何代码使用**，意味着大小球两个发射口的几何差异（炮口位置不同 → 同一目标的 yaw 角不同）**目前没有纳入弹道计算**。建议按 `AutoAimPrompt.md` 的需求实现时，明确以哪一套为唯一来源，并把 `RelPos*` 接进去。

**P5 — 代码可读性遗留问题（低优先，但会误导后人）**
- DECODE `controllers\shooter\Shooter.java` 里，大球增益字段的注释写的是「小球参数」、小球增益字段的注释写的是「大球参数」——**注释与代码相反**，且实际有两组增益按 750 °/s 硬切换，与注释暗示的语义都不一致。
- `Controllers/Shooter/Shooter.md` 全文只有一句话：「已废弃，仅用于prototype测试」。
- `Turret.java` 的 `RelPosBig` / `RelPosSmall` 注释说是「发射器中心的地面投影」，但无人使用。

---

> **关于报告本身的一句话**：本报告的全部技术结论都来自仓库内可复现的 `git log` / `git show` / `git diff` 与工作区文件全文，以及 `tools/tuning/out/` 下已生成的实测报告。凡属推断的内容，均在正文标注 `【推断】`。`doc/` 目录下另存有四份分段源文件（`_section_r1_baseline_and_new_arch.md`、`_section_offseason_july.md`、`_section_biobuzz_tuning.md`、`_section_flywheel_bench.md`），内容是本报告的严格子集。
## 九、证据索引、复现命令与产物清单

> 本章让任何读者都能独立复核前八章的每一条结论。所有路径均为本机实测路径；命令可直接粘贴执行。

### 9.1 三个仓库的定位（2026-10-16 检查时点实测）

| 仓库 | 本地路径 | 远端 | 检查分支 | HEAD |
|---|---|---|---|---|
| **Team19725_DECODE** | `G:\19725\code_part\project\Team19725_DECODE` | `git@github.com:BluePowerRobotics/Team19725_DECODE.git` | `master` | `328a6c3` · 2026-02-04 · zihengtangtzh25 · `Merge branch 'master' of ...` |
| **BIOBUZZ** | `G:\19725\code_part\project\BIOBUZZ` | `git@github.com:BluePowerRobotics/BIOBUZZ.git` | `flywheelControlSystemChange` | `202cd58` · 2026-10-15 · zihengtangtzh25 · `Merge branch 'flywheelControlSystemChange' of github.com:BluePowerRobotics/BIOBUZZ into flywheelControlSystemChange` |
| **19725_DECODE_OffSeasonGame** | `G:\19725\code_part\project\19725_DECODE_OffSeasonGame` | `git@github.com:BluePowerRobotics/19725_DECODE_OffSeasonGame.git` | `master` | `f8bda9c` · 2026-08-18 · Suanni-135 · `UPDATE` |

> **第三个仓库为何必须纳入**：用户需求只提了 DECODE 与 BIOBUZZ，但 2026 年 7 月这个时间窗内，飞轮电机控制的**实际改动全部发生在 OffSeasonGame**（DECODE 在 2026-02-04 之后就没有再动过发射机构）。BIOBUZZ 的 PIDSVA 库正是从 OffSeasonGame 的 `MotorExamples\PIDSVAControllers\` 继承来的。若只对比两个仓库，会得出「7 月没有变化」的错误结论。

### 9.2 关键提交索引（BIOBUZZ，按时间序）

| 提交 | 日期 | 作者 | 标题 | 与飞轮/炮台电机控制的关系 |
|---|---|---|---|---|
| `e106987` | 2026-06-03 | Lee | `Start` | 初始导入。新增 `utility/PID/{PIDController,PIDSVAController,SVAController,SlotConfig}.java`。**尚无任何飞轮/炮台代码** |
| `c9ba4b3` | 2026-06-14 | Zhz2025 | `一些PID优化和说明` | 第一次真正的控制律修改：PIDSVA 的积分从**无条件累加**改为 **I-zone 门控**（`|err| < iZone` 才积分，否则清零）；`SlotConfig.maxI` 默认值 `1` → `Double.POSITIVE_INFINITY`，新增 `iZone` 字段 |
| `ed7ade7` | 2026-08-19 | Suanni-135 | — | 新增 `utility/PID/MotorPIDCore.java`（**底盘**通用核，与飞轮无关） |
| `2936c21` | 2026-09-22 | Lee | `上赛季数据接入+shooter原型机` | 新增 `PrototypeShooter.java`（+179，PIDSVA 功率版）+ `PrototypeShooterTest.java`（+93）；导入 DECODE 的参考实现 |
| `9f05ad2` | 2026-09-28 | Suanni-135 | — | 新增 `OpModes/ShooterTuningTester.java`（+464，4 模式半自动台架）；PIDSVA 加 7 项 `lastXTerm` 诊断字段 |
| `ef4b120` | 2026-09-30 | Lee | `Turret手动` | **架构断点**：新增 `Controllers/Turret/{Shooter.java 275, Turret.java 129, YawController.java 187}` |
| `550ba4d` | 2026-10-01 | Lee | `Turret测试` | 新增 4 个炮台测试 OpMode（`FlywheelTest` / `TurretManualTest` / `TurretTickTest` / `TurretYawTest`） |
| `5996db3` | 2026-10-01 | Lee | `线性二元统计，PID小修，状态记录` | **PIDSVA API 重构**：5 参数外部 `velocity/acceleration` 版本 → 4 参数 `calculate(setpoint, measurement, dt, boolean velocityLoop)`，内部由 setpoint 差分推导参考速度/加速度；新增 slot 未配置抛异常。**这就是 `Controllers/Turret/Shooter.java:205` 调用的那个签名** |
| `d354201` | 2026-10-09 | Suanni-135 | `自动修正` | 仅 `ShooterTuningTester.java` 1 行 + `HypParams.java` 删空白。**无控制变化** |
| **`1937409`** | **2026-10-11** | zihengtangtzh25 | **`飞轮控制系统修改`** | **控制律断点，+1915/−0 纯新增**：`PrototypeShooter.java` 重写为自包含占空比律（+282）；新增 `OpModes/Tuning/` 全部 6 个 Java（`DropDetectionTuner` 190 / `FeedforwardTuner` 195 / `RelayFeedbackTuner` 218 / `StepResponseTuner` 203 / `ZieglerNicholsTuner` 221 / `TuningLogger` 94）+ `Guide.md`（219）；新增 `.trae/documents/飞轮控制系统重构方案.md`（+200） |
| **`b182459`** | **2026-10-15** | zihengtangtzh25 | **`小修`** | **+1760/−71**：`PrototypeShooter.java` +22/−1 = **掉速检测 arming 规则**；6 个 Tuner + Guide 修订；新增被跟踪的 `tools/tuning/{README.md, analyze_tuning.py 687, analyze_tuning.m 679, make_samples.py 119}` |
| `202cd58` | 2026-10-15 | zihengtangtzh25 | merge | = 当前 HEAD |

**同期的其他分支**：

| 引用 | SHA | 日期 | 说明 |
|---|---|---|---|
| `master` | `272398a` | 2026-10-15 | Lee `tests`。**有** `PrototypeShooter` / `Controllers/Turret/*` / `OpModes` 测试台，**无** `OpModes/Tuning/`、**无** `tools/tuning/*`、**无** `OpModes/Flywheel/`。其 `PrototypeShooter` 是 **1937409 之前的 PIDSVA 版本**（无掉速检测） |
| `origin/26.10.11` | `f4333ac` | 2026-10-11 | = master + 全部 7 个 `OpModes/Tuning/*` + `tools/tuning/{README,analyze_tuning.py,analyze_tuning.m,make_samples.py}`。其 `PrototypeShooter` = **1937409 重写版但无 arming**。仍无 `OpModes/Flywheel/` |
| `flywheelControlSystemChange` | `202cd58` | 2026-10-15 | = `origin/26.10.11` + `b182459`（arming + 工具链） |

> 因此，**同一条控制律在三个引用上有三个版本**：master = 无掉速检测；26.10.11 = 有掉速检测无 arming；flywheelControlSystemChange = 有掉速检测 + arming。

### 9.3 复现命令清单

```powershell
# ── 仓库定位 ──
git -C G:\19725\code_part\project\BIOBUZZ rev-parse --abbrev-ref HEAD
git -C G:\19725\code_part\project\BIOBUZZ log -1 --date=short --format='%h|%ad|%an|%s'
git -C G:\19725\code_part\project\BIOBUZZ remote get-url origin

# ── 7 月窗口（OffSeasonGame）──
git -C G:\19725\code_part\project\19725_DECODE_OffSeasonGame log --all --since=2026-06-20 --until=2026-09-01 --date=short --format='%h|%ad|%an|%s'
# 单文件历史（跟重命名）
git -C G:\19725\code_part\project\19725_DECODE_OffSeasonGame log --follow --all --date=short --format='%h|%ad|%an|%s' -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java

# ── 控制律断点的实际差异（最重要的一条）──
git -C G:\19725\code_part\project\BIOBUZZ show 1937409 --stat
git -C G:\19725\code_part\project\BIOBUZZ show 1937409 -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Shooter/PrototypeShooter/PrototypeShooter.java
# arming 规则的精确定位（+22/−1）
git -C G:\19725\code_part\project\BIOBUZZ diff 1937409 b182459 -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Shooter/PrototypeShooter/PrototypeShooter.java

# ── 三分支控制律对比 ──
git -C G:\19725\code_part\project\BIOBUZZ diff --stat master origin/26.10.11
git -C G:\19725\code_part\project\BIOBUZZ diff --stat origin/26.10.11 flywheelControlSystemChange
git -C G:\19725\code_part\project\BIOBUZZ ls-tree -r --name-only master | Select-String -Pattern 'Tuning|Flywheel|tools'
git -C G:\19725\code_part\project\BIOBUZZ ls-tree -r --name-only origin/26.10.11 | Select-String -Pattern 'Tuning|Flywheel|tools'

# ── 未提交产物清单 ──
git -C G:\19725\code_part\project\BIOBUZZ status --short --untracked-files=all
git -C G:\19725\code_part\project\BIOBUZZ diff --stat HEAD

# ── PIDSVA API 重构的精确差异 ──
git -C G:\19725\code_part\project\BIOBUZZ show 5996db3 -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/PID/PIDSVAController.java

# ── I-zone 门控的引入 ──
git -C G:\19725\code_part\project\BIOBUZZ show c9ba4b3 -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/PID/PIDSVAController.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/PID/SlotConfig.java

# ── OffSeasonGame 的 stash 核对（PowerShell 需要给花括号加引号）──
git -C G:\19725\code_part\project\19725_DECODE_OffSeasonGame stash list
git -C G:\19725\code_part\project\19725_DECODE_OffSeasonGame 'stash@{0}' show --stat

# ── 离线分析工具（需要 Python 3 + numpy + matplotlib，无需 scipy）──
python tools/tuning/make_samples.py ./samples            # 生成真值已知的合成数据
python tools/tuning/analyze_tuning.py ./samples          # 复算，应还原 README §6 表中的真值
python tools/tuning/analyze_tuning.py ./tuning_data --no-plot
```

> **MATLAB 版的环境约束**：`analyze_tuning.m` 需要写 `%APPDATA%\MathWorks` 与临时目录，因此运行它的沙箱必须是 `danger-full-access`，否则 MATLAB 直接启动失败（退出码对应 `0xC0000142`）。此约束已记录在 `tools/tuning/README.md:56-58`。

### 9.4 未提交产物清单（工作区独有，不在任何 git 提交中）

**(a) `OpModes/Flywheel/` — 三级飞轮台架（共 1343 行 Java + 262 行 Guide）**

| 文件 | 行数 | 作用 |
|---|---|---|
| `Flywheel1OpenLoopLimit.java` | 316 | 阶段①：无控制器，开环爬升占空比，测极限转速与对应母线电压 |
| `Flywheel2PidVsOpen.java` | 359 | 阶段②：单 PID 整定，并与「不用 PID（恒占空比）」对照 |
| `Flywheel3ControllerVsPid.java` | 448 | 阶段③：现有完整控制律 vs 纯 PID 对照 |
| `FlywheelBench.java` | 109 | 跨 OpMode 的静态台架结果表 |
| `FlywheelLog.java` | 111 | CSV 记录器，写 `<RC 外部存储>/flywheel/fly1_openloop_<时间戳>.csv` |
| `Guide.md` | 262 | 数据链说明（手柄 → ω* → duty → H 桥 → V = V_bus·duty → I → T → Jeq·ω̇ = n·Kt·I − Beq·ω − T_L → 编码器）、CSV 取回命令、故障排查 |

**(b) `tools/tuning/` 的 7 个未跟踪文件（共 1974 行）**

| 文件 | 行数 | 作用 |
|---|---|---|
| `analyze_dashboard_log.py` | 533 | 分析 FTC Dashboard 导出的 `.jsonl` 运行日志；输出 overview / zoom / scatter 三图与增益反演 |
| `analyze_launch_events.py` | 272 | 自动识别发射掉速事件，统计跌幅、恢复时间、恢复期占空比与电流峰值、boost 参与度 |
| `plot_flywheel.py` | 364 | 画 Flywheel 台架 CSV（`fly1/fly2/fly3_*`），自动识别三种模式 |
| `plot_speed.m` | 415 | 纯 MATLAB 画转速图 + 4 面板诊断图 |
| `check_plot_data.py` | 138 | 绘图数据一致性自检（列/行/单调性/NaN/时间轴/数值合理性/PNG 可解码/交叉复算） |
| `export_clean_table.py` | 119 | 丢弃**损坏的毫秒字段**、按行序重建均匀时间轴，并重算 `error = target − velocity` |
| `转速图分析指南.md` | 133 | 读图方法学 + 三档验收阈值表 + 症状→原因→修法对照（**整套判据体系的方法论载体**） |

**(c) `tools/tuning/out/` — 4 组实测数据与 25 个分析产物**

- 清洗后 CSV：`out/clean/{log2_clean.csv, log3_clean.csv, log4_clean.csv, shooter_log_clean.csv}`
- 结论文本：`out/{log2_shooter_report.txt 5477B, log3_shooter_report.txt 4949B, log4_report.txt 5739B, log4_launch_report.txt 2975B, shooter_log_report.txt 3318B, plot_speed_stats.txt 2820B}`
- 图片：`out/` 下 13 张 PNG（各日志的 overview / zoom / scatter，log4 另有 clean/launch 系列）

**(d) 其他未跟踪项**

- `_ctx_b8_baseline.txt`（根目录，审计过程产生的中间文件，非源码）
- `doc/_section_*.md`（本报告的 4 份分段源文件）

### 9.5 正文数字的出处对照

| 正文位置 | 结论 | 数据来源文件 |
|---|---|---|
| 6.4.1、8.3 结论 4 | `kp` 反演 0.019045，一个台阶跳 0.3809 占空比，判「严重」 | `tools/tuning/out/shooter_log_report.txt` |
| 6.4.2、8.3 结论 5 | log2 vs log3：同 kp、boost 触发 21.1% vs 0.0%、duty sd 0.079 vs 0.018 | `tools/tuning/out/log2_shooter_report.txt` + `log3_shooter_report.txt` |
| 6.4.3、8.3 结论 6 | 13 次发射、跌幅中位 270、恢复中位 0.428 s、100% 饱和、boost 峰值 0.400 | `tools/tuning/out/log4_launch_report.txt` |
| 6.4.3 | 1750 段稳态 sd 11.0 / 误差 −0.7 / duty sd 0.020；2250 段饱和 74.9% | `tools/tuning/out/log4_report.txt` |
| 6.4.4 | 独立复算 14 事件（vs Python 13）、恢复中位 0.418 s | `tools/tuning/out/plot_speed_stats.txt` |
| 6.5.3 验收表 | 三档阈值（稳态误差 / v_sd / duty sd / 饱和 / 过冲 / 恢复时间 / kp×台阶） | `tools/tuning/转速图分析指南.md`（未提交） |
| 2.x 提交时间线 | 每个提交的日期、作者、标题、规模 | `git log` / `git show --stat`（可用 §9.3 命令复现） |
| 五章 7 月编年 | 41 条提交、`Turret.java` 被改 14 次、`3bac6fb` / `431f220` 两次结构改动 | OffSeasonGame 仓库 `git log` |
| 五章 PIDSVA 前史 | `PIDSVAControllers`（复数包）2026-06-10/11 诞生、7 月零改动 | OffSeasonGame 仓库 `git log` |
| 七章 台架细节 | 三个 Flywheel OpMode 的行数、参数默认值、互不依赖 | 工作区文件全文（未提交） |

### 9.6 证据等级与尚未完全核实的事项

**等级约定**：`【源码】`= 读过文件全文并逐行核对；`【Git】`= git 命令实测输出；`【报告】`= 仓库内已有文档的原文；`【推断】`= 由以上证据推导，未直接验证。

**明确未完全核实的三处**（如实列出，不编造）：

1. **`TurretDegreeController.java` 在 `stash@{0}` 中的 5 行改动未展开**。该 stash（`stash@{2026-09-24}` = `5ac7b1d`，2026-09-24，zihengtangtzh25）共触及 6 个文件，其中 `Turret/turner/TurretDegreeController.java` 仅 5 行，其余为 `AutoAction*` / `EatTest`（455+/454−，近 1:1，属格式改写）。这 5 行的具体内容未读取，因此**无法断言该 stash 是否包含任何控制逻辑变化**。
2. **`TurretDegreeController.stop()` 是否存在未验证**。
3. **以下文件未读全文**：`zhz.md`、`TurretPrompt.md`（部分）、`MotorExamples/README.md`、`MotorPIDTuning.java`。凡涉及这些文件的结论均标注了来源等级。

**其他需要提示的事实**：

- 《飞轮系统控制理论研究报告》**不在仓库内**。它被 `OpModes/Tuning/Guide.md:3`、`tools/tuning/README.md:7`、`.trae/documents/飞轮控制系统重构方案.md` 三处引用为「第 6 章」的来源，但仓库里没有这份文档，根目录只有 `README.md`（723 B）。本报告对「无模型标定」方法论的描述**全部以代码与 Guide.md 的实际内容为准**，不依赖这份外部文档。
- `doc/` 下较早的分段文件中曾出现「`272398a` 不可从当前检出到达」的说法——该说法**已过时**：`272398a` 确实是 `master` 的当前 tip（2026-10-15，Lee `tests`），只是不在 `flywheelControlSystemChange` 的祖先链上。以 §9.2 的表为准。
- `doc/_section_biobuzz_tuning.md` 曾被 `_section_r1_baseline_and_new_arch.md` 的头部列为四份分段文件之一，但在本次会话早期**实际不存在**（负责该段的子代理耗尽预算未落盘）。第六章由主代理基于 `OpModes/Tuning/Guide.md`、`tools/tuning/README.md`、`转速图分析指南.md` 原文与 `out/` 下已生成的四份实测报告重新撰写。

---

# 十、2026-10-22~27 增补：第 4 代定参、双飞轮与验证工具链

> 第一至九章的证据停在 2026-10-16（`202cd58`）。10-22 起飞轮这条线又走了三步：**第 4 代控制律重写（删 boost）→ 双飞轮参数分家并定稿 → 四件验证工具落地、六条判据收尾**。
> 本章按「控制律断点 / 量化验证 / 参数定稿 / 判据现状 / 工具链 / 教训与遗留」六节记录，数字全部来自本轮四次实机日志的离线复算。

## 10.1 本轮提交时间线（`git log`，2026-10-22 ~ 10-27）

| 提交 | 日期 | 标题 | 规模 |
|---|---|---|---|
| `0969cbd` | 2026-10-22 | 飞轮控制系统：第 4 代（tauV 调度 + 掉速冻结 + 电压归一化，删除 boost） | 控制律与参数层重写 |
| `229728e` | 2026-10-27 | 飞轮参数分家：大小飞轮各自一套参数，控制参数唯一出处 `FlywheelParams` | 参数容器 + 注入式构造 |
| `82c88ce` | 2026-10-27 | Merge `master`（`FTC_Format.md` 等 206 行文档） | 合并 |
| `e8b86c3` | 2026-10-27 | 飞轮验证工具链：`FlywheelRun` 键位重排 + 电压扫描 + 多代对比 | 21 files, +4524 / −17 |
| `ad35364` | 2026-10-27 | 飞轮定参：大小轮各自一套实测参数 + 数据统一走 Dashboard 遥测 | 15 files, +508 / −430 |
| `85cdca4` | 2026-10-27 | 判据⑥收尾：`VoltageSweep` 四档实测通过（最大偏差 2.2%） | 文档 2 个 |

> 与 §2.4「尚未提交的工作（2026-10-16）」对照：那批"工作区独有"的台架 OpMode（`Flywheel1OpenLoopLimit` / `Flywheel2PidVsOpen` / `Flywheel3ControllerVsPid`）**仍未入库**，第 4 代改为走 `FlywheelRun` + `VoltageSweep` + `pastcalculater` 三条线；文档层则补齐了 `doc/现在的控制系统.md`（第一至九章时它还是第 3 代内容）。

## 10.2 控制律断点：第 3 代（boost）→ 第 4 代（滤波调度 + 掉速冻结 + 电压归一化）

第 3 代（`1937409`，2026-10-11 起）的律是
`duty = kS·sgn(ω*) + kff·ω* + kp·err + ki·∫err(+iZone 门控) + Kboost·err`（掉速时 boost 顶到 `boostMax`）。
§6.4 的实测已经指出它的问题：**13/13 次发射 boost 全部顶死 `boostMax=0.4`**（等于把前馈+P 推到饱和），且 `iZone=500` 让积分在大偏差时直接清零，恢复期完全没有积分作用。

第 4 代（`0969cbd`）把这三件事一次性换掉：

| 维度 | 第 3 代 | 第 4 代 | 换掉的理由 |
|---|---|---|---|
| 测量平滑 | 无（直接用 `ω`） | `ωf = EMA(ω, τ(\|e\|))`，τ 在 `tauVMax=0.055`↔`tauVMin=0.012` 间按误差调度 | 量化台阶（20 tick/s）让 `ω` 本身是方波，直接进 P 项等于把台阶噪声灌进占空比 |
| 掉速处理 | 检测到就加 boost，`boostMax` 限制幅度 | **冻结积分 + 冻结母线滤波 + hold-off**，靠 P 项与饱和自然恢复 | boost 与 P 同向叠加，实测 100% 饱和，等于没有额外作用；冻结积分才是真正需要的那件事 |
| 积分门控 | `\|err\| < iZone(500)` 否则清零 | `\|err\| < eBand(= max(0.02ω*, 2·台阶))` 内积分；带外冻结、超 `iFreezeTimeoutSec=1.5 s` 清零 | eBand 由量化台阶直接算出，不再是手调常数 |
| 电压 | 无 | `s = clamp(V_nom/V_busFilt, 0.85, 1.25)` 只乘前馈，掉速期间冻结 | 电池从 12.9 V 掉到 10.5 V 时前馈要自动补上 |
| 微分 | 无（更早的 PIDSVA 有 kD） | 无 | 量化台阶下 kD 只会放大噪声 |
| 参数出处 | `PrototypeShooter` 内部常量 | `utility/Shooter/FlywheelParams.java`（`@Config` 单一出处，`copyOf`/`applyTo`/`copyFrom`） | 整定值散落、Dashboard 改不到 |

验证方法也换了代：第 3 代用「遥测 5 行 + 离线 boost 判据」，第 4 代直接用 `FlywheelRun` 输出 15 个键（含 `ffTerm/pTerm/iTerm/integral/inDrop/iActive/settled/vscale`），**每个分量都能单独画图**，这才使得下面 10.3 的三条量化验证成为可能。

## 10.3 三个机制的量化验证（本轮新增的实测证据）

**(a) 量化台阶决定 kp 上限**（判据①的物理来源）。两档稳态相减得对象增益 `Kd ≈ 2740 tick/s` 每单位占空比；`kp·Kd` 就是「读数跳一格 → 稳态转速偏多少格」的单步增益：

| kp | `kp×台阶(20)` | `kp·Kd` | 极限环幅度 | 备注 |
|---|---|---|---|---|
| 2.20378e-3（09-04 整定值） | 0.044 | 6.0 | ±2 格 | 判据①仍在限内，但余量只剩 12% |
| 1.81917e-3（13:07 日志反推） | 0.036 | 5.0 | ±1~2 格 | 稳态 `v_sd` 13~19 |
| **1.0e-3**（10-27 中途值） | 0.020 | 2.7 | ±1 格 | 稳态 `duty_sd` 降到 0.006~0.008，**但回带从 0.31 s 涨到 0.91 s** |
| **1.9095e-3 / 1.739e-3**（最终，大/小） | 0.038 / 0.035 | 5.2 / 4.7 | ±1~2 格 | 回带 0.31 / 0.19 s，`duty_sd` 0.013~0.021，**两头都过** |

结论：`kp` 不能只看判据①的 0.05 上限——它是**回带时间**（`τ = 1/(kp·Kd)`）与**稳态抖动**的同一个旋钮。1.0e-3 是"稳态最漂亮"的错解，1.9e-3 才是两项都达标的交点。

**(b) 滤波调度拐点必须 ≥1 个台阶。** 第 4 代初版取 `eLoFrac=0.30 / eHiFrac=0.90`，在 1900 tick/s 处拐点是 **12 / 36 tick/s**，都低于一个台阶（20）：
实测 `|e|` 有 **14% 的时间越过 36 而进入 `tauVMin=8 ms` 的"快档"**（比 `tauVMax` 还快），平均 τ 只有 24.3 ms。
把拐点抬到 `0.50 / 1.50`（=20 / 60 tick/s）后，同一段读数离线重放：**P 项 sd 0.0265 → 0.0144（−46%）**，调度占空 50/36/14 → 86/14/0，平均 τ 52 ms。

**(c) 回程是纯 P 一阶过程，积分在掉速期间 0 参与。** 19:07 的日志里 780 个掉速样本 **`iActive` 恒为 0**（掉速态 + `|e|>eBand` 双重门控），回带曲线严格服从 `t = ln(跌幅/eBand)/(kp·Kd)`；把 12 个事件画在同一张图上，kp 三条解析曲线与实测点吻合（`recovery_model.png`）。这解释了「为什么加了滤波仍然抖/回得慢」：抖是 `kp·Kd` 决定的，慢是 `1/(kp·Kd)` 决定的，**滤波只改圆滑程度**。

## 10.4 双飞轮：参数分家（`229728e`）与定稿

**(a) 分家。** `FlywheelParams.BIG` / `FlywheelParams.SMALL` 两个 static 实例，字段完全相同、数值各自独立；`PrototypeShooter(hw, telemetry, motorName, params)` 注入式构造（`PrototypeShooter.java:98` 是**引用**，改容器即时生效）；`FlywheelRun` 增加 Dashboard 开关 `paramsSet`（`BIG`/`SMALL`）与 `motorName`，选 `SMALL` 时屏上打印该套参数摘要，若仍是 BIG 的占位拷贝会额外提示。

**(b) 小球第一次实测（19:59）暴露的问题。** 当时小球块还是 BIG 的占位拷贝（`kff=3.8307e-4` 等），但日志反推的前馈只有 `ff/ω* = 3.135e-4`，于是**积分长期顶着 +0.09~0.11 占空比**（∫ = 37.3 / 44.2，判据③限值 15.4，超 2.4~2.9 倍）。由两个稳态点反推真实对象模型得 `duty = 0.02677 + 3.7268e-4·ω`，据此把 `kff` 调到 3.83e-4 量级、`ki` 同步下调，积分立刻回到 ±1.6。

**(c) 定稿（`ad35364`，两轮实测）**：

| 参数 | BIG | SMALL | 与第 3 代/初版的差别 |
|---|---|---|---|
| `kS` | 0.02945 | 0.02945 | 未动（09-04 回归值） |
| `kff` | 3.8307e-4 | **3.8346e-4** | 小球从"抄 BIG"改为实测值 |
| `kp` | **1.9095e-3** | **1.739e-3** | 小球比大球低 9%（纹波更大） |
| `ki` | 2.59224e-3 | **2.50771e-3** | 小球同步微调 |
| `useVoltageNorm` | **true** | **true** | 第 4 代初版为 false |

两台飞轮的静态模型几乎相同（`0.0294+3.83e-4ω` vs `0.0295+3.83e-4ω`）——**同一批电机/同一传动**，所以第 3 代"一套参数打天下"并非完全错误，只是纹波与积分行为不同。

## 10.5 六条判据的现状（2026-10-27）

| # | 判据 | 结论 | 关键数字 |
|---|---|---|---|
| ① | `kp×台阶 ≤ 0.05` | **过** | 0.0382（大）/ 0.0348（小） |
| ② | 稳态 `v_sd ≤ 12`、`\|误差\| ≤ 0.12%`、`duty_sd ≤ 0.025`、饱和 0% | **三项过、`v_sd` 不过** | `duty_sd` 0.0130~0.0206 ✓；误差 ≤0.11% ✓；`v_sd` 9.0~15.1 / 10.2~17.9 ✗（量化下限） |
| ③ | 积分长期值 < `0.04/ki` ≈ 15.4 | **过** | ∫ = −2.1 / −3.7（大）、+1.3 / −1.6（小） |
| ④ | 回带中位 ≤0.43 s、过冲 ≤ +20 | **过** | 0.31 s（最大 0.45）/ 0.19 s（最大 0.32） |
| ⑤ | `settled` 稳态 0.1 s 内 true、恢复期 false、不抖 | **过** | 稳态 96~100%，恢复期正确翻 0 |
| ⑥ | 满电 vs 10.5 V，`duty·V_bus` 差 ≤3% | **过** | 四档最大偏差 2.2%（1600）/ 1.1%（1800） |

判据②的 `v_sd` 是**唯一不达标项**，且已论证为结构下限：±1 格方波 sd = 10、±2 格 = 20，判据却要求 12。要根治必须让控制器"知道自己在量台阶"（反馈死区或把 `ωf` 量化到台阶中点），属于改控制律、需重做全表验收——已列入待办 P1。

判据⑥的实现方式见 10.6 的 `VoltageSweep`：四档 `10.5 / 11.0 / 11.5 / 12.0 V`，负载占空比 0.91 / 0.78 / 0.48 / 0.01，`duty` 从 0.642 抬到 0.740（+15%），与 `V_nom/V_bus = +13.7%` 吻合；母线闭环 `max|实测−目标| = 0.141 V`（规格 ±0.15）。

## 10.6 验证工具链（`e8b86c3` + `ad35364`）

第 3 代的验证手段是「遥测 5 行 + 离线 `analyze_*.py`」，本轮补成四件套，且**数据一律走 `telemetry`**（各 OpMode 已绑 `MultipleTelemetry`，由 FTC Dashboard 自带 Logging 记录并导出 CSV，不再写自定义文件）：

| 工具 | 位置 | 作用 |
|---|---|---|
| `FlywheelRun` | `OpModes/FlywheelRun.java` | 手动验证：START 静止、`A→1600`、`B→1900`、十字键 ±100、`X→停`；15 个键（`target/velocity/duty/voltage/error/current/integral/pTerm/iTerm/ffTerm/vscale/indrop/iactive/settled`）；`paramsSet`/`motorName` 切飞轮 |
| `VoltageSweep` | `OpModes/VoltageSweep/` | 负载电机做母线电压闭环（PI + 速率限幅，±0.15 V），每档 7 段（settle/a/b/pre/dip/rec/rel），量化判据⑥；`_dip` 段用开环压占空比模拟掉速 |
| `PastCompare` + `TuneGen1/3/4` | `utility/pastcalculater/` | 三代精简计算器（Gen1/2 PIDSVA、Gen3 boost、Gen4 现行）+ 各自 static 参数容器；在线**一次只驱动一代**、三代同输入并行算、28 键同时记录；也可离线回放历史 CSV 只比 duty；分代 Tuner 各标各的 |
| `plot_csv.py` | `tools/tuning/plot_csv.py` | 任意 Dashboard CSV → **一列一张 PNG** + `_summary.txt`；`--group`/`--segment`/`--overlay` 支持分代、分段与分量叠加 |

> `utility/pastcalculater/` 的 OpMode 与计算器同目录（**有意不遵守 `FTC_Format.md` 的分层要求**，用户明确要求）；`docs` 见各自 `GUIDE.md`（`VoltageSweep/GUIDE.md`、`pastcalculater/GUIDE.md`）+ `tools/tuning/PLOT_GUIDE.md`。

## 10.7 本轮教训与遗留

**三条教训（都已写进文档，避免下次重犯）**

1. **`kp` 是双向旋钮**：判据①只管上限，回带时间由 `1/(kp·Kd)` 定；把 `kp` 减半能让 `duty_sd` 减半，代价是回带翻倍（0.31 → 0.91 s）。整定必须同时看「稳态 `duty_sd`」与「回带中位」两条曲线。
2. **滤波拐点低于量化台阶 = 白滤**：`eLoFrac=0.30` 在 1900 处只有 12 tick/s，14% 的时间反而切到更快的 `tauVMin`。拐点必须 ≥1 个台阶。
3. **参数只活在 Dashboard 里等于没定**：`FlywheelParams` 的 `@Config` 挂在实例字段上，Dashboard 改不动实例字段；而 `GenXParams` 与 Dashboard 面板能改的是 static 字段。整定完必须回写源码并编译，否则重启即丢（本轮就发生过一次「日志与截图参数不一致」）。

**遗留（按优先级，详见 `doc/飞轮待办改进清单.md` §二）**

- P1：量化台阶感知（根治 `v_sd`）、发射门控判据、电流保护/告警、掉速期间母线补偿的取舍；
- P2：高转速区与 2400 天花板直测、`kS/kff` 多点复核、`tauBus` 实测复核（归一化开了以后它变关键）、带球 10~15 发、电压扫描低压端（单负载只能到 ~10.5 V）；
- P3/P4：供电能力、工作转速、传动惯量；飞轮接入 `TeleOpBlue/Red` 与自动发射流程；大小飞轮的带球对比。

---

**报告完。**
