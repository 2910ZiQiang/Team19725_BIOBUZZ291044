# 炮台（Turret）瞄准与发射指南

本文说明 `Controllers/Turret` 下炮台的**自动瞄准**、**手动瞄准**、**发射门控**的计算逻辑，以及**自动标定流程**与**比赛操作方法**。所有结论均来自当前代码实现，若代码改动请同步更新本文。

---

## 1. 结构与职责划分

| 部件 | 文件 | 职责 |
| --- | --- | --- |
| `Turret` | [Turret.java](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java) | 汇总上层目标，分发给平转与两个发射器，决定扳机释放 |
| `YawController` | [YawController.java](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/YawController.java) | 平转电机位置闭环（PIDSVA）+ 到位判据 |
| `Shooter` | [Shooter.java](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Shooter.java) | 单发射器：飞轮速度环、俯仰舵机、扳机舵机；以及自动瞄准反解 |
| `FlywheelParams` / `FlywheelCalculator` | `utility/Shooter/` | 飞轮速度环参数与计算、稳态判据 |
| `RobotPosition` | `Processors/RobotPosition/` | 提供车辆位姿 `Pose2d` 与当前球门 `getGoalPose()` |

机械结构：**一个大球发射器 + 一个小球发射器**，两者朝向始终平行，共用同一个平转电机（共享 yaw）。每个发射器各有一个飞轮、一个俯仰舵机、一个扳机舵机。

器件名（HardwareMap 配置必须一致）：

- 平转：`yawMotor`
- 大球：`shooterBig` / `pitchBig` / `triggerBig`
- 小球：`shooterSmall` / `pitchSmall` / `triggerSmall`

三个 `Turret.update` / `Turret.updateManual` 的重载语义：

| 方法 | 用途 |
| --- | --- |
| `update(yaw, pitchBig, pitchSmall, speedBig, speedSmall, trigBig, trigSmall)` | 完全开放环目标（最底层，测试用） |
| `update(allowFire, preload, forceReleaseBig, forceReleaseSmall)` | **自动瞄准**，位姿/球门由 `RobotPosition` 提供 |
| `updateManual(yawRate, pitchBig, pitchSmall, speedBig, speedSmall, trigBig, trigSmall)` | **手动瞄准**，摇杆给角速度、俯仰与转速由操作手给 |

---

## 2. 坐标系与符号约定

全部沿用 RoadRunner / `FTC_Format.md` 的约定：

- 场地坐标系：原点在场地某固定点，`x`、`y` 为英寸，`heading`（记作 θ）为**车头朝向**，逆时针为正。
- 车体相对坐标系：`x` 指向**车头**，`y` 指向**车体左侧**。
- 平转角 `yaw`：`yaw = 0` 表示两个发射器朝向车头方向，**逆时针为正**。
- 平转机械行程：`yaw ∈ [-yawRange, +yawRange]`，默认 `yawRange = π`，即**硬限位、只能转一圈**。任一物理指向在该区间内的表示是唯一的（折叠到 `(-π, π]`），所以代码中**不叠加 2π 分支**。
- 角度归一化：`Vector2D.normalizeAngle(a)` 把角度折算到 `(-π, π]`。

因为硬限位，`[-π, π]` 本身就是完整可达行程，超出部分会被饱和钳制（而不是折到另一侧）。

---

## 3. 自动瞄准解算

入口：[Turret.update(allowFire, preload, forceReleaseBig, forceReleaseSmall)](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L206-L305)，每帧调用一次。

> **球门来源**：`Turret` 自己不选门，每帧读 `RobotPosition.getGoalPose()`。该缓存由 `RobotPosition.update()` 按以下优先级刷新：
>
> 1. **本帧 MT1 非 MIDDLE 观测**（`AUDIENCE_UP` / `AUDIENCE_DOWN`）优先级最高，直接决定球门；
> 2. 否则保持上一帧选择（最近一次非 MIDDLE 读数，或 TeleOp 中操作手通过 GP2.LT/RT 的手动设置）；
> 3. 从未观测到抬升侧、操作手也未设置时默认 Up。
>
> - `AUDIENCE_UP` → 本颜色的 `*AudienceUp`；`AUDIENCE_DOWN` → `*AudienceDown`（抬升侧 = 打同侧）。
> - 操作手手动入口：`RobotPosition.setManualGoalDown(boolean)`（TeleOp 按键映射见 §10）。
>
> ⚠️ 自动阶段底盘用的 `AutoAimAction` 仍是**独立实现**：它也按「抬升侧 = 同侧、`MIDDLE` 保持上次锁定」选择球门，当前与 `RobotPosition` 结论一致；但它直接读融合位姿 / `getHiveState()`，尚未改为读 `RobotPosition.getGoalPose()`，也不响应操作手手动设置。详见第 13 节。
>
> ⚠️ TeleOp 中 GP1.LT/RT 的车头瞄准**不读** `RobotPosition.getGoalPose()`：它按扳机与颜色直接选择车头球门（见 §10），只影响车头朝向，不改写炮台/共享球门。因此按住 GP1 扳机时车头与炮口可能暂时指向不同球门，这是有意保留的操作便利。

### 3.1 发射器实际位置

发射器旋转中心在车体上的投影由 `RelPosBig` / `RelPosSmall` 给出（车体相对坐标，英寸），见 [Turret.java](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L43-L54)：

```java
public static Vector2D RelPosBig = new Vector2D(0, 0);   // todo: 实机标定
public static Vector2D RelPosSmall = new Vector2D(0, 0); // todo: 实机标定
```

旋转到场地系（逆时针旋转 θ）：

$$
p_s = p + R(\theta)\, r,\qquad
R(\theta)=\begin{bmatrix}\cos\theta & -\sin\theta\\ \sin\theta & \cos\theta\end{bmatrix}
$$

即 [AimGeometry.shooterGroundPos](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/AimGeometry.java#L25-L30) 中（`Shooter.solve` 与 `Turret.goalBodyYaw` 都调用它）：

```java
Vector2D rotated = Vector2D.rotate(relPos != null ? relPos : Vector2D.ZERO,
        pose.heading.toDouble());
return new Vector2D(pose.position.x + rotated.getX(),
        pose.position.y + rotated.getY());
```

`RelPos` 是**车体坐标系下的固定偏移**（发射器相对机器人几何中心的安装位置），不是随 yaw 变化的量——发射器绕平转轴旋转，但旋转中心本身在车体上不动。

### 3.2 所需平转角

方位角（场地系）：

$$
\beta = \operatorname{atan2}(y_g - y_s,\; x_g - x_s)
$$

所需平转角（车体相对，逆时针为正）：

$$
yaw = \operatorname{wrap}(\beta - \theta) \in (-\pi,\ \pi]
$$

### 3.3 射程

$$
d = \lVert p_g - p_s \rVert = \sqrt{(x_g-x_s)^2 + (y_g-y_s)^2}
$$

单位英寸，供档位选择与转速线性模型使用。

### 3.4 仰角档位选择（离散 + 滞回）

仰角不是连续量，只能取有限个**离散档位**。每档 `PitchModel` 包含：

| 字段 | 含义 |
| --- | --- |
| `pitchPos` | 该档的俯仰舵机位置 `[0,1]` |
| `minRange` / `maxRange` | 该档适用射程区间（英寸） |
| `speedSlope` / `speedIntercept` | 该档内「射程 → 转速」线性模型 |

档位表在 [Shooter.java 静态块](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Shooter.java#L190-L203) 中定义，**当前是占位值，必须实机标定**：

```java
BIG_PARAMS.pitchModels = new PitchModel[]{
        new PitchModel(0.30, 30.0, 70.0, 30.0, 900.0),
        new PitchModel(0.50, 60.0, 100.0, 34.0, 1000.0),
        new PitchModel(0.70, 90.0, 150.0, 38.0, 1100.0)
};
```

选择规则（[selectPitchModel](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Shooter.java#L388-L434)，三步）：

1. **优先**：所有 `covers(d)`（`minRange ≤ d ≤ maxRange`）的档位中，取 `speedFor(d)` **转速最小**的一档（射程重叠区避免用高速档空耗）。
2. **滞回**：若当前已选中的档位虽然不再是"最优"，但射程仍落在 `[minRange − pitchSwitchMargin, maxRange + pitchSwitchMargin]` 内，则**保持原档**（`pitchSwitchMargin = 5.0 in`）。目的是防止射程在档位边界抖动时，俯仰目标与转速目标逐帧跳变——那会不断重置 `pitchSettleTime` 计时与飞轮稳态计数，导致 `isReady()` 永远不成立。
3. **无覆盖**：超出所有档位区间时，取**射程区间距离最近**的一档（距离定义为到最近端点的间隔）。

**契约与陷阱**：

- 表**必须按 `pitchPos` 升序**：`getMinPitch()` / `getMaxPitch()` 直接取 `models[0]` / `models[n-1]` 作为预载的两个安全姿态，未排序时静默给出错误仰角（代码中无断言）。
- 允许射程区间**重叠**，重叠区按「所需转速最小」选档：占位表里第二档斜率更大，60~70 in 的重叠区会选中第一档（更低仰角），因此"距离越远仰角越高"的直觉在重叠区**不成立**。标定完成后需检查档位选择随距离是否连续。
- **滞回只锁俯仰，不锁转速模型**：第 2 步保持旧档时，射程可能已越出该档的标定区间（最多 `pitchSwitchMargin = 5 in`）。因此 `solve()` 里查转速模型前会把射程**钳回区间端点**（`clampToModelInterval`），即"区间外取边界值"，**不做线性外推**——外推会得到未经标定的转速，偏大时还会顶到 `speedMax`，使 `isSettled()` 永不成立、该档位附近发不出去。

档位表为空（`pitchModels.length == 0`）时退化为 `pitchMax` + 转速 0，保证机构安全。

### 3.5 转速与钳制

```math
n^\* = \operatorname{round}(\text{speedSlope}\cdot \hat d + \text{speedIntercept}),\qquad
\hat d = \operatorname{clamp}(d,\ \text{minRange},\ \text{maxRange}),\qquad
n = \operatorname{clamp}(n^*,\ 0,\ \text{speedMax})
```

`d̂` 是**钳到所选档位标定区间内**的射程（见 §3.4 最后一条）：模型只在标定区间内成立，区间外取边界值而不外推。

`speedMax` 来自该发射器的 `FlywheelParams`（BIG/SMALL 各自独立）。

### 3.6 共享平转角：两发射器的劣角平分线

两个发射器安装位置不同（`RelPosBig ≠ RelPosSmall`），各自解出的 `yaw` 也略有差异，但只有一个平转电机。因此取**劣角平分线**（[Turret.java](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L261-L276)）：

```math
yaw_{auto} = \operatorname{atan2}\!\big(\sin yaw_B + \sin yaw_S,\ \cos yaw_B + \cos yaw_S\big)
```

用"单位向量求和再取角"而不是 `(yaw_B + yaw_S)/2`，可以正确处理跨 `±π` 的绕回（例如 `+179°` 与 `−179°`，算术平均会得到 0°，向量平均得到 180°）。

平分后：

```java
autoYawSpread   = |wrap(yaw_B − yaw_S)|;
if (autoYawSpread > maxShooterYawSpread) {        // 默认 π/2
    autoYawTarget = yawController.getTargetYaw(); // 退化：保持上一帧目标
    autoYawFeasible = false;
} else {
    autoYawTarget = atan2(Σsin, Σcos);
    autoYawFeasible = |autoYawTarget| <= YawController.yawRange;
}
```

**退化保护（`maxShooterYawSpread`，默认 90°）**：单位向量求和在两方位相差接近 180° 时和向量趋近于零，`atan2(0,0)` 返回 0——平分线失去意义，炮口会被推到车头方向却照常判到位。夹角超过该值时直接判 `autoYawFeasible = false`，并在**不更新目标**（免得突跳）的情况下走与"平转超限"相同的安全通路。

`autoYawFeasible = false` 表示目标物理上**不可达或不可解**，此时**不自动扣扳机**，并保持上一帧的速度与仰角（见第 4 节）。

> 正后方附近 `+π−ε` 与 `−π+ε` 逐帧翻转引起的"整圈反转"由 `YawController.yawSeamHysteresis`（默认 0.1 rad ≈ 5.7°）吸收，详见第 6 节。

### 3.7 算例

设车位于 `p = (0, 0)`、`θ = 0`，球门在 `p_g = (0, 60)`（正左方 60 in），`RelPosBig = (0, 3)`、`RelPosSmall = (0, -3)`：

- 大球：`p_s = (0, 3)`，`β = atan2(57, 0) = 90°`，`yaw_B = 90°`，`d = 57 in`
- 小球：`p_s = (0, −3)`，`β = atan2(63, 0) = 90°`，`yaw_S = 90°`，`d = 63 in`
- 平分：`atan2(sin90°+sin90°, cos90°+cos90°) = 90°`，`feasible = true`
- 若 `d = 57` 落在第二档 `[60, 100]` 之外、但在 `pitchSwitchMargin` 内则继续保持第一档；否则按最近区间选第一档 `[30, 70]`，`pitch = 0.30`，`speed = round(30×57 + 900) = 2610`

---

## 4. 自动发射门控（扳机逻辑）

### 4.1 无有效定位的安全分支

若 `RobotPosition` 不存在，或 `pose` / `goal` 的 `x` / `y` / `heading` 有任何非有限值（NaN/Inf），走安全分支：

- 平转保持上一帧目标（不再重新瞄准）；
- 两个发射器 `speed = 0`、`pitch = getMaxPitch()`、扳机**关闭**；
- `autoYawFeasible = false`，返回 `FireReady(false, false)`。

### 4.2 本帧目标分配（每个发射器独立）

优先级（[Turret.resolveTarget](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L589-L601)）：

| 条件 | 速度 / 仰角 |
| --- | --- |
| `forceReleaseX == true` | 解算值 `sol.speed` / `sol.pitch`（扳机强制释放，无视就绪与不可达） |
| 否则 `!autoYawFeasible` | **保持上一帧**速度与仰角（不重新装填，等炮台转回来） |
| 否则 `allowFire == true` | 解算值 |
| 否则（不允许发射） | **预载规则** `preloadFor(...)`（见第 5 节） |

### 4.3 就绪与发射

先以 `release = false` 下发，拿到本帧机构就绪，再决定扳机：

```java
boolean bigCanFire   = autoYawFeasible && yawReady && bigReady;
boolean smallCanFire = autoYawFeasible && yawReady && smallReady;
```

即"可发射"必须同时满足：**指向可达** + **平转到位** + **飞轮与俯仰到位**。其中：

- `yawReady`：`YawController` 的到位判据（位置误差、折返残差、角速度、连续帧数，见第 6 节）；
- `bigReady` / `smallReady`：`Shooter.update(...)` 的返回值 = `isReady()`，即
  `targetSpeed > 0 && calculator.isSettled() && (now − pitchCommandTime) ≥ pitchSettleTime`。
  俯仰舵机无位置反馈，只能用"目标变更后经过 `pitchSettleTime`（0.25 s）"近似到位；飞轮用误差带稳态判据（连续 10 帧）。

**⚠️ 由此推出的行为边界**：`isReady()` 依赖 `FlywheelCalculator.isSettled()`（连续 10 帧在误差带内），而目标转速只在解算值变化超过 `speedDeadband = 20 tick/s`（对应射程约 0.6 in）时才刷新。**底盘一移动，射程逐帧变化就会不断刷新目标转速、清空稳态计数，可发射状态永远不成立**——当前实现只支持"停车发射"：行驶中（run-and-gun）在数学上不可能进入可发射状态，扳机只能靠 `fireHoldTime` 的保持窗口断续续上。这是 `speedDeadband` 与 `isSettled` 的耦合结果，不是标定问题；底盘原地旋转同理（`θ` 一变，`yaw` 目标就变）。

### 4.4 最短释放保持 `fireHoldTime`

```java
boolean yawHoldOk = autoYawFeasible && yawReady;      // 指向必须仍然有效
bigGate.update(forceReleaseBig, allowFire, autoYawFeasible, bigCanFire,
        yawHoldOk, nowSec, fireHoldTime);
smallGate.update(forceReleaseSmall, allowFire, autoYawFeasible, smallCanFire,
        yawHoldOk, nowSec, fireHoldTime);
```

四个分支的实际顺序在 `TriggerGate.update` 内（`forceRelease` 优先于 `!allowFire || !feasible`）：

| 条件 | 动作 |
| --- | --- |
| `forceRelease` | `fired = true`，保持窗口清零（松手当帧即由下一分支关闭） |
| `!allowFire \|\| !feasible` | `fired = false`，保持窗口清零 |
| `canFire` | `fired = true`，窗口推到 `now + fireHoldTime` |
| 否则 | `fired = yawHoldOk && now < holdUntil` |

要点：

- 一旦开始释放，**至少连续释放 `fireHoldTime`（0.15 s）**，条件中断时先看截止时刻，超时才关闭。
- 目的：自动瞄准下转速随射程逐帧重算，`speedDeadband = 20 tick/s` 只能吸收小抖动，边界附近 `isReady()` 会真假交替；若每帧重判，扳机舵机会逐帧开合、打断装填。
- **保持窗口只吸收"转速/俯仰就绪"的抖动，不吸收"指向失效"**：窗口内仍要求 `yawHoldOk`。否则球门切换（HIVE 翻转 → 目标跳变近 180°）或底盘持续转动时，只要出现一帧就绪，扳机就会被连续打开整个保持时长——120°/s 下 0.15 s 相当于约 **18° 偏角**出球。
- **立即关闭的四种情况**（不进保持窗口）：取消 `allowFire`、`autoYawFeasible` 变假（含夹角退化）、平转未到位（`yawReady` 假）、强制释放松手。
- 0.15 s 参考舵机单次动作时间，`fireHoldTime` 应 **≥ 舵机行程时间**，否则保持无意义。

### 4.5 返回值的语义

`FireReady(big, small)` 只表示**是否具备发射条件**（`bigCanFire` / `smallCanFire`），与 `allowFire` 和强制释放无关；**本帧是否真的扣了扳机**看遥测 `Auto big fired` / `Auto small fired`（对应 `bigGate.isFired()` / `smallGate.isFired()`）。上层若要用"已发射"做状态机，请读遥测或自行判断，不要拿返回值代替。

---

## 5. 预载（Preload）规则

`allowFire == false` 时按 [preloadFor](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L604-L616) 决定速度与仰角，四条规则按顺序判断：

| 条件 | 速度 | 仰角 |
| --- | --- | --- |
| `preload == false`（预载关闭） | 0 | `getMaxPitch()`（最高档） |
| 车辆位于不可发射区（`HypParams.NoShootArea != null && Contains(x, y)`） | 0 | `getMinPitch()`（最低档） |
| **其余情况** | `min(sol.speed, preSpeedMax)` | **解算档位 `sol.pitch`**（不随限速改变） |

设计意图：

- **预载关闭**：飞轮完全停转、炮口抬起（最高档），最安全、最省电，适合赛前/待机。
- **不可发射区**：不允许发射但也不想让球在管内滚动——停转 + 炮口压低（最低档），避免球自行滑出。
- **常规预载**：俯仰姿态与发射时**完全一致**，只把转速钳到 `preSpeedMax` 预充，触发瞬间只需补转速。
- **为什么"超速"不再抬到最高档**：`sol.speed > preSpeedMax` 只说明"转速要限"，不该改俯仰。旧实现此时把仰角改成 `getMaxPitch()`，于是按 A 的瞬间俯仰目标跳变 → `pitchCommandTime` 重置 → 必须再等 `pitchSettleTime`（0.25 s）才可能发射；松开 A 又摆回最高档，下一次再等一遍，A 键抖动时反复触发。现在最高/最低档只保留给上面两个安全姿态。


---

## 6. 手动瞄准（三种 YawMode）

手动不是"给绝对角度"，而是**把摇杆的角速度积分成一个被维持的角度**，再按模式映射成车体相对 yaw。入口 [updateManual](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L380-L413)。

### 6.1 速率积分

```math
manualAngle \mathrel{+}= \omega \cdot \Delta t
```

- `dt = now − lastManualTime`；首帧（`lastManualTime == 0`）、`dt ≤ 0` 或 `dt > 0.1 s` 时用标称值 **0.02 s**，防止一次积分大跳变。
- 首帧先用编码器实测角初始化（`rebaseManualAngle(measuredYaw(), ...)`），保证**上电瞬间目标 = 当前实际角度**，不会猛转。
- 摇杆回中 → `ω = 0` → 角度保持不变（与底盘的速度控制同构）。TeleOp 中 `ω = −stickX × maxYawRate(120°/s)`。

### 6.2 三种模式

| 模式 | 被维持的量 `manualAngle` | 映射到车体相对 yaw | 越过限位的行为 |
| --- | --- | --- | --- |
| `RELATIVE` | 车体相对 yaw 本身 | 直接使用 | **饱和钳制**到 ±`yawRange` |
| `ABSOLUTE` | 场地绝对朝向 | `out = clampToRange(manualAngle − θ)` | 输出钳制 + **回写状态** |
| `GOAL` | 相对球门方位的偏差 | `out = clampToRange(bearing + manualAngle)` | 输出钳制 + **回写状态** |

`bearing = goalBodyYaw(pose, goal)`，即两个发射器指向球门的**劣角平分线**（与自动瞄准同一公式，但 [独立实现](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L548-L560)，不调用 `solve`，以免改写 `selectedModel` 档位滞回状态）。两者共用退化判据 `bisectorYaw`：两发射器方位夹角超过 `maxShooterYawSpread`（默认 90°）时和向量退化，此时**保持上一帧有效方位**；从未有效（进入 GOAL 首帧即退化）则把当前平转目标**锁存**为基准，此后不再回读 `getTargetYaw()`。不能每帧回读：它正是本函数返回值经 `clampManualAngle` 钳制后的结果，会形成 `bearing ← 上一帧输出` 正反馈——GOAL 模式每帧把 `manualAngle` 回写为 `out − bearing` 使偏移量被锁定不再吸收，摇杆回中后目标仍按该偏移量逐帧自增并撞向 ±`yawRange`。锁存后 `bearing` 为常量，退化为 `T = C + m`，只有摇杆能改变目标。这样不会把球门方位错当成车头方向（`atan2(0,0)=0`）。

实现见 [clampManualAngle](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L472-L491)：

- **REL 必须饱和，不能折叠**。`manualAngle` 就是相对 yaw，撞限位后再推一步若"折到另一端"，炮台会反向整圈扫回，推杆方向也反转。所以用 `clampAngle`（纯饱和）。
- **ABS/GOAL 先折算到 `(-π, π]` 再钳制，并把状态回写**为输出对应的等价表示（`manualAngle = out + θ` 或 `manualAngle = out − bearing`）。回写值与原值只差 2π 整数倍，锁定语义不变；不回写则顶在限位继续推杆会把状态越积越远，反向推杆先要"消化"这段累积量，表现为一段**无响应的死区**。
- **ABS/GOAL 缺定位**（`pose == null` 或 `goal == null`）：只做折算 + 钳制输出，**不写回状态**（此时 `manualAngle` 的语义不是相对 yaw，可能停在别的 2π 分支）。

### 6.3 模式切换的连续性

`setManualYawMode(mode)`（[Turret.java](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Turret.java#L341-L350)）在 `manualInitialized` 后执行 `rebaseManualAngle(yawController.getTargetYaw(), pose, goal)`：

- 以**上一帧已钳制的目标**为基准换算新的被维持量 → 切换瞬间下发目标**连续，不跳变**；
- `rebaseManualAngle` 只在换算基准上做加减 θ / bearing，不会换到不可达角度。

循环顺序：`RELATIVE → ABSOLUTE → GOAL → RELATIVE`（`cycleManualYawMode()`）。

自动 → 手动切换时：`Turret.update(自动)` 会把 `manualInitialized = false`；再次进手动时用**实测角**重新初始化，避免沿用上一次手动的 `manualAngle` 造成一帧大角度突跳。

### 6.4 模式选择的实战建议

| 场景 | 推荐模式 |
| --- | --- |
| 想固定炮口相对车身的朝向（例如贴墙扫射） | `RELATIVE` |
| 车体在转、炮口要锁死某个场地方向（例如锁定球门侧） | `ABSOLUTE` |
| 车体乱转，只想微调"相对球门偏左/偏右" | `GOAL`（偏差 0 时与自动瞄准同向） |

---

## 7. 平转位置闭环 `YawController`

### 7.1 目标换算

```math
\text{targetTicks} = \text{yawZeroTicks} + yaw \cdot \text{tickPerRad}
```

两个参数都必须实机标定：

- `tickPerRad`：tick → yaw 的映射（**含符号**），单位 tick/rad；
- `yawZeroTicks`：机械上 `yaw = 0`（朝车头）时的编码器读数。

`motorReversed` 是**方向开关**，同时翻转功率与编码器，保证"正功率 → tick 增加"（位置环稳定的前提）。它**不能**用 `tickPerRad` 的符号代替：闭环特征值为 `−gK`（`g = d(tick)/d(power)`），与 `tickPerRad` 符号无关。该参数只在构造时下发，Dashboard 改后需重新 Init opmode。

**目标死区 `targetDeadbandTicks`**（默认 2 tick）：新目标与此前**已下发**目标的差小于该值时不重下 setpoint（`stop()` 会作废已下发值，重启后按当帧请求重新建立基准）。原因见 §7.2 第 3 点——自动瞄准的 yaw 目标每帧由位姿重算，setpoint 抖动会被 SVA 的差分前馈放大；死区把"没在动的目标"变成**严格常量**，于是 `ṙ ≡ 0`、`sgn(0) = 0`、`kV·ṙ ≡ 0`，这部分抖动从源头消失。取值必须远小于 `onTargetTolerance`（20 tick）：2 tick 在 `tickPerRad=1000` 时 ≈ 0.002 rad ≈ 0.11°。设 0 可关闭死区（退回旧行为）。

### 7.2 控制律

PIDSVA 位置环（[PIDSVAController](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/PID/PIDSVAController.java)），每帧刷新参数后 `resetSlot` 再 `calculate(targetTicks, currentTicks, dt, false)`：

```math
u = k_P e + k_I \!\int e + k_D \dot e + k_S \operatorname{sgn}(\dot r) + k_V \dot r + k_A \ddot r,\qquad e = \text{target} - \text{current}
```

默认参数：`kP = 0.005`、`kI = 0`、`kD = 0`、`kS = 0.05`、`kV = 0.0002`、`kA = 0`、`maxI = 1.0`、`iZone = 200 tick`、输出 `±1`。

**`maxI` 的语义是"积分累加量限幅"**：它钳的是 `∫e dt`（单位 tick·s），不是 I 项输出；I 项的真实上限是 `kI × maxI`。不要把它当占空比限幅读。

**`kS / kV / kA` 的参考量是"setpoint 差分"，不是规划轨迹**（[PIDSVAController.java:116-134](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/PID/PIDSVAController.java#L116-L134)）：

```text
ṙ = (setpoint − previousSetpoint) / dt     // 首帧为 0
r̈ = (ṙ − previousṙ) / dt                   // 首帧为 0
```

这个定义会带来三个可实测的后果：

1. **目标恒定时 `ṙ ≡ 0` ⇒ `Math.signum(0) = 0` ⇒ S 项恒为 0**：静摩擦前馈在"保持位置"时完全不起作用，维持位置只剩 `kP·e`，稳态静差 ≈ `kS / kP`（默认 `0.05 / 0.005 = 10 tick`，`tickPerRad = 1000` 时约 **0.57°**），且**符号取决于最后一次运动方向**（摩擦滞回）——同一目标从左右两侧靠近的落点不重合。当前 `onTargetTolerance = 0.02 rad = 20 tick` 刚好放行这个静差；一旦把容差收紧到 10 tick 以内，就会变成"永远判不到位"。
2. **目标阶跃 / 逐帧抖动时 `ṙ` 被 `dt` 放大**：1° 的 setpoint 变化 = 17.5 tick，在 20 ms 内算出 `ṙ = 875 tick/s` → `kV·ṙ = 0.175`，再加 `kS = 0.05`，**单帧踢 22% 占空比**；`kA` 项在阶跃瞬间是脉冲（默认 0 才没暴露）。
3. **自动瞄准下 setpoint 每帧都在变**：0.5° 的定位/解算噪声 = 8.7 tick/帧 = 436 tick/s → `kV` 项约 **0.087 占空比**的逐帧抖动，比正常 `kP·e`（稳态 0.02~0.05）还大。这是"自动瞄准下炮台嗡嗡抖 / 来回蹭"的代码级原因，**与参数是否标定无关**（`kS`、`kV` 默认是开启的）。**本版已用 §7.1 的 `targetDeadbandTicks` 抑制**：目标静止时 `ṙ ≡ 0`，S/V/A 三项全部归零；但**目标持续跟随移动时**（底盘在开、或 EKF 每帧都有 >2 tick 的真实变化）仍会出现单帧 `kS` 前馈脉冲——那属于残余抖动，彻底解决要么把 `kS` 置 0、要么给 setpoint 加限幅器。

**整定建议（对应上面三点）**：先把 `kS / kV / kA` 全置 0、只用 P（必要时加小 I），确认位置环基本功能与 `Yaw on target` 稳定；只有确认前馈确实带来收益、且 setpoint 已平滑（或已加 setpoint 的速度/加速度限幅）时，再逐项加回。**不要靠加大 `kP` 压第 3 点的抖动**——那只会把噪声放大得更多。

### 7.3 折返抑制（正后方抖动）

在 `±yawRange` 两端（`+π−ε ↔ −π+ε`）物理上可能是**同一朝向的两种表示**：

```java
double deltaYaw = requestedYaw - targetYaw;
if (Math.abs(deltaYaw) > Math.PI
        && Math.abs(Vector2D.normalizeAngle(deltaYaw)) <= yawSeamHysteresis) {
    seamError = Math.abs(Vector2D.normalizeAngle(deltaYaw));
    requestedYaw = targetYaw;   // 保持上一帧目标
}
```

- 判据**必须用 `yawSeamHysteresis`（0.1 rad ≈ 5.7°），不能用 `onTargetTolerance`（0.02 rad ≈ 1.1°）**：正后方附近的定位/解算噪声通常 1°~3°，用后者会反复失效，炮台按约 350° 整圈扫过去、再反向扫回，永远到不了位。
- 抑制期间的实际指向偏差计入 `cachedSeamError`，参与到位判据（`cachedSeamError ≤ onTargetTolerance`），所以**不会"偏着也判到位"**、不会带偏扣扳机。
- 抑制窗口内保持旧目标的最大物理误差 `≤ 0.1 rad`；超出窗口的真实目标变更仍会绕行整圈——硬限位下这是唯一可达路径，不是 bug。
- **由此产生的"不发火死区"**：抑制残差与 `onTargetTolerance`（0.02 rad）比较，而抑制窗口是 `yawSeamHysteresis`（0.1 rad）。因为 `0.02 < 0.1`，**物理偏差落在 0.02~0.1 rad（1.1°~5.7°）时，炮台既不转向也不判到位**，自动发射在该窗口内被整体禁止。表现为"目标贴近底盘正后方时打不了"，应写进操作手册。

### 7.4 到位判据

```java
positionOk = |targetTicks − currentTicks| < onTargetTolerance·|tickPerRad|
             && cachedSeamError <= onTargetTolerance;
rateOk     = |cachedYawRate| < max(onTargetRateTolerance, 3·(1/|tickPerRad|)/dt);
onTarget   = 连续 onTargetFrames 帧 positionOk && rateOk;
```

- `onTargetTolerance = 0.02 rad`、`onTargetRateTolerance = 0.15 rad/s`、`onTargetFrames = 3`。
- 角速度判据有下限 `3 tick / dt`：`tickPerRad` 较小时，编码器量化噪声（1~2 tick/帧）本身就会让判据永不满足。
- 连续 3 帧去抖：防止高速扫过目标时在容差带内瞬时判到位。

---

## 8. 飞轮速度环

参数集中在 [FlywheelParams](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/Shooter/FlywheelParams.java) 的 `BIG` / `SMALL` 两份实例，`Shooter` 每帧 `params.flywheel.applyTo(calculator)` 后调用一次 `calculate(...)`：

```math
\text{duty} = s\,(k_S\operatorname{sgn}\omega^\* + k_{ff}\omega^\*) + k_P(\omega^\*-\omega_f) + k_I\!\int(\omega^\*-\omega_f)
```

`s = 电压归一化系数`（`useVoltageNorm`，以 `vBusNominal = 12 V` 为基准，`[vScaleMin, vScaleMax] = [0.85, 1.25]` 限幅，`tauBus = 1.0 s` 低通）。

`Shooter.update` 中与本指南直接相关的量：

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `speedDeadband` | 20 tick/s | 目标差小于该值不刷新目标，避免位姿噪声反复清零稳态计数（≈0.6 in 射程，是"行驶中打不了"的直接原因，见 §4.3） |
| `pitchModels` | 3 个占位档 | 仰角档位表；**必须按 `pitchPos` 升序**，空表 ⇒ 永不自动发射 |
| `pitchSwitchMargin` | 5 in | 档位切换滞回裕量，防止射程在边界抖动导致俯仰目标逐帧跳变 |
| `preSpeedMax` | 2000 tick/s | 预载转速上限；解算超过它就退化为「该转速 + 最高仰角」 |
| `pitchSettleTime` | 0.25 s | 俯仰目标变更后的近似到位时间（舵机无位置反馈） |
| `triggerOnPos` / `triggerOffPos` | 1.0 / 0.0 | 扳机释放 / 关闭的舵机位置 |
| `settleFrac` | 0.02 | 稳态判据的误差带宽（相对目标转速） |
| `settleFrames` | 10 帧 ≈ 0.1 s | 连续误差在带内才算 `isSettled()` |

转速上限 `speedMax` 不在这里，而在 `FlywheelParams`（`BIG` / `SMALL` 各自 3000）。

`pitchModels` 为空时自动瞄准退化为"最高仰角 + 转速 0"，`isReady()` 因 `targetSpeed == 0` 恒为假，表现为**永不自动发射**——这是档位表未配置的典型症状。

`telemetryDetailed = true` 可展开飞轮控制量分解（`kS` 项、`kff` 项、P 项、I 项、电压系数等），用于整定与复盘。

---

## 9. 自动标定流程

按顺序做，前一步不通过不要做下一步。

### 第 1 步：机械零位与单位 —— `TurretTickTest`

OpMode：`TurretTickTest`（[文件](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/TurretTickTest.java)）

| 手柄 | 功能 |
| --- | --- |
| GP1 左摇杆 X | 平转开环功率（限幅 `maxYawPower = 0.5`） |
| GP1 左摇杆 Y / 右摇杆 Y | 小球 / 大球俯仰舵机位置 |
| GP2 左摇杆 Y / 右摇杆 Y | 小球 / 大球扳机舵机位置 |
| GP1 A | 编码器计数清零 |

标定量：

1. **`tickPerRad`**：A 清零 → 手动转一个**已知角度** Δ（建议 ≥ 90°，用角度尺或场地贴线）→ 读累计 tick `T` → `tickPerRad = T / toRadians(Δ)`。注意**符号**：正功率使炮台逆时针转时为正，若为负说明方向反了，应改 `motorReversed` 而不是保留负 `tickPerRad`。
2. **`yawZeroTicks`**：把炮台摆到机械上 `yaw = 0`（两发射器平行朝车头）→ A 清零或直接读 tick。
3. **`pitchMin` / `pitchMax`**：扫俯仰舵机全行程，记录舵机不打滑、不顶死两端的位置。
4. **`triggerOnPos` / `triggerOffPos`**：扫扳机舵机，记录"完全释放（能过球）"与"完全挡住"两端。

舵机**没有位置反馈**，以上全部靠目视 + 试射确认。

### 第 2 步：平转闭环整定 —— `TurretYawTest`

OpMode：`TurretYawTest`

| 手柄 | 功能 |
| --- | --- |
| GP1 十字键 左/右 | 目标角步进 `angleStepDeg = 10°`（长按 `repeatPeriodMs = 150` 连续步进） |
| GP1 A | 目标归零 |
| GP1 B | `yawController.stop()` 复位控制器状态 |

遥测含 `Yaw target/current/error (deg)`、`Yaw rate`、`Yaw power`、`Yaw on target`、`Yaw seam error`、`Yaw ticks`。

整定顺序：

1. 先把 `kS/kV/kA` 置 0，`kI/kD` 置 0，只调 `kP`：从 0.001 起加，直到 ±90° 阶跃**无持续振荡**、且能在 1 s 内接近目标。
2. 有稳态误差再加 `kI`（配 `iZone = 200 tick`、`maxI = 1.0`）；注意积分在折返抑制期间会累积，若发现过冲明显就把 `kI` 减半。
3. `kD` 一般保持 0（编码器量化噪声大，微分容易放大噪声）。
4. 最后加 `kS`（克服静摩擦，让起步更快）与 `kV`（跟随速度前馈）。若加了 `kV` 后在静止目标上出现"顶住不动/来回蹭"，说明参考速度在抖动，直接把 `kV` 设 0。
5. 用 `Yaw on target` 验证：阶跃后应在约 0.1~0.3 s 内稳定变 true，且**不应在 true/false 间长期抖动**。若抖动，加大 `onTargetTolerance` 或 `onTargetFrames`，不要靠加 `kP`。

### 第 3 步：手动联调 —— `TurretManualTest`

OpMode：`TurretManualTest`

| 手柄 | 功能 |
| --- | --- |
| GP2 左摇杆 X | 手动平转**目标角**（`targetYaw = −stickX × yawRange`，直接位置控制） |
| Dashboard | `targetPitchBig` / `targetPitchSmall` 设仰角 |
| GP2 十字键 上/下 | 大球转速 ±`speedStep = 100` |
| GP2 十字键 左/右 | 小球转速 ±`speedStep` |
| GP2 RB / LB | 大球 / 小球扳机 |
| GP2 RT | intake（`sweeperMotor`）直通 |

检查项：摇杆打到限位时是否**停住而不是绕整圈**、`Fire ready big/small` 是否随转速到位变真、电池电压跌落时飞轮是否仍能到位。

> 注：本 OpMode 直接下发**目标角**（`−stickX × yawRange`），不经过 `updateManual` 的速率积分与三模式，因此 `RELATIVE / ABSOLUTE / GOAL` 的验证要在 `TeleOpMainBase`（GP2 X 循环）或 `AimModelCalibration`（GP2 X 循环）里做。

### 第 4 步：射程-转速模型标定 —— `AimModelCalibrationTuner`

OpMode：`AimModelCalibration`（group `Tuning`，[文件](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Tuning/AimModelCalibrationTuner.java)）

**主控（GP1）**

| 按键 | 功能 |
| --- | --- |
| 左摇杆 / 右摇杆 X | 底盘平移 / 旋转 |
| LB / RB | 集球器 吃球 / 输出 |
| A / B | 把最近一发标为**大球 / 小球命中**（未标记的样本不参与回归） |
| X | 位姿重置到本方 `ResetPose` |
| Y | 撤销最近一条样本（两个发射器按时间取最新） |

**副控（GP2）**

| 按键 | 功能 |
| --- | --- |
| 左摇杆 X | 平转**角速度**（回中即停） |
| X | 循环平转模式 `RELATIVE → ABSOLUTE → GOAL` |
| 十字键 上/下 | 大球目标转速 ±`speedStep` |
| 十字键 左/右 | 小球目标转速 ±`speedStep` |
| RB / LB | 大球 / 小球扳机（按住释放，每个上升沿记一发） |

**仰角由 Dashboard 的 `targetPitchBig` / `targetPitchSmall` 直接设置，不由手柄控制。**

流程：

1. Dashboard → 打开 `AimModelCalibration` → 点 **Start**（先开 Logging）。
2. 上电时把车放在 `HypParams.ResetPoseRed/Blue` 对应的位置（X 重同步用同一参照点）。
3. 把 `targetPitchBig/Small` 设成**现有某一档的 `pitchPos`**（例如 0.30）。若设成档位表里没有的值，该发样本匹配不到档位（`pitchMatchTol = 0.02`），**不参与回归**。
4. 把车开到该档适用射程区间内的多个距离（建议覆盖区间两端 + 中间，至少 3 个点、跨度 ≥ `minSpanIn = 8 in`），每个距离调转速直到稳定命中，然后按 **A（大球）/ B（小球）** 标记命中。
5. 每发样本自动记录：`range / targetSpeed / actualSpeed / pitch / aimErrorDeg / ready / busV`。参与回归的条件是：

   ```text
   hit == true
   && (!requireSettled || ready)          // ready = 发射瞬间 Shooter.isReady()
   && |aimErrorDeg| <= aimToleranceDeg    // 默认 2.0°
   && |pitch − model.pitchPos| <= pitchMatchTol
   ```

   其中 `aimErrorDeg = normalize(bearing − (θ + getCurrentYaw()))`，即"炮口实际指向与理想指向的偏差"。
6. 每档内做一元线性回归 `stats.add(range, targetSpeed)` → `speed = slope·range + intercept`，并输出 `R²` 与样本数 `n`。
7. **档位边界推断**：相邻两档的边界取 `0.5 × (本档实测射程最大值 + 下档实测射程最小值)`；首档下限、末档上限各向外扩 `edgeMarginIn = 5 in`。
8. 不达标（`n < 2` / 跨度不足 / 斜率非正）会标 `invalid`，补样本重做。
9. **Stop** → 导出 CSV（Dashboard Logging），并读 Driver Station 上的建议行：

   ```text
   >>> BIG_MODEL[0] new PitchModel(0.300, 40.0, 82.0, 31.20, 880.0) n=5 R2=0.97
   ```

10. **人工**把建议值抄回 [Shooter.java 静态块](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Controllers/Turret/Shooter.java#L190-L203)。该 OpMode **不会**自动写回。

对每个档位重复 3~9 步（大球、小球各自独立，惯量与齿比不同）。

注意事项：

- 每次打点前先确认**平转已到位**（遥测 `AutoYaw on target` / `aimErrorDeg` 在容差内），否则样本会被 `aimToleranceDeg` 过滤掉。
- `requireSettled = true` 时只接受发射瞬间飞轮已稳定的样本；若飞轮整定不理想可临时设 false，但回归结果偏差会变大。
- 每个发射器最多保留 `maxShotsKept = 60` 发（超出丢最旧），标定中途不必重启。
- 同一档位内**不要改 `targetPitch`**，否则样本会散到不同档、无法回归。

### 第 5 步：`RelPosBig` / `RelPosSmall`

这两个值只影响"发射器绕平转轴"的位置修正，量级很小（几英寸），但有三个实际作用：

1. 两个发射器指向同一球门时 `yaw` 不同，平分角补偿；
2. `GOAL` 手动模式的 `bearing` 基准；
3. 极近距离（< 20 in）时射程差异明显。

标定方法（任一）：

- **卷尺法**：以机器人几何中心为原点，量发射器旋转中心的车体坐标 `(x前, y左)`，直接填 `RelPosBig/Small`（注意 `y` 向左为正）。
- **指向法**：车停在已知位姿，把炮台转到使炮口对准远处一个已知场地坐标的目标，用测得 `yaw` 反解出 `RelPos`（两条不同方向的观测即可三角定位）。

标定完请把 `todo:` 注释删掉。

### 第 6 步：整机验收

在 `TeleOpMainBase` 里按第 10 节的比赛操作跑一遍：自动瞄准 → 等 `Fire ready` 变真 → 按住 A 发射；确认三个距离档都命中、正后方附近无整圈扫动、电池跌到 11 V 仍能到位。

---

## 10. 比赛操作（`TeleOpMainBase`）

OpMode：[TeleOpMainBase](file:///F:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/CompetitionOpmode/TeleOpMainBase.java)

### 主控 GP1（底盘）

| 按键 | 功能 |
| --- | --- |
| 左摇杆 | 平移 |
| 右摇杆 X | 旋转 |
| LT / RT | 按住时车头直接瞄准 `aimGoal(true/false)` 选定的球门：红方 LT = AudienceDown / RT = AudienceUp，蓝方相反；只影响车头，不写回炮台球门 |
| X | 无头模式切换（field-centric / robot-centric） |
| A | 重置定位到 `ResetPose` |
| LB / RB | 集球器 吃球 / 输出 |

### 副控 GP2（炮台）

| 按键 | 功能 |
| --- | --- |
| **B** | 切换 **AUTO / MANUAL** |
| **Y** | 切换预载（`preload`） |
| **LT / RT（按住）** | 手动选择炮台球门：红方 LT = AudienceDown / RT = AudienceUp，蓝方相反；仅在本帧 MT1 无有效（非 MIDDLE）读数时生效，并保持到下一次有效读数 |
| **X** | 循环手动模式 `RELATIVE → ABSOLUTE → GOAL` |
| **A（按住）** | 自动模式：允许发射（松开即取消，扳机立即关闭） |
| RB / LB | 自动模式：强制释放 大球 / 小球扳机（无视就绪）<br>手动模式：大球 / 小球扳机 |
| 左摇杆 X | 手动平转角速度（满偏 = ±120°/s） |
| 左摇杆 Y | 小球俯仰（`axisToRange` 映射到 `pitchMin..pitchMax`） |
| 右摇杆 Y | 大球俯仰 |
| 十字键 上/下 | 大球目标转速 ±100 tick/s |
| 十字键 左/右 | 小球目标转速 ±100 tick/s |

自动模式调用 `turret.update(allowFire, preload, forceBig, forceSmall)`；手动模式调用 `turret.updateManual(...)`。

遥测 `Aim goal side` / `Aim goal (shared)` 显示炮台当前球门；按住 GP1 扳机时车头可能临时指向另一个门，属预期。

### 操作要点

1. **待机/行进**：AUTO 模式、松开 A、**预载开**（Y）。炮台会预充转速、自动跟球门。
2. **发射**：确认 `Fire ready big/small` 为真（或至少平转到位），**按住 A**；离开发射位立刻松开。
3. **强制释放**只在"机构显示未就绪但确认可以打"的应急场景用，它**无视平转是否到位**，容易打偏。
4. **手动微调**：X 切到 `GOAL` 模式，用左摇杆 X 打相对球门的小偏差，松杆即锁定；`RELATIVE` 适合固定车体相对朝向。
5. **断定位**：A 重置位姿（车身必须放在约定位置）。定位失效时自动瞄准自动进入安全分支（炮口抬起、飞轮停转、不发射）。

---

## 11. 遥测字段速查

`turret.setTelemetry()` 输出（需调用方自行 `telemetry.update()`）：

| 字段 | 含义 |
| --- | --- |
| `Yaw target / current / error (deg)` | 平转目标 / 实测 / 误差 |
| `Yaw rate (rad/s)` | 编码器差分角速度 |
| `Yaw power` | 平转输出功率 |
| `Yaw on target` | 到位判据（连续 3 帧） |
| `Yaw seam error (deg)` | 折返抑制残差（未抑制为 0） |
| `Yaw ticks` | 编码器读数 |
| `AutoYaw target (deg)` | 平分后的自动平转目标 |
| `AutoYaw feasible` | 目标是否在 ±`yawRange` 内且夹角未退化 |
| `AutoYaw spread (deg)` | 两发射器方位角夹角（超过 `maxShooterYawSpread` 即判不可行） |
| `AutoYaw on target` | 平转是否到位 |
| `Auto preload` | 预载开关 |
| `Auto in no-shoot area` | 是否位于不可发射区 |
| `Auto big/small fired` | 本帧是否**实际**释放扳机 |
| `Auto big/small range / pitch / speed` | 解算射程、仰角、转速（**本帧解算值**，未经死区锁存） |
| `Manual yaw mode` / `Manual angle` / `Manual yaw target` | 手动模式、被维持量、本帧下发目标 |
| `<设备名> ...` | 各发射器的飞轮目标/实测/误差、`Fire ready`、电流、母线电压 |

> **`<设备名> speed target` 是"死区锁存后"的目标值，不是本帧解算值**：`Shooter.update` 只在变化超过 `speedDeadband` 时才刷新它。因此 `Auto big speed`（解算）与 `<设备名> speed target`（实际下发）最多可永久相差一个死区（约 20 tick/s）。排查"目标转速与射程对不上"时以 `Auto … speed` 为准。
>
> `Manual …` 三行只在手动模式下出现；`Auto …` 段只在自动模式下出现（两者由同一个 `aimSource` 枚举 `NONE / AUTO / MANUAL` 互斥控制）。

---

## 12. 常见故障排查

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 按住 A 但扳机不动 | `isReady()` 恒假：`targetSpeed == 0`（档位表空或 `speedMax` 为 0）；飞轮 `isSettled()` 不成立；`pitchSettleTime` 被逐帧重置 | 先看遥测 `Auto ... speed`；`pitchModels` 是否为占位/空；调 `speedDeadband` 或飞轮参数 |
| 扳机一开一合、球被卡住 | `fireHoldTime` 小于舵机行程；`pitchSwitchMargin` 太小导致档位逐帧跳变 | 加大 `fireHoldTime`（≥ 舵机动作时间）、`pitchSwitchMargin` |
| 正后方附近炮台整圈来回扫 | `yawSeamHysteresis` 太小（被定位噪声穿越） | 加大到 0.1~0.2 rad；不要用 `onTargetTolerance` 代替它 |
| 平转输出直接打满、越推越远 | `motorReversed` 方向错（位置环变正反馈） | 置 `motorReversed = true` 并重新 Init |
| 目标角正确但机械停歪 | `yawZeroTicks` / `tickPerRad` 标定错（尤其符号） | 重做第 1 步 |
| 炮台"偏着"却判到位并自动发射 | 折返抑制残差未参与判据（代码已处理）；或 `onTargetTolerance` 过大 | 检查 `Yaw seam error`；容差不要超过 0.03 rad |
| 近/远距离总是差一点 | `RelPosBig/Small` 未标定；档位边界/斜率未回归 | 做第 4、5 步 |
| 电池电压低时飞轮不到位 | 电压归一化限幅 `vScaleMin` 太严或 `kff` 偏小 | 查 `telemetryDetailed` 的 duty 分解，必要时放宽 `vScaleMin` |
| 自动/手动切换瞬间炮台猛转 | 手动状态未重新初始化（代码已处理：`manualInitialized = false`） | 若仍出现，检查是否绕过 `Turret` 直接调 `YawController` |
| 手动顶住限位后反向推杆无响应 | ABS/GOAL 未回写状态（代码已处理）；REL 用错折叠而非饱和 | 确认走的是 `clampManualAngle` |
| 自动瞄准下炮台持续抖动 / 来回蹭 / 发烫 | `kS/kV` 前馈由 **setpoint 差分**推导，逐帧位姿噪声被 `1/dt` 放大（§7.2 第 3 点） | 先把 `kS = kV = kA = 0`，只用 P 验证；确认后再逐项加回 |
| 车一动、或一原地旋转，按住 A 就不打 | `speedDeadband` 与 `isSettled()` 耦合：射程逐帧变化 → 目标转速不断刷新 → 稳态计数永远清零（§4.3） | 属当前实现的行为边界，只能**停车发射**；不要靠加大 `speedDeadband` 硬顶（会牺牲出球一致性） |
| 目标贴近底盘正后方时永远 `ready = false` | 折返抑制窗口 0.1 rad > 到位容差 0.02 rad，物理偏差在 0.02~0.1 rad 时既不转向也不判到位（§7.3） | 属预期行为；转动底盘让目标离开正后方窗口 |
| 同一目标从左右两侧靠近，停的落点不一致 | 目标恒定时 S 项归零，只剩 `kP·e` 抗摩擦，静差方向取决于最后运动方向（§7.2 第 1 点） | 参数无法消除；需 setpoint 平滑/规划，或接受 ~0.5° 的方向性偏差 |
| 电池电压正常、机构也在转，但永不自动发射 | `pitchModels` 为空（或未填标定值）→ 解算速度 0 → `targetSpeed == 0` → `isReady()` 恒假 | 填档位表（第 4 步），或先用强制释放验证机构 |

---

## 13. 已知问题与待统一项（截至本次文档同步）

本节记录**代码事实**与**跨文件不一致**，供后续修改时对照。本文其余各节一律按"现在的代码"描述，不按"应该是什么"描述。

| # | 现象 | 位置 | 影响 |
| --- | --- | --- | --- |
| 1 | 球门选择曾有两套相反规则：`RobotPosition` 的 `MIDDLE` 取上次抬升侧的**相反侧**，`AutoAimAction` 的 `MIDDLE` **保持上次锁定侧** | `RobotPosition.java`、`AutoAimAction.java:153-168` | **已修**：`RobotPosition` 统一为"抬升侧=同侧，`MIDDLE`/无读数保持上次选择（含 GP2.LT/RT 手动输入），本帧非 MIDDLE 观测优先"。`AutoAimAction` 仍是独立实现（自动阶段尚未接入 `Turret`，故当前不冲突）；待办：`AutoAimAction` 改为直接读 `RobotPosition.getGoalPose()` |
| 2 | `AutoAimAction` 的超时与成功都 `return false`，只有遥测 `AutoAim=Timeout` 能区分 | `AutoAimAction.java:135-137,171-180` | 用 `SequentialAction` / `ActionRunner` 无法做"对齐成功才允许发射"的门控 |
| 3 | 平转 SVA 前馈取 setpoint 差分 | `PIDSVAController.java:116-134` | 静止时静摩擦前馈消失（方向性静差）、逐帧噪声被 `1/dt` 放大（自动模式抖动）。见 §7.2。**部分已修**：`YawController.targetDeadbandTicks` 使"目标不变"时 `ṙ ≡ 0`，逐帧抖动消失；**静差与"目标持续移动时的单帧前馈脉冲"仍在** |
| 4 | `autoYawFeasible` 分支在默认 `yawRange = π` 下不可达（平分角恒在 (-π, π]） | `Turret.java:272` | 保留无害，但"平转超限"路径实际从未被验证，只有调小 `yawRange` 才会生效 |
| 5 | 自动模式每帧对扳机写两次（先 `update(..., false)`，后 `setTrigger(true)`） | `Shooter.updateMechanisms` + `Turret.update` 末尾统一写一次 | **已修**：机构更新与扳机分离，每帧只写一次；末次生效语义不变 |
| 6 | `Turret.measuredYaw()` 直接读电机编码器，绕过"硬件只在 update 内采样"的约定 | `YawController.getMeasuredYaw()` | **已修**：读数下沉到 `YawController`，并保留独立于 `getCurrentYaw()`（后者读缓存，首帧为 0，不能用于手动首帧初始化） |
| 7 | `Vector2D.normalizeAngle` 用 while 循环逐次 ±2π，末尾还有一次 `fromPolar(...).getRadian()` 冗余往返 | `Vector2D.java:178-190` | 当前所有调用点输入有界，暂不触发；若把未限幅的积分量传进来，极小概率长时间空转 |
| 8 | `getMinPitch()` / `getMaxPitch()` 依赖档位表按 `pitchPos` 升序，无校验 | `Shooter.java:432-449` | 表未排序时静默给出错误的预载安全姿态 |
| 9 | TeleOp 球门有三条入口：MT1 非 MIDDLE 观测、GP2.LT/RT 手动设置、GP1.LT/RT 车头独立选门 | `RobotPosition.java`、`TeleOpMainBase.java` | 前两者写入同一个共享球门（MT1 本帧观测优先，否则保持含手动输入）；GP1 只在按住扳机时改车头目标，不写回共享球门，故车头/炮口可暂时指向不同门（有意设计） |

### 本次文档同步的改动

- 2026-11 瞄准代码重构（行为不变）：§3.1 公式位置改指 `utility/AimGeometry`；§4.4 解释器由内联字段改为 `TriggerGate`（并补分支顺序表）；§4.5 遥测来源改为 `bigGate/smallGate`；全篇行号锚点按重构后的文件更新；§13 第 5、6 条标记为已修。
- `TurretGuide.md`：§3 补「球门来源」与 `MIDDLE` 规则；§3.4 补档位表排序契约与重叠区非单调；§4.3 补「行驶中无法进入 ready」的行为边界；§7.2 重写 SVA 前馈语义（`maxI` 单位、静止 S 项归零、噪声放大）；§7.3 补折返抑制的「不发火死区」；§8 / §11 补参数与遥测语义；§12 增补 5 条排查项；新增本节。
- `TurretPrompt.md`：`Shooter` 由 PIDSVA 改为 `FlywheelCalculator`，补 `solve` / `setTrigger` / `getMinPitch` / `getMaxPitch` 与完整 `Params` 字段；补 `updateManual` 的语义。
- `AutoAimPrompt.md`：补 `speedDeadband`、`pitchSwitchMargin` 两处滞回的必要性、空档位表的后果、档位表排序与重叠契约。
- `TurretTestPrompt.md`：`TurretManualTest` 的仰角改为 Dashboard 设置（与代码一致），补 右 trigger = intake。

### 2026-11 缺陷修复（P0/P1）

| 级别 | 问题 | 修复 | 涉及文件 |
| --- | --- | --- | --- |
| P0 | 触发保持窗口只比时间、不校验指向，球门切换或底盘转动时可带着约 18° 偏角连续出球 | 保持分支追加 `yawHoldOk = autoYawFeasible && yawReady` | `Turret.java`（`update(boolean,boolean,boolean,boolean)`） |
| P0 | 车头按固定 Up/Down 门转向、炮台按 HIVE 动态门瞄准，两者可指向相反球门 | 车头改为 **GP1.LT/RT 直接选门**（颜色映射与 GP2 一致），只影响车头、不写回共享球门；`chassisAimGoal(boolean)` 直接返回 `aimGoal(leftTrigger)`（曾短暂改为优先取 `RobotPosition.getGoalPose()`，现按操作需求回退） | `TeleOpMainBase.java` |
| P1 | `RobotPosition` 的 `MIDDLE` 取"上次抬升侧的相反侧"，与另两个分支的"抬升侧=同侧"约定相反，一帧 MIDDLE 就翻转球门 | `MIDDLE` 改为保持上次选择（滞回），从未观测到抬升侧时默认 AudienceUp | `RobotPosition.java` |
| P1 | 两发射器方位接近 180° 时劣角平分线退化（`atan2(0,0)=0`），却仍判可行并自动发射 | 新增 `maxShooterYawSpread`（默认 90°）：超限直接判 `autoYawFeasible = false`，目标保持不变，走平转超限的安全通路；遥测新增 `AutoYaw spread (deg)` | `Turret.java` |
| P1 | 手动 `GOAL` 模式用同一平分公式但缺退化判定，退化时会把球门方位当成车头方向并照常跟随 | 退化判定抽为共用 `bisectorYaw`（超限返回 NaN）；`GOAL` 保持上一帧有效方位，从未有效则**锁存**当前平转目标为基准（不得每帧回读 `getTargetYaw()`，否则 `bearing ← 上帧输出` 正反馈会让摇杆回中后目标持续漂移撞限位） | `Turret.java`（`goalBodyYaw` / `clampManualAngle`） |
| P1 | 退出自动瞄准时只清 `*FireHoldUntil`，未清 `autoBigFired/autoSmallFired`，`isBigFiring()/isSmallFiring()` 可永久残留 true（且 `autoAimActive=false` 时遥测不显示 Auto fired 行，属静默残留） | 新增 `clearAutoFireState()`，`update(double,...)` 与 `updateManual(...)` 开头统一清四个自动发射状态 | `Turret.java` |
| P1 | 档位滞回把**转速模型**一并锁住：射程越出该档标定区间（≤ `pitchSwitchMargin`）后仍按旧档公式**线性外推**，转速偏大/偏小、甚至顶到 `speedMax`，使 `isSettled()` 永不成立（档位边界附近发不出去） | `solve()` 查转速模型前把射程钳回该档 `[minRange, maxRange]`（新增 `clampToModelInterval`）：区间外取边界值、不外推；俯仰仍按滞回保持 | `Shooter.java` |
| P1 | 预载在 `sol.speed > preSpeedMax` 时把仰角改成 `getMaxPitch()`，与发射档位不一致 → 按 A 瞬间俯仰目标跳变、`pitchCommandTime` 重置，**每次发射多等 `pitchSettleTime`(0.25 s)**；松开 A 又摆回最高档，A 键抖动时反复触发 | 预载只钳制**转速** `min(sol.speed, preSpeedMax)`，仰角一律用解算档位；最高/最低档仅保留给"预载关闭 / 禁射区"两个安全姿态 | `Turret.java`（`preloadFor`） |
| P2 | 自动瞄准的 yaw 目标每帧由位姿重算，定位噪声经 PIDSVA 的 setpoint 差分前馈（`kV·ṙ`、`kS·sgn(ṙ)`）放大成逐帧 ±0.05~0.16 占空比抖动 → 炮台静止瞄准时嗡嗡抖、`onTarget` 反复进出 | 新增 `YawController.targetDeadbandTicks`（默认 2 tick）：目标变化不足死区时不重下 setpoint，静止时 `ṙ ≡ 0`、SVA 三项归零；`stop()` 作废已下发值。转速侧早有同构的 `speedDeadband` | `YawController.java` |

### 2026-11 球门选择变更（手动设置 + 车头独立选门）

| 类型 | 变更 | 实现 | 涉及文件 |
| --- | --- | --- | --- |
| 功能 | 手动阶段允许操作手用 GP2.LT/RT 指定炮台球门 | `RobotPosition.setManualGoalDown(boolean)` 写入 `goalDown` / `lastLiftedSide` / `cachedGoalPose`；`update()` 改为只在**本帧** MT1 非 MIDDLE 观测时覆盖，`MIDDLE` / 无读数保持上一帧（含手动输入）。TeleOp 在 `update()` 前处理扳机：红方 LT=Down / RT=Up，蓝方相反 | `RobotPosition.java`、`TeleOpMainBase.java` |
| 变更 | GP1.LT/RT 改为直接选择车头球门 | `chassisAimGoal()` 直接返回 `aimGoal(leftTrigger)`，不调用 `setManualGoalDown`、不写回共享球门；只影响按住扳机期间的车头朝向。颜色映射与 GP2 一致；遥测 `Aim goal (shared)` / `Aim goal side` 仍表示炮台共享球门 | `TeleOpMainBase.java`、`TeleOpRed.java`、`TeleOpBlue.java` |

> 未处理（本次范围外）：`AutoAimAction` 的首帧微分冲击与"成功/超时都返回 false"（自动阶段）；§12 表中 P2 类问题（超量程外推仍可自动发射等）。
