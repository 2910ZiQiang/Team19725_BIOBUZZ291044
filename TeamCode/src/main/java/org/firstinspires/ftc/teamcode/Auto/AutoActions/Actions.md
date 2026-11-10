# AutoActions 使用说明

本目录（`Auto/AutoActions/`）汇总自动阶段可入队执行的 Road Runner `Action`。
所有移动类 Action 的位姿来源统一为 **`AdaptiveEKFLocalizer` 融合定位系统**
（Pinpoint 里程计 + Limelight MT1 视觉的自适应 EKF 融合位姿）：

- 构造时通过 `RobotPosition.getInstance().getPose2d()` 取起点位姿（轨迹在构造时刻固化）；
- 执行时轨迹跟随内部持续调用融合定位器推进；
- `AutoAimAction` 每帧直接读 `AdaptiveEKFLocalizer.getPose()`；
- 组合类 Action（`AutoCycleAction` / `ContinuousRouteAction`）把移动段交给
  `utility/DeferredAction` 延迟构造，保证仍以**执行时的真实位姿**规划。

## 通用约定

1. **串行队列执行**：由 `utility/ActionRunner`（`add` / `update` / `isBusy` / `clear`）调度，
   OpMode 主循环中每帧调用 `actionRunner.update()`。
2. **非阻塞**：每个 Action 的 `run(packet)` 每帧执行一次，返回 `true` 表示未完成，
   返回 `false` 表示结束（队列自动切换下一个 Action）。
3. **构造时刻固化**：轨迹类 Action 应在**入队前、机器人位置基本确定后**再 new，
   不要提前构造后跨多个状态复用。
4. **Dashboard 调参**：带 `@Config` 的 Action，其 `public static` 参数可在 FTC Dashboard
   运行时实时修改，无需重新部署。
5. **单位**：坐标英寸，角度弧度（`strafeToLinearHeading`），时间毫秒。
6. **时间最优**：所有移动类 Action 都必须以最短时间走完整条路径——
   起步冲到最大速度、中段巡航、末端最快停止（梯形速度曲线）。
   实现方式：**一段移动只用一个 `actionBuilder` 链**，中间点作为链式途经点
   （段间速度连续、不中途停车）；约束来源为 `MecanumDrive.Params` 五参数。
   原理、标定与验证方法见
   [`RoadRunner/时间最优运动.md`](../../RoadRunner/时间最优运动.md)。
7. **多段组合**（`SequentialAction` / `ParallelAction`）：可把一个完整周期合成单个 Action 入队，
   并用 `utility/DeferredAction` 包装移动段，使其仍在**执行时**才构造（避免用旧位姿规划）。
   但需注意：`MecanumDrive.actionBuilder` 的 `beginEndVel = 0`，Road Runner 的 `profile()`
   用同一个值同时作为**起点与终点速度**，因此**每一份 `build()` 出来的轨迹必然以 v=0 结束**——
   任何 Action 组合器都无法消除两条独立轨迹之间的物理停车。
   「周期组合」见第 13 节，「单条轨迹不停车」见第 14 节。

## Action 一览

| Action | 作用 | 构造参数 |
|--------|------|---------|
| `GoToStopPose` | 前往停车位姿并停车 | `(MecanumDrive drive, Pose2d stopPose)` |
| `GoToShootAreaAction` | 抵达指定发射区域内离机器人最近的点，车头预瞄指定球门 | `(TeamColor, int areaIndex, Pose2d goal)` |
| `GoToShootAreaNearestVerticesAction` | 单段连续轨迹依次经过两个发射区域各自距原点最近的顶点 | `(TeamColor, Pose2d goal)` |
| `GoToEatPointAction` | 到指定取球点（intake 到位、车头朝取球点） | `(int pointIndex)` |
| `GoToNearestEatPointAction` | 自动选近的取球点并前往 | 无 |
| `GoToUnvisitedEatPointViaHiveAction` | 经 Hive 正下方前往此前未访问的取球点 | `(TeamColor)` |
| `GoToOtherShootAreaViaHiveAction` | 经 Hive 正下方前往另一发射区域并预瞄球门 | `(TeamColor, Pose2d goal)` |
| `GoToUnvisitedEatPointAvoidHiveAction` | 绕开 Hive（外侧）前往此前未访问的取球点 | `(TeamColor)` |
| `GoToOtherShootAreaAvoidHiveAction` | 绕开 Hive（外侧）前往另一发射区域并预瞄球门 | `(TeamColor, Pose2d goal)` |
| `AutoAimAction` | 原地 PD 转向，自动锁定当前 HIVE 侧球门 | `(TeamColor)` |
| `EatAction` | 集球器以吃球速度运转指定时长后停转 | `(Sweeper)` 或 `(Sweeper, long durationMs)` |
| `AdaptiveParkAction` | T-3s 检测整机是否接触场地边框，接触则开到安全位置停靠，否则原地停止 | `(MecanumDrive, ElapsedTime matchTime, long durationMs, long leadTimeMs)` |
| `AutoCycleAction` | **保守版**：把「去取球点 → 吃球 → 去另一发射区 →（可选）瞄准」合成一个 Action（取球点仍停车） | `(TeamColor, Sweeper, Pose2d goal)` |
| `ContinuousRouteAction` | **激进版**：去取球点 + 路过式吃球 + 去另一发射区，全程**单条轨迹不停车** | `(TeamColor, Sweeper, Pose2d goal)` |

---

## 1. GoToStopPose

**作用**：用 `strafeToLinearHeading` 从当前融合位姿一次性平移旋转到停车位姿。

**构造**：`new GoToStopPose(drive, HypParams.StopPoseRed)`
（蓝队用 `HypParams.StopPoseBlue`）

**完成条件**：Road Runner 轨迹执行完毕。

## 2. GoToShootAreaAction

**作用**：
1. 按队伍颜色与序号选择发射区域（`ConvexPolygon`）：
   `FIRST=1` / `SECOND=2`，非法序号抛 `IllegalArgumentException`；
2. 用 `ConvexPolygon.NearestVectorFrom` 求多边形上离机器人最近的点作为目标位置；
3. 航向指向调用方传入的**单个具体球门**（与手动阶段
   `Chassis#update(double, double, Pose2d)` / `Chassis#aimOmega` 完全一致）；
4. `strafeToLinearHeading` 到位。精确瞄准由后续 `AutoAimAction` 完成。

**构造**：
```java
new GoToShootAreaAction(TeamColor.RED,
                        GoToShootAreaAction.FIRST,
                        HypParams.RedAudienceUp);
```

## 3. GoToShootAreaNearestVerticesAction

**作用**：
1. 计算机器人到本颜色两个发射区域边界的最近距离，确定最近区域与另一区域；
2. 分别取两个多边形中距场地原点最近的顶点，航向指向传入的具体球门；
3. 在**同一个** `actionBuilder` 链上用两次 `strafeToLinearHeading` 构建**单段连续轨迹**：
   起点 → 最近区域顶点 → 另一区域顶点；两个顶点均为途经点，机器人在巡航中依次通过
   （**不中途停车**），以最短时间完成整条路径。

**构造**：
```java
new GoToShootAreaNearestVerticesAction(TeamColor.RED, HypParams.RedAudienceUp);
```

## 4. GoToEatPointAction

**作用**：取球点为底盘前端 intake 的目标点（`HypParams.EatPoint1=(24,-72)`、
`EatPoint2=(-72,-24)`，场地固定坐标不分红蓝）。构造时：
1. 用 `ChassisGeometry.centerPoseFromIntakeTarget` 由 intake 目标点反推底盘中心位姿
   ——前端 intake 恰好落在取球点，且车头面朝取球点；
2. `strafeToLinearHeading` 到位。到位后通常入队 `EatAction`。

**构造**：`new GoToEatPointAction(GoToEatPointAction.FIRST)`（或 `SECOND`，非法序号抛异常）。

## 5. GoToNearestEatPointAction

**作用**：构造时计算底盘中心到 `EatPoint1` / `EatPoint2` 的直线距离，自动选近者，
其余同 `GoToEatPointAction`。

**构造**：`new GoToNearestEatPointAction()`。

## Hive 路线几何约束（`GoTo*ViaHive` / `GoTo*AvoidHive` 共用）

以下 4 个 Action 均继承 `HiveRouteAction` 基类，由 `utility/HiveRoutePlanner` 生成途经点，
保证对外接矩形（整机轮廓）的几何约束：

- **Hive 正下方矩形**：红队 x∈[-24,24]、y∈[-25,-1]（`HypParams.HiveRectRed`）；
  蓝队为**中心对称**区域 x∈[-24,24]、y∈[1,25]（`HypParams.HiveRectBlue`）。
- **Hive 禁线**：红队为 (24,-24)–(-24,-24) 两点间线段（`HypParams.HiveForbiddenLineRedA/B`）；
  蓝队为 (24,24)–(-24,24)（`HypParams.HiveForbiddenLineBlueA/B`）。
- **整机轮廓**：底盘外接矩形，半长 / 半宽为 `HypParams.chassisHalfLengthIn` /
  `chassisHalfWidthIn`，由 `ChassisGeometry.footprintPolygon` 生成。
- **通过 Hive 正下方**（`throughHive=true`）：轨迹中途存在整机外接矩形**完全落入矩形内**
  的时刻（`HiveRoutePlanner.isFootprintInsideHiveRect`）。
- **绕开 Hive**（`throughHive=false`）：中转路线整机始终位于矩形之外。

两者都**不得与 Hive 禁线接触**（`HiveRoutePlanner.isFootprintTouchingForbiddenLine`）。

**规划策略**：中转途经点航向统一为 0（面向 +x），使外接矩形轴对齐；内车道
`y=±12.5`（禁线与矩形内侧边界的中点），纵向车道 `x=±(24 + 对角半长 + 间隙)`、
外侧车道 `y=±(25 + 对角半长 + 间隙)`（对角半长 = √(半长²+半宽²)，间隙
`HypParams.hiveAvoidClearanceIn`）。

**约束边界**：起点、终点分别位于发射区域 / 取球点，可能本身就贴着矩形边缘
（发射区域与矩形相邻），因此「整机完全位于矩形之外」仅对**中转途经点及其间的车道段**
成立。车道取值与间隙需**实机标定**。

**队伍对称**：蓝队全部几何为中心对称——取球点取 `(x,y)->(-x,-y)`、矩形与禁线为红队的
中心对称、车道 y 取反、纵向车道 x 相同。

## 6. GoToUnvisitedEatPointViaHiveAction

**作用**：从机器人所在发射区域出发，**经 Hive 正下方**前往此前未到达过的吸球点。
1. 「未访问取球点」由 `utility/EatPointTracker` 共享状态决定：在未访问的取球点中选距
   机器人最近者（两个都已访问时退回取最近者）；红队用共享取球点，蓝队用其中心对称点；
2. 由 `ChassisGeometry.centerPoseFromIntakeTarget` 反推使 intake 到位、车头朝取球点的中心位姿；
3. 途经点由 `HiveRoutePlanner.waypoints(..., throughHive=true)` 生成；
4. 到达后 `onComplete()` 调用 `EatPointTracker.markVisited(pointIndex)`。

**构造**：`new GoToUnvisitedEatPointViaHiveAction(TeamColor.RED)`。
**访问记账**：由 `AutoRed` / `AutoBlue` 在自动阶段开始时调用 `EatPointTracker.clear()` 清空。

## 7. GoToOtherShootAreaViaHiveAction

**作用**：从机器人所在发射区域出发，**经 Hive 正下方**前往**另一个发射区域**。
1. 「所在发射区域」按当前位姿自动判断（在位姿内取该区域；否则取边界最近者）；
2. 终点为另一发射区域内距机器人最近的点，航向指向调用方指定的单个球门
   （与手动阶段自动瞄准一致，精确瞄准由后续 `AutoAimAction` 完成）；
3. 途经点由 `HiveRoutePlanner.waypoints(..., throughHive=true)` 生成。

**构造**：`new GoToOtherShootAreaViaHiveAction(TeamColor.RED, HypParams.RedAudienceUp)`。

## 8. GoToUnvisitedEatPointAvoidHiveAction

**作用**：从机器人所在发射区域出发，**绕开 Hive**（经非 Hive 正下方，沿矩形外侧）前往
此前未到达过的吸球点。终点选择同 `GoToUnvisitedEatPointViaHiveAction`，路径由
`HiveRoutePlanner.waypoints(..., throughHive=false)` 生成（沿外侧车道 `y=±39.7` 绕过矩形）。
**构造**：`new GoToUnvisitedEatPointAvoidHiveAction(TeamColor.RED)`。

## 9. GoToOtherShootAreaAvoidHiveAction

**作用**：从机器人所在发射区域出发，**绕开 Hive**（外侧）前往**另一个发射区域**，其余同
`GoToOtherShootAreaViaHiveAction`，仅路径模式不同。
**构造**：`new GoToOtherShootAreaAvoidHiveAction(TeamColor.RED, HypParams.RedAudienceUp)`。

## 10. AutoAimAction

**作用**：每帧读融合位姿，依据 `RobotPosition.getHiveState()` 选择球门
（`AUDIENCE_UP→Up`、`AUDIENCE_DOWN→Down`、`MIDDLE→保持上次锁定，无历史默认 Up`），
原地转向使车头对准球门，仅下发角速度、平移为 0。

**控制律**：期望航向 `des = atan2(goalY-y, goalX-x)`，误差归一化到 `[-π,π]`
后做 PD：`omega = Kp·e + Kd·de/dt`，死区内输出 0，输出限幅。

**完成条件**：航向误差与实际角速度同时小于容差并持续 `settleMs` 毫秒；
超过 `timeoutMs` 强制结束（交上层状态机处理）。

**Dashboard 可调参数**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `aimKp` | 3.0 | 航向环 P |
| `aimKd` | 0.15 | 航向环 D |
| `aimMaxOmega` | π | 角速度上限 (rad/s) |
| `headingTol` | 2° | 航向收敛容差 |
| `omegaTol` | 0.15 | 角速度收敛容差 (rad/s) |
| `settleMs` | 200 | 收敛持续时间 |
| `timeoutMs` | 3000 | 超时强制结束 |

**构造**：`new AutoAimAction(TeamColor.RED)`。

## 11. EatAction

**作用**：每帧 `sweeper.setEat()` + `sweeper.update()`（Sweeper 每帧自动归零目标，
必须每帧重新置位），持续到时后 `setStop()` 并结束。

**Dashboard 可调参数**：`eatDurationMs`（默认 2000 ms，待实机标定）；
吃球速度 `Sweeper.EatVel`（默认 700 tick/s）在 `Sweeper` 类中调整。

**构造**：
```java
Sweeper sweeper = new Sweeper(hardwareMap, telemetry);
new EatAction(sweeper);                       // 用 eatDurationMs 默认时长
new EatAction(sweeper, 1500);                 // 指定时长
```

## 12. AdaptiveParkAction（自适应停靠）

**作用**：自动阶段结束前 **3 秒**（T-3s）检测整机外接矩形是否接触场地边框，并据此决定
**本局停靠点**。停靠点由当时位姿实时解算，**不是固定点**。

**场地边界**：12 ft × 12 ft 场地以原点为中心，边界为 `|x| = |y| = 72 in`
（`HypParams.fieldHalfSizeIn`）。

**判定与动作**：

| 检测结果 | 动作 | 停靠点 |
|---------|------|--------|
| 四角任一到达/越过 ±72 in（接触） | 沿 `strafeToLinearHeading` 轨迹开到最小平移解 | 解算出的安全位姿（动态） |
| 四角全部在边界内（不接触） | `setDrivePowers(0,0,0)` 原地停止 | 检测时刻的当前位姿 |

**最小平移解**（`safeParkPose`）：设底盘半长 / 半宽为 `L` / `W`，航向 `θ`，求旋转矩形的
轴对齐半投影

```
ex = |L·cosθ| + |W·sinθ|
ey = |L·sinθ| + |W·cosθ|
```

再把中心夹到 `[-limit+ex, limit-ex]`（y 同理），其中
`limit = fieldHalfSizeIn - parkBorderMarginIn`。这样得到的**轴对齐包围盒**完全落在安全区内，
且位移最小、**不改变航向**。

**执行时序**：

1. 未到 T-3s：每帧保持电机停止并返回 `true`（可提前入队，作为队列最后一个元素）；
2. 到 T-3s：检测一次，记录 `borderContact` 与 `parkPose`；
3. 接触 → 建轨迹并跟随；不接触 → 停车并立刻返回 `false`；
4. 轨迹结束后再次停车并返回 `false`。

**构造**（与 OpMode 共用同一个比赛计时器）：

```java
ElapsedTime matchTime = new ElapsedTime();   // START 后创建

actionRunner.clear();
actionRunner.add(new AdaptiveParkAction(
        drive, matchTime,
        HypParams.AUTONOMOUS_DURATION_MS,
        HypParams.PARK_TIME_THRESHOLD_MS));
```

**参数说明**（均在 `Parameter/HypParams.java`，`@Config` 可在 Dashboard 实时调整）：

| 参数 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `AUTONOMOUS_DURATION_MS` | 30000 | ms | 自动阶段总时长，作为倒计时基准 |
| `PARK_TIME_THRESHOLD_MS` | 3000 | ms | 停靠触发阈值（剩余时间 ≤ 该值即检测） |
| `fieldHalfSizeIn` | 72.0 | in | 场地半边长（12 ft × 12 ft → ±72） |
| `parkBorderMarginIn` | 3.0 | in | 停靠时整机外接盒与边框保留的安全间隙 |
| `chassisHalfLengthIn` / `chassisHalfWidthIn` | 见 HypParams | in | 整机轮廓半长 / 半宽，用于外接矩形 |

**遥测键**（FTC Dashboard）：

| 键 | 含义 |
|----|------|
| `AdaptivePark` | 阶段文字（`waiting until T-3s` / `no border contact -> stop` / `border contact -> moving inside`） |
| `AdaptivePark remaining(ms)` | 等待阶段的剩余时间 |
| `AdaptivePark park x` / `park y` / `park heading(deg)` | 本局停靠点 |

**对外查询**：`borderContact()`、`parkPose()`。

**注意**：

- 位姿来自融合定位，本 Action **不推进定位器**（由 OpMode 每帧唯一一次 `update()`）。
- 判据依赖定位精度；若实车贴边仍有压线风险，调大 `parkBorderMarginIn`（如 4~5 in）。
- 极端情况下（外接盒大于安全区）退回场地中心 `(0,0)`，航向不变。

## 13. AutoCycleAction（保守版：周期组合）

**作用**：把自动阶段的一个完整周期合成为**一个** Action 入队：

```
GoToUnvisitedEatPointAvoidHiveAction → EatAction
  → GoToOtherShootAreaViaHiveAction →（可选）AutoAimAction
```

取球点仍是终点，车会**停稳后**再吃球——与现主程序状态机语义一致，只是把「逐段投递」变成「一次投递」。

**相对逐段入队的收益**：

| 问题 | 逐段入队 | 组合后 |
|------|----------|--------|
| 段间空档 | `ActionRunner` **每帧最多启动一个 Action**，交接处有 1 帧（约 20 ms）空档，该帧没有任何代码给底盘下发电机指令 | `SequentialAction` 是 `tailrec`，上一子 Action 返回 false 的**同一帧**接下一段 |
| 队列状态抖动 | `isBusy()` 在段间短暂为 false，`Chassis` 可能误判「队列空闲」而放行手柄或主动刹停 | 整个周期 `isBusy()` 恒为 true |
| 旧位姿规划 | 每段在轮到自己时才 new，位姿正确 | 若把子 Action 提前 new，则会用**上一段开始前**的旧位姿规划；用 `DeferredAction` 保证仍是执行时构造 |

**实现要点**：

```java
List<Action> steps = new ArrayList<>();
steps.add(new DeferredAction(() -> new GoToUnvisitedEatPointAvoidHiveAction(teamColor)));
steps.add(new EatAction(sweeper));
steps.add(new DeferredAction(() -> new GoToOtherShootAreaViaHiveAction(teamColor, goal)));
if (includeAim) steps.add(new AutoAimAction(teamColor));
return new SequentialAction(steps);
```

**Dashboard 可调参数**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `includeAim` | `true` | 是否在周期末尾追加 `AutoAimAction` |

**构造 / 接入**：

```java
actionRunner.add(new AutoCycleAction(teamColor, sweeper, HypParams.BlueAudienceUp));
```

**注意**：本 Action 只能消除调度空档，**不能**消除轨迹之间的减速到零（吃球段与移动段仍是独立轨迹）。
需要全程不停车请用第 14 节。

**状态**：尚未接入 `AutoRed` / `AutoBlue`，主程序仍按原状态机逐段投递。

## 14. ContinuousRouteAction（激进版：单条轨迹不停车）

**作用**：把「前往取球点 + 路过式吃球 + 前往另一发射区域」合成一个 Action，
且整条路线**只有一份 `TrajectoryActionBuilder` 链**：途经点按速度/加速度约束通过，只在最末端停车。

**为什么必须是一条 builder 链**：

`MecanumDrive.actionBuilder` 传入 `beginEndVel = 0.0`，而 Road Runner 的 `profile()` 用同一个值
同时作为**起点与终点速度**——因此每一份 `build()` 出来的轨迹必然以 `v = 0` 结束。
`SequentialAction` / `ParallelAction` 都改变不了这一点；只有把全部途经点串进同一条链，
才会共用一条时间最优 profile。

**路线构成**：

```
start → leg1 途经点（绕开 Hive） → 触发点 → 取球点（作为途经点） → leg2 途经点（经 Hive 正下方） → 发射区
```

- 取球点作为**途经点**而非终点，因此经过时不停车；
- leg1 / leg2 途经点分别由 `HiveRoutePlanner.waypoints(..., throughHive=false / true)` 生成，
  几何约束同第 6~9 节；
- 到达后自动 `EatPointTracker.markVisited(eatIndex)`。

**吃球如何与移动并行**：

在触发点（到达取球点前 `eatTriggerDistanceIn` 英寸）用 `builder.afterDisp(0.0, eatSeq)` 注册标记。
Road Runner 构建时会把它包成：

```
ParallelAction(轨迹 Action, SequentialAction(SleepAction(触发时间), eatSeq))
```

即吃球与轨迹跟随**并行执行**，轨迹不等待吃球。`eatSeq` 直接复用 `EatAction`
（每帧 `setEat()` + `update()`；因为 `Sweeper.update()` 每帧会把目标速度归零，
一次性 `InstantAction` 无效）。

**三个必须知道的代价**：

| 现象 | 原因 | 规避 |
|------|------|------|
| 车已停在发射区，Action 仍未结束 | `ParallelAction` 需**两个子项都结束**；吃球时长 > 剩余轨迹时间 | `eatDurationMs` 设为约等于「触发点到路线结束」的行驶时间 |
| 吃球可能提前/滞后 | `afterDisp` 实际是 `SleepAction(profile.inverse(disp))` 的**墙钟计时**，按**规划时间**触发而非实际位移；打滑、被顶、电压低时偏差明显 | 需精确时改为「每帧读实际位姿，距触发点 ≤ 容差才启动」的自定义 Action |
| 行驶中吃球 | 前提是集球器能在行驶中把球吃进去 | 必须静止对准才能进球时，请用第 13 节保守版 |

**Dashboard 可调参数**：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `eatTriggerDistanceIn` | 12.0 | 到达取球点前多少英寸开始吃球（沿最后接近方向回退） |
| `eatDurationMs` | 1200 | 吃球时长，应≈触发点到路线结束的行驶时间 |
| `secondLegThroughHive` | `true` | 第二段是否经 Hive 正下方（`false` 为绕外侧） |
| `includeAim` | `true` | 路线末尾是否追加 `AutoAimAction` |

**构造 / 接入**：

```java
actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, HypParams.BlueAudienceUp));
```

**状态**：尚未接入 `AutoRed` / `AutoBlue`。

## 典型串联顺序

```
GoToShootAreaAction → AutoAimAction →（发射）
GoToEatPointAction / GoToNearestEatPointAction → EatAction
GoToOtherShootAreaViaHiveAction / GoToOtherShootAreaAvoidHiveAction → AutoAimAction →（发射）
GoToUnvisitedEatPointViaHiveAction / GoToUnvisitedEatPointAvoidHiveAction → EatAction
剩余时间 ≤ PARK_TIME_THRESHOLD_MS → ActionRunner.clear() + AdaptiveParkAction
```

**整周期组合（可选，见 13 / 14 节）**：

```java
actionRunner.add(new AutoCycleAction(teamColor, sweeper, goal));        // 保守版：取球点停车吃球
actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, goal));  // 激进版：单条轨迹不停车
```

两者都只需在 `actionRunner` 空闲时投递即可，后续周期重复投递。

## 自动阶段主程序（AutoRed / AutoBlue）

两个 OpMode 使用同一套状态机，仅队伍颜色、起始位姿与预瞄准球门不同：

| 项目 | AutoRed | AutoBlue |
|------|---------|----------|
| 队伍 | `TeamColor.RED` | `TeamColor.BLUE` |
| 起始位姿 | `HypParams.startPoseRed` | `HypParams.startPoseBlue` |
| 预瞄准球门 | `HypParams.RedAudienceUp` | `HypParams.BlueAudienceUp` |

**状态机与动作循环**（动作队列空闲时投递下一段）：

```
TO_EAT_POINT   → GoToUnvisitedEatPointAvoidHiveAction（绕开 Hive 去未访问取球点）
EATING         → EatAction
TO_SHOOT_AREA  → GoToOtherShootAreaViaHiveAction（经 Hive 正下方去另一发射区）
TO_SHOOT_AREA 完成 → 回到 TO_EAT_POINT，持续循环
PARKING        → AdaptiveParkAction
STOP
```

**停靠的硬保障**：主循环每帧计算 `remainingMs`；一旦
`remainingMs <= PARK_TIME_THRESHOLD_MS` 且尚未投递过停靠动作，立即
`actionRunner.clear()` 中断**正在执行**的任意 Action，再投递 `AdaptiveParkAction`，
并用 `parkScheduled` 保证只投递一次。因此即使上一段长轨迹尚未跑完，T-3s 也一定会转入停靠。

**结束收尾**：`RobotStateStore.savePose(...)` 记录结束位姿，`sweeper.setStop()` +
`chassis.stop()` 确保底盘与集球器停稳。

## 参数配置：AutoParams（盟友活动区 / 我方禁止活动区）

**文件**：`Parameter/AutoParams.java`（`@Config`，与 `HypParams` 同级）

**作用**：登记 3～5 名队友在「我方区域」某段时间内占据的场地范围，
作为**我方在该时段内的禁止活动区域**，避免自动阶段抢位 / 相撞。

**关键语义**：每个禁区 = **一块凸多边形区域 + 一个时间窗**；
时间窗之外该区域**不构成限制**，因此同一块场地在不同时段可被不同队友占用。

### 全局参数

| 参数 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `useAllyZones` | `true` | — | 总开关；`false` 时所有查询恒返回「不受限」 |
| `allyZoneClearanceIn` | `2.0` | in | 安全外扩量：整机轮廓距禁区边界小于该值即判闯入，用于吸收定位误差 |
| `allyZoneEscapeMarginIn` | `1.0` | in | 脱困余量：脱困目标点停在「禁区边界 + clearance + 本值」之外，避免贴边反复触发 |

### 区域与时间窗

所有区域按**红队视角**以场地坐标（英寸）填写，蓝队由原点中心对称自动生成：

| 字段 | 区域名称 | 范围（红队） | 时间窗 |
|------|----------|--------------|--------|
| `ALLY_ZONE_1` | `AllyA-StartCorner` | `x∈[-70,-40]`、`y∈[40,60]` | 0 – 6000 ms |
| `ALLY_ZONE_2` | `AllyB-MidLeft` | `x∈[-20,10]`、`y∈[30,50]` | 3000 – 12000 ms |
| `ALLY_ZONE_3` | `AllyC-RightStart` | `x∈[25,55]`、`y∈[35,55]` | 0 – 9000 ms |
| `ALLY_ZONE_4` | `AllyD-NearHive` | `x∈[-15,15]`、`y∈[8,28]` | 8000 – 20000 ms |
| `ALLY_ZONE_5` | `AllyE-RightLow` | `x∈[40,68]`、`y∈[5,25]` | 12000 – 30000 ms |
| `ALLY_ZONES_RED` | — | 上述 5 个 | 各自时间窗 |
| `ALLY_ZONES_BLUE` | — | 上述 5 个的原点中心对称 | 各自时间窗（不变） |

> 以上坐标与时间窗均为**占位默认值**，必须按队友实际跑位与战术安排修改。
> 需要 3 个或 4 个队友时，直接删减 `ALLY_ZONE_*` 并把 `ALLY_ZONES_RED` 的 `Arrays.asList(...)`
> 改成对应数量即可。

**红蓝镜像**：`ALLY_ZONES_BLUE` 由 `area.inRelative(0, 0, Math.PI)` 生成，
即 `(x, y) → (-x, -y)`，与 `HypParams` 中发射区域、Hive 矩形的红蓝约定一致。

**时间基准**：`startMs` / `endMs` 为**相对自动阶段 START 的毫秒数**，与 OpMode 中的
`ElapsedTime matchTime` 同一基准；自动阶段总时长见 `HypParams.AUTONOMOUS_DURATION_MS`。

### 查询接口

| 方法 | 返回 | 用途 |
|------|------|------|
| `zonesFor(teamColor)` | `List<AllyZone>` | 全量禁区（不按时间过滤） |
| `activeZones(teamColor, matchMs)` | `List<AllyZone>` | 该时刻**生效**的禁区 |
| `activeZoneNames(teamColor, matchMs)` | `List<String>` | 生效禁区名称（遥测 / 日志） |
| `isForbidden(teamColor, pose, matchMs)` | `boolean` | 整机是否闯入任一生效禁区 |
| `firstConflictingZone(teamColor, pose, matchMs)` | `AllyZone` / `null` | 冲突来源，供决定绕行方向 |
| `nearestSafePose(teamColor, pose, matchMs)` | `Pose2d` | 脱离禁区的**最小位移**安全位姿（含安全余量、自动夹在场地内） |

`AllyZone` 自身提供：`activeAt(matchMs)`、`contains(x, y)`、`distanceTo(x, y)`、`mirrored()`。

**判定精度**：`isForbidden` 取 `ChassisGeometry.footprintCorners(pose)` 的**整机外接矩形四个角点**
逐点判定，任一角点到禁区距离 ≤ `allyZoneClearanceIn` 即判闯入——
判定的是**整机轮廓**而非单纯中心点。

### 已内置到自动 OpMode 的守卫（无需额外调用）

`AutoRed` / `AutoBlue` 主循环**每帧**执行下列守卫，因此禁区是**自动生效**的：

```java
long matchMs = (long) matchTime.milliseconds();

// 停靠阶段优先，不再检查禁区
Pose2d pose = RobotPosition.getInstance().getPose2d();
AutoParams.AllyZone conflict = parkScheduled || !AutoParams.useAllyZones
        ? null
        : AutoParams.firstConflictingZone(teamColor, pose, matchMs);

if (conflict != null && !zoneEscapeActive) {
    zoneEscapeActive = true;
    actionRunner.clear();                                       // 中断当前动作
    actionRunner.add(new GoToStopPose(                          // 改投脱困动作
            drive, AutoParams.nearestSafePose(teamColor, pose, matchMs)));
} else if (conflict == null) {
    zoneEscapeActive = false;                                   // 已脱离，恢复常规循环
}

// 脱困期间暂停常规动作投递
if (!actionRunner.isBusy() && !zoneEscapeActive) { /* 状态机投递 */ }
```

行为要点：

1. **闯入即中断**：`actionRunner.clear()` 会打断正在执行的轨迹 Action，不等其跑完；
2. **脱困目标动态求解**：`nearestSafePose()` 以禁区包围盒为界，把底盘中心推到最近盒外位置
   （航向不变、位移最小），并额外留出 `allyZoneClearanceIn + allyZoneEscapeMarginIn`，
   结果还要夹在场地内，不会为了躲禁区而开出边框；
3. **只触发一次**：`zoneEscapeActive` 置位后不再重复投递；
4. **自动恢复**：脱困后 `conflict == null`，标志清零，状态机从原状态继续投递动作；
5. **与停靠互斥**：`parkScheduled == true` 后守卫停用，停靠逻辑独占控制权。

遥测新增三行：`Active Zones`（生效禁区名）、`In Forbidden`（是否闯入）、`Escaping`（是否在脱困）。

### 其他用法示例

**1）遥测显示当前生效的禁区（Dashboard / Driver Station）：**

```java
long matchMs = (long) matchTime.milliseconds();
telemetry.addData("Active Zones", AutoParams.activeZoneNames(teamColor, matchMs));
telemetry.addData("In Forbidden", AutoParams.isForbidden(
        teamColor, RobotPosition.getInstance().getPose2d(), matchMs));
```

**2）规划前检查目标点是否落在禁区内（提前绕行，而不是闯进去再脱困）：**

```java
AutoParams.AllyZone conflict = AutoParams.firstConflictingZone(teamColor, targetPose, matchMs);
if (conflict != null) {
    // targetPose 在该时段属于队友活动范围，需要改点或等待时间窗结束
    telemetry.addData("Blocked by", conflict.name + " " + conflict);
}
```

**3）临时关闭约束做对比试验（Dashboard Config 页）：**

```java
AutoParams.useAllyZones = false;   // 等价于在 Dashboard 取消勾选
```

### 修改注意

- 区域与时间窗在**类加载时**构建为不可变列表，修改后需**重新运行 OpMode** 生效；
  `useAllyZones`、`allyZoneClearanceIn`、`allyZoneEscapeMarginIn` 三个标量可在 Dashboard Config 页实时调整。
- 禁区判定依赖定位精度，实测若出现「压线」，优先调大 `allyZoneClearanceIn`（3~4 in）。
- 时间窗写反（`startMs > endMs`）时构造器会自动交换，不会抛异常。

## 实机测试

每个 Action 都有对应的实机测试 OpMode，位于
`Auto/AutoOpModes/`（Driver Station 中组名 **ActionTest**）：

| OpMode | 被测 Action |
|--------|------------|
| `Test: GoToStopPose` | `GoToStopPose` |
| `Test: GoToShootArea` | `GoToShootAreaAction` |
| `Test: GoToShootAreaVertices` | `GoToShootAreaNearestVerticesAction` |
| `Test: GoToEatPoint` | `GoToEatPointAction` |
| `Test: GoToNearestEatPoint` | `GoToNearestEatPointAction` |
| `Test: GoToUnvisitedEatPointViaHive` | `GoToUnvisitedEatPointViaHiveAction` |
| `Test: GoToOtherShootAreaViaHive` | `GoToOtherShootAreaViaHiveAction` |
| `Test: GoToUnvisitedEatPointAvoidHive` | `GoToUnvisitedEatPointAvoidHiveAction` |
| `Test: GoToOtherShootAreaAvoidHive` | `GoToOtherShootAreaAvoidHiveAction` |
| `Test: AutoAim` | `AutoAimAction` |
| `Test: Eat` | `EatAction` |

> `AdaptiveParkAction` 没有独立测试 OpMode：它的触发依赖比赛计时与自动阶段完整流程，
> 请通过 `AutoRed` / `AutoBlue` 实机联调验证（重点观察 T-3s 是否停车、接触时是否向场内移动、
> Dashboard 的 `AdaptivePark park x/y` 是否为场内位置）。
>
> `AutoCycleAction` / `ContinuousRouteAction` 同样暂无独立测试 OpMode：它们由上表 Action 组合而成，
> 建议先逐个验证被组合的 Action，再在 `AutoBlue` 中接入联调；也可参照 `ActionTestBase`
> 新增一个只运行单个组合 Action 的测试 OpMode。

**Init 阶段按键选择**：

- `A` / `B`：红队 / 蓝队（决定起始位姿、Limelight pipeline、球门颜色）；
- 十字键上 / 下：切换 `FIRST` / `SECOND`（仅发射区域、取球点测试显示）；
- `X`：切换瞄准球门 AudienceUp / AudienceDown（仅发射区域类测试显示）。

按下 START 后自动构造并执行该 Action，完成后 OpMode 自动结束；
Dashboard 与 Driver Station 遥测区可观察目标位姿、误差、Action 状态。
运行前请将机器人放到与所选起始位姿一致的位置和朝向上。
