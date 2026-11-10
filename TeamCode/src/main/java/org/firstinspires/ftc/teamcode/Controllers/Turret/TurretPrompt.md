完成Turret包负责控制炮台，要求如下：
炮台的结构：由一个大球发射器和一个小球发射器，和共同的平转机构组成，每个发射器含一个飞轮、一个由舵机控制的仰角机构（代码中的 `pitchServo`，文档里称"俯仰"）和一个由舵机控制的扳机，两发射器的朝向始终平行，由一个电机控制。
各文件功能：
- Turret.java：负责炮台整体控制，提供一个update函数，每帧调用设置状态，参数如下：
  - yaw: 目标平转角度
  - pitchBig: 大球俯仰机构目标舵机位置（不一定是真实发射角度）
  - pitchSmall: 小球俯仰机构目标舵机位置（不一定是真实发射角度）
  - speedBig: 大球目标发射器速度
  - speedSmall: 小球目标发射器速度
  - triggerBig: 大球扳机是否释放
  - triggerSmall: 小球扳机是否释放
  返回两个bool变量，表示是否能够发射大球和小球
  超参数：
  - RelPosBig: 大球发射器中心在地面投影在相对坐标系的坐标，单位inch
  - RelPosSmall: 小球发射器中心在地面投影在相对坐标系的坐标，单位inch
  - fireHoldTime: 自动发射时扳机的最短连续释放时间（秒），吸收就绪状态的逐帧抖动；取消发射、平转超限、强制释放松手时立即关闭
  另提供两个重载（三个 update 语义不同，不要混用）：
  - `update(allowFire, preload, forceReleaseBig, forceReleaseSmall)`：自动瞄准，位姿与球门由 `RobotPosition` 提供（球门来自 `getGoalPose()`，优先级：本帧 MT1 非 MIDDLE 观测 > TeleOp GP2.LT/RT 手动设置 > 上一帧保持），行为见 AutoAimPrompt.md
  - `updateManual(yawRate, pitchBig, pitchSmall, speedBig, speedSmall, triggerBig, triggerSmall)`：手动瞄准，输入是**平转角速度**（弧度/秒，逆时针为正），由 `Turret` 积分成被维持的角度，再按 RELATIVE / ABSOLUTE / GOAL 三种模式映射成车体相对 yaw
- YawController.java：负责平转机构的控制，提供一个update函数，每帧调用设置目标平转角度，钳制到预设范围内，计算电机目标位置，使用PIDSVA控制电机达到目标位置
    超参数：
  - yawRange: 平转机构最大角度绝对值，单位为弧度
  - tickPerRad: 每个弧度对应的电机tick变化量（tick→yaw 的映射，含符号），只描述读数换算，不能替代方向开关
  - motorReversed: yaw 电机方向开关，置 true 时保证"正功率 → tick 增加"；这是位置环稳定的前提（闭环特征值为 d(tick)/d(power) 的符号，与 tickPerRad 无关）
  - onTargetTolerance / onTargetRateTolerance / onTargetFrames: 到位判据（位置容差、角速度容差、连续满足帧数），用于判断是否允许发射
  - yawSeamHysteresis: 折返抑制窗口（弧度），±π 两种表示的最短角差不超过该值时保持旧目标，避免正后方附近整圈反转；保持期间的真实偏差参与到位判据（不超过 onTargetTolerance 才允许发射）
  - yaw电机的PIDSVA参数
- Shooter.java：负责单个发射器的控制，提供一个 update 函数，每帧调用设置目标速度、舵机位置和扳机状态，钳制到可用范围后，**飞轮用 `utility/Shooter/FlywheelCalculator`（第 4 代控制律，已不再使用 PIDSVA）**做速度闭环，俯仰与扳机舵机直接下发位置。
    功能：
  - `update(speed, pitch, release)`：下发目标并返回 `isReady()`（飞轮稳态 + 俯仰动作时间）
  - `solve(robotPose, goalPose, relPos)`：自动瞄准解算，返回 `ShotSolution{yaw, range, pitch, speed}`
  - `setTrigger(release)`：只写扳机，不重算速度环
  - `getMinPitch() / getMaxPitch()`：档位表的最低 / 最高仰角（预载的两个安全姿态）
    超参数（`Params`，大球 / 小球各一份实例：`BIG_PARAMS` / `SMALL_PARAMS`）：
  - `flywheel`: `FlywheelParams`（飞轮速度环全部参数，`BIG` / `SMALL` 两个静态实例），见 `utility/Shooter/`
  - `pitchMin` / `pitchMax`: 俯仰舵机可用位置范围
  - `pitchSettleTime`: 俯仰目标变更后视为到位的近似时间（秒）
  - `pitchModels`: 离散仰角档位表（`pitchPos` / `minRange` / `maxRange` / `speedSlope` / `speedIntercept`），**必须按 `pitchPos` 升序**
  - `pitchSwitchMargin`: 档位切换滞回裕量（英寸）
  - `preSpeedMax`: 预载最大转速
  - `speedDeadband`: 转速目标死区 (tick/s)
  - `triggerOnPos` / `triggerOffPos`: 扳机释放 / 关闭时目标舵机位置
  - `speedMax`: 最大转速（实际来自 `flywheel.speedMax`）
