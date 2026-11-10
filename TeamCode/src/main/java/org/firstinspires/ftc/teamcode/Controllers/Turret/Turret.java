package org.firstinspires.ftc.teamcode.Controllers.Turret;

import static org.firstinspires.ftc.teamcode.utility.Time.seconds;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.AimGeometry;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 炮台整体控制器。
 *
 * <p><b>机械结构</b>：一个大球发射器、一个小球发射器，以及二者共用的平转机构。
 * 每个发射器含一个飞轮、一个俯仰舵机和一个扳机舵机；两发射器朝向始终平行，
 * 由同一个平转电机驱动，因此共享同一个平转角 yaw。
 *
 * <p><b>职责划分</b>：
 * <ul>
 *   <li>{@link YawController} —— 平转机构的位置闭环</li>
 *   <li>{@link Shooter} —— 单个发射器的飞轮速度环、俯仰与扳机</li>
 *   <li>本类 —— 把上层下达的目标分发给上述两个部件，并汇总到位状态</li>
 * </ul>
 *
 * <p>按 {@code FTC_Format.md} 的约定，{@link #update} 会各调用一次其直接依赖的
 * 部件的 update 函数；上层每帧只需调用本类的 update。
 *
 * <p><b>自动瞄准</b>：重载 {@link #update(boolean, boolean, boolean, boolean)}，车辆与球门位置
 * 从 {@link RobotPosition} 读取；两个发射器各自反解平转角与射程并按标定的射程-转速档位表
 * 解出飞轮速度，平转角取两方向的劣角平分线。允许发射时按解算值驱动机构，机构与平转
 * 均到位即自动释放对应扳机（并至少连续释放 {@link #fireHoldTime} 秒，吸收就绪状态的逐帧抖动）；
 * 不允许发射时按预载规则驱动机构。
 */
@Config
public class Turret {

    // ==================== 超参数 ====================

    /**
     * 大球发射器旋转中心在地面的投影，在相对坐标系中的坐标（英寸）。
     * 相对坐标系（RoadRunner）：x 指向车头，y 指向车体左侧。
     * 自动瞄准解算发射器实际位置时使用。todo: 实机标定。
     */
    public static Vector2D RelPosBig = new Vector2D(0, 0);

    /**
     * 小球发射器旋转中心在地面的投影，在相对坐标系中的坐标（英寸）。
     * 自动瞄准解算发射器实际位置时使用。todo: 实机标定。
     */
    public static Vector2D RelPosSmall = new Vector2D(0, 0);

    /**
     * 自动发射时扳机的最短连续释放时间 (秒)。
     *
     * <p>飞轮到位判据见 {@link Shooter#isReady()}，其底层稳态计数在任何目标转速变化时清零；
     * 自动瞄准下转速随射程逐帧重算，到位状态会在边界附近逐帧抖动。若每帧重新决定扳机，
     * 扳机会随之逐帧开合，装填被打断。取不小于舵机单次动作时间（约 0.15 s）即可
     * 把抖动吸收成一次连续释放；条件连续失效超过该时长后才关闭。
     *
     * <p><b>保持窗口只吸收"转速/俯仰就绪"的抖动，不吸收指向失效</b>：窗口内仍要求
     * {@code autoYawFeasible} 与平转到位（{@link YawController#isOnTarget()}）。
     * 否则球门切换（HIVE 状态翻转 → 目标跳变近 180°）或底盘持续转动时，
     * 只要出现一帧就绪，扳机就会被连续打开整个保持时长
     * （0.15 s 在 120°/s 的平转速下相当于约 18° 的偏角），带着偏角出球。
     *
     * <p>取消允许发射、平转超限（指向已不可达）、指向失效与强制释放松手都立即关闭，不进入保持窗口。
     */
    public static double fireHoldTime = 0.15;

    /**
     * 两个发射器方位角的允许最大夹角（弧度）。
     *
     * <p>超过该值说明球门几乎位于两发射器连线的延长线上（约 180° 方位差）：
     * 此时劣角平分线用的单位向量和趋近于零，{@code atan2(0,0)} 返回 0，
     * 平分结果不可信 —— 炮口会被推到车头方向，却照常判到位并自动扣扳机。
     * 因此超过该夹角直接判 {@code autoYawFeasible = false}（保持转速与仰角、禁止自动发射），
     * 与平转超限走同一条安全通路。
     */
    public static double maxShooterYawSpread = Math.PI / 2;

    // ==================== 器件名 ====================

    private static final String BIG_MOTOR = "shooterBig";
    private static final String BIG_PITCH_SERVO = "pitchBig";
    private static final String BIG_TRIGGER_SERVO = "triggerBig";

    private static final String SMALL_MOTOR = "shooterSmall";
    private static final String SMALL_PITCH_SERVO = "pitchSmall";
    private static final String SMALL_TRIGGER_SERVO = "triggerSmall";

    // ==================== 部件 ====================

    public final YawController yawController;
    public final Shooter shooterBig;
    public final Shooter shooterSmall;

    private final Telemetry telemetry;

    // ==================== 自动瞄准运行时状态（供遥测） ====================

    /** 炮台当前由哪条路径驱动（决定遥测分段，且互斥） */
    private AimSource aimSource = AimSource.NONE;

    /** 瞄准来源：同一时刻只有一条路径在驱动炮台 */
    private enum AimSource { NONE, AUTO, MANUAL }

    /** 最近一次解算的平转目标（劣角平分线，已折叠到 (-π, π]，弧度，未钳制） */
    private double autoYawTarget = 0;
    /** 最近一次解算的平转目标是否在 ±{@link YawController#yawRange} 内 */
    private boolean autoYawFeasible = true;
    /** 最近一次两发射器方位角的夹角（弧度，折算到 [0, π]），供遥测与退化判定 */
    private double autoYawSpread = 0;
    /** 最近一次预载是否开启 */
    private boolean autoPreloadEnabled = true;
    /** 最近一次车辆中心是否位于不可发射区 */
    private boolean autoInNoShootArea = false;
    /** 最近一次两个发射器的解算结果；RobotPosition 不可用时为 null */
    private Shooter.ShotSolution autoBigSolution;
    private Shooter.ShotSolution autoSmallSolution;
    /** 最近一帧实际释放的扳机（大球/小球各一个状态机，遥测字段名不变） */
    private final TriggerGate bigGate = new TriggerGate();
    private final TriggerGate smallGate = new TriggerGate();

    // ==================== 手动平转模式（供遥测） ====================

    /** 手动平转模式：决定手动摇杆速率积分所维持的量 */
    public enum YawMode {
        /** 维持相对朝向：锁定车体坐标系下的平转角（等价于旧手动 update 的常量目标） */
        RELATIVE,
        /** 维持绝对朝向：锁定场地坐标系方向，车体旋转时平转自动反向补偿 */
        ABSOLUTE,
        /** 维持与球门夹角：锁定相对球门方位的偏差；偏差为 0 时与自动瞄准同向 */
        GOAL
    }

    /** 当前手动平转模式 */
    private YawMode manualYawMode = YawMode.RELATIVE;
    /** 当前模式被维持的角度（弧度）：REL=车体相对 yaw（饱和在 ±yawRange）；ABS=场地绝对朝向；GOAL=相对球门方位偏差 */
    private double manualAngle = 0;
    /** 手动模式是否已用实测角初始化状态 */
    private boolean manualInitialized = false;
    /** 上一次 updateManual 的调用时刻 (秒)，用于积分 dt */
    private double lastManualTime = 0;
    /** 最近一次 updateManual 算出的车体相对目标（已钳制到 ±yawRange，供遥测） */
    private double manualYawTarget = 0;
    /** 最近一次可解的球门方位（车体相对，弧度）：GOAL 模式方位退化时保持此值；NaN = 尚无有效值 */
    private double lastGoalBodyYaw = Double.NaN;
    /** 上一帧的大球 / 小球扳机释放状态，用于检测"发射完成"（true → false 下降沿） */
    private boolean lastTriggerBig = false;
    private boolean lastTriggerSmall = false;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.yawController = new YawController(hardwareMap, telemetry);
        this.shooterBig = new Shooter(hardwareMap, telemetry,
                BIG_MOTOR, BIG_PITCH_SERVO, BIG_TRIGGER_SERVO, Shooter.BIG_PARAMS);
        this.shooterSmall = new Shooter(hardwareMap, telemetry,
                SMALL_MOTOR, SMALL_PITCH_SERVO, SMALL_TRIGGER_SERVO, Shooter.SMALL_PARAMS);
        stop();
    }

    // ==================== update ====================

    /**
     * {@link #update} 的返回值：两个发射器当前是否具备发射条件。
     */
    public static class FireReady {
        /** 大球是否可发射 */
        public final boolean big;
        /** 小球是否可发射 */
        public final boolean small;

        public FireReady(boolean big, boolean small) {
            this.big = big;
            this.small = small;
        }
    }

    /**
     * 每帧调用一次：设置炮台目标状态。
     *
     * @param yaw           目标平转角度（弧度，0 表示朝车头，超出范围会被钳制）
     * @param pitchBig      大球俯仰机构目标舵机位置（不一定是真实发射角度）
     * @param pitchSmall    小球俯仰机构目标舵机位置（不一定是真实发射角度）
     * @param speedBig      大球目标飞轮转速 (tick/s)
     * @param speedSmall    小球目标飞轮转速 (tick/s)
     * @param triggerBig    大球扳机是否释放（true = 释放/发射）
     * @param triggerSmall  小球扳机是否释放（true = 释放/发射）
     * @return 两个发射器是否能够发射——各自要求飞轮转速与俯仰机构到位，
     *         且平转机构已到达目标角度（朝向不对时即便机构就绪也不能发射）
     */
    public FireReady update(double yaw, double pitchBig, double pitchSmall,
                            int speedBig, int speedSmall,
                            boolean triggerBig, boolean triggerSmall) {
        aimSource = AimSource.NONE;
        clearAutoFireState();

        boolean yawReady = yawController.update(yaw);
        boolean bigReady = shooterBig.update(speedBig, pitchBig, triggerBig);
        boolean smallReady = shooterSmall.update(speedSmall, pitchSmall, triggerSmall);

        return new FireReady(yawReady && bigReady, yawReady && smallReady);
    }

    /**
     * 自动瞄准：每帧调用一次。车辆位姿与球门位置从 {@link RobotPosition} 读取，
     * 两发射器各自解算平转角/射程/仰角/转速，平转角取两方向的劣角平分线。
     *
     * <p><b>目标分配</b>（每个发射器独立）：
     * <ul>
     *   <li>{@code forceReleaseX} → 采用解算速度与仰角，并强制释放扳机；</li>
     *   <li>平转超限 → 保持上一帧速度与仰角，期间不自动释放扳机；</li>
     *   <li>{@code allowFire} → 采用解算速度与仰角，机构与平转均到位即释放扳机，
 *       并至少连续释放 {@link #fireHoldTime} 秒（吸收就绪状态的逐帧抖动）；</li>
     *   <li>否则 → 预载：预载关闭时速度 0、仰角取最高档；位于不可发射区（
     *       {@link HypParams#NoShootArea}）时速度 0、仰角取最低档；其余情况<b>一律采用解算档位</b>，
     *       只把转速限制到 {@link Shooter.Params#preSpeedMax}。预载与发射必须共用同一个俯仰姿态，
     *       否则每次扣扳机都要重等 {@link Shooter.Params#pitchSettleTime}，且舵机在按下/松开间来回摆动。</li>
     * </ul>
     *
     * @param allowFire        是否允许发射（否则按预载规则驱动机构）
     * @param preload          是否启用预载（false 时预载速度恒为 0、仰角取最高档）
     * @param forceReleaseBig  是否强制释放大球扳机（无视就绪与平转超限）
     * @param forceReleaseSmall 是否强制释放小球扳机（无视就绪与平转超限）
     * @return 两发射器当前是否具备发射条件（平转到位 + 机构就绪），与 {@code allowFire}
     *         及强制释放无关；本帧是否实际扣扳机见遥测 Auto big/small fired
     */
    public FireReady update(boolean allowFire, boolean preload,
                            boolean forceReleaseBig, boolean forceReleaseSmall) {
        aimSource = AimSource.AUTO;
        autoPreloadEnabled = preload;
        // 自动模式接管后手动状态作废：下次进入手动时用实测角重新初始化，
        // 避免沿用上一次手动会话的 manualAngle 造成目标一帧内大角度突跳
        manualInitialized = false;

        RobotPosition robotPosition = RobotPosition.getInstanceOrNull();
        Pose2d pose = (robotPosition != null) ? robotPosition.getPose2d() : null;
        Pose2d goal = (robotPosition != null) ? robotPosition.getGoalPose() : null;
        if (!isFinitePose(pose) || !isFinitePose(goal)) {
            // 无有效定位/球门：不瞄准也不发射，按"预载关闭"姿态安全停转
            autoBigSolution = null;
            autoSmallSolution = null;
            autoYawTarget = yawController.getTargetYaw();
            autoYawFeasible = false;
            autoYawSpread = 0;
            autoInNoShootArea = false;
            bigGate.clear();
            smallGate.clear();

            yawController.update(autoYawTarget);
            shooterBig.update(0, shooterBig.getMaxPitch(), false);
            shooterSmall.update(0, shooterSmall.getMaxPitch(), false);
            return new FireReady(false, false);
        }

        Shooter.ShotSolution bigSol = shooterBig.solve(pose, goal, RelPosBig);
        Shooter.ShotSolution smallSol = shooterSmall.solve(pose, goal, RelPosSmall);
        autoBigSolution = bigSol;
        autoSmallSolution = smallSol;

        // ---- 平转角：两方向的劣角平分线 ----
        // 硬限位 ±yawRange 内任一指向的唯一可达表示 = 折叠到 (-π, π]；不叠加 2π 分支（该分支越界）。
        // 正后方 ±π 两表示的整圈反转由 YawController.yawSeamHysteresis 抑制，抑制残差计入到位判据。
        // 退化点：两方位接近相反时和向量趋零、atan2(0,0)=0；夹角超过 maxShooterYawSpread 判不可行
        // （走与平转超限相同的安全通路：保持转速与仰角、禁止自动发射），且不更新目标以免突跳。
        autoYawSpread = Math.abs(Vector2D.normalizeAngle(bigSol.yaw - smallSol.yaw));
        autoYawTarget = bisectorYaw(bigSol.yaw, smallSol.yaw);
        if (!Double.isFinite(autoYawTarget)) {
            autoYawTarget = yawController.getTargetYaw();   // 退化：保持上一帧目标
            autoYawFeasible = false;
        } else {
            autoYawFeasible = Math.abs(autoYawTarget) <= YawController.yawRange;
        }
        boolean yawReady = yawController.update(autoYawTarget);

        autoInNoShootArea = HypParams.NoShootArea != null
                && HypParams.NoShootArea.Contains(pose.position.x, pose.position.y);

        // ---- 各发射器独立决定本帧目标（分支顺序：强制释放 > 平转不可行 > 发射 > 预载）----
        Target bigTarget = resolveTarget(shooterBig, bigSol, forceReleaseBig, allowFire, preload);
        Target smallTarget = resolveTarget(shooterSmall, smallSol, forceReleaseSmall, allowFire, preload);

        // 先只下发机构目标，拿到本帧就绪结果；扳机在本帧末尾按"最终值"统一写一次
        boolean bigReady = shooterBig.updateMechanisms(bigTarget.speed, bigTarget.pitch);
        boolean smallReady = shooterSmall.updateMechanisms(smallTarget.speed, smallTarget.pitch);

        boolean bigCanFire = autoYawFeasible && yawReady && bigReady;
        boolean smallCanFire = autoYawFeasible && yawReady && smallReady;

        // ---- 扳机释放 + 最短保持（语义见 TriggerGate：强制释放 > 取消允许/不可行 > 到位 > 保持窗口）----
        // 窗口只吸收"转速/俯仰就绪"的抖动，续开仍要求指向有效（yawHoldOk），不吸收指向失效。
        boolean yawHoldOk = autoYawFeasible && yawReady;
        double nowSec = seconds();
        bigGate.update(forceReleaseBig, allowFire, autoYawFeasible, bigCanFire,
                yawHoldOk, nowSec, fireHoldTime);
        smallGate.update(forceReleaseSmall, allowFire, autoYawFeasible, smallCanFire,
                yawHoldOk, nowSec, fireHoldTime);

        shooterBig.setTrigger(bigGate.isFired());
        shooterSmall.setTrigger(smallGate.isFired());

        // 返回值统一为"是否具备发射条件"，与 allowFire / 强制释放无关；
        // 本帧是否实际扣扳机见遥测 Auto big fired / Auto small fired
        return new FireReady(bigCanFire, smallCanFire);
    }

    // ==================== 手动平转（速率控制） ====================

    /** @return 当前手动平转模式 */
    public YawMode getManualYawMode() {
        return manualYawMode;
    }

    /** @return 当前模式被维持的角度（弧度）；REL 为车体相对 yaw（饱和在 ±yawRange），ABS 为场地绝对朝向，GOAL 为相对球门方位偏差 */
    public double getManualAngle() {
        return manualAngle;
    }

    /**
     * @return 自动瞄准下本帧大球扳机是否处于释放状态
     *         （含 {@link #fireHoldTime} 的最短保持窗口与强制释放）
     */
    public boolean isBigFiring() {
        return bigGate.isFired();
    }

    /**
     * @return 自动瞄准下本帧小球扳机是否处于释放状态
     *         （含 {@link #fireHoldTime} 的最短保持窗口与强制释放）
     */
    public boolean isSmallFiring() {
        return smallGate.isFired();
    }

    /**
     * 切换手动平转模式。若已初始化，则以上一帧已钳制目标为基准换算状态，
     * 使切换瞬间下发的 yaw 目标保持连续（不跳变、不换到不可达角度）。
     *
     * @param mode 新模式；null 或与当前相同则忽略
     */
    public void setManualYawMode(YawMode mode) {
        if (mode == null || mode == manualYawMode) {
            return;
        }
        manualYawMode = mode;
        if (manualInitialized) {
            rebaseManualAngle(yawController.getTargetYaw(), poseOrNull(), resolveGoal());
        }
    }

    /** RELATIVE → ABSOLUTE → GOAL → RELATIVE */
    public void cycleManualYawMode() {
        switch (manualYawMode) {
            case RELATIVE:
                setManualYawMode(YawMode.ABSOLUTE);
                break;
            case ABSOLUTE:
                setManualYawMode(YawMode.GOAL);
                break;
            default:
                setManualYawMode(YawMode.RELATIVE);
                break;
        }
    }

    /**
     * 手动模式：把角速度输入积分进当前模式被维持的角，再映射为平转目标。
     * GOAL 模式始终使用 {@link RobotPosition#getGoalPose()} 的当前球门，
     * 与 {@link #setManualYawMode} 的换算基准保持同一来源。
     *
     * @param yawRate     目标角速度（弧度/秒，车体相对方向，逆时针为正）
     * @param pitchBig    大球俯仰舵机位置
     * @param pitchSmall  小球俯仰舵机位置
     * @param speedBig    大球目标飞轮转速 (tick/s)
     * @param speedSmall  小球目标飞轮转速 (tick/s)
     * @param triggerBig  大球扳机是否释放
     * @param triggerSmall 小球扳机是否释放
     * @return 两发射器是否具备发射条件
     */
    public FireReady updateManual(double yawRate, double pitchBig, double pitchSmall,
                                  int speedBig, int speedSmall,
                                  boolean triggerBig, boolean triggerSmall) {
        aimSource = AimSource.MANUAL;
        clearAutoFireState();

        double now = seconds();
        double dt = (lastManualTime == 0) ? 0.02 : (now - lastManualTime);
        lastManualTime = now;
        if (dt <= 0 || dt > 0.1) {
            // 首帧或长时间停顿：用标称周期，避免一次积分跳变
            dt = 0.02;
        }

        Pose2d pose = poseOrNull();
        Pose2d goal = resolveGoal();

        if (!manualInitialized) {
            // 用编码器实测角初始化，首帧目标 = 当前实际角度，避免上电瞬间猛转
            rebaseManualAngle(measuredYaw(), pose, goal);
            manualInitialized = true;
        }

        manualAngle += yawRate * dt;
        // 按当前模式映射为可达的车体相对 yaw；返回值即本帧下发的目标
        double yaw = clampManualAngle(pose, goal);
        manualYawTarget = yaw;

        boolean yawReady = yawController.update(yaw);
        boolean bigReady = shooterBig.update(speedBig, pitchBig, triggerBig);
        boolean smallReady = shooterSmall.update(speedSmall, pitchSmall, triggerSmall);

        // 发射完成（扳机释放 true → false 的下降沿）→ 清空球道计数，
        // 一次发射流程结束后整条球道已空，计数归零以便下一轮重新统计
        if ((lastTriggerBig && !triggerBig) || (lastTriggerSmall && !triggerSmall)) {
            RobotPosition rp = RobotPosition.getInstanceOrNull();
            if (rp != null) {
                rp.resetBallCount();
            }
        }
        lastTriggerBig = triggerBig;
        lastTriggerSmall = triggerSmall;

        return new FireReady(yawReady && bigReady, yawReady && smallReady);
    }

    /**
     * 清空自动发射状态。切到手动或完全开放环目标时调用：只清保持窗口不够，
     * 还必须清 fired，否则 {@link #isBigFiring()} / {@link #isSmallFiring()}
     * 会残留上一帧的 true（且此时遥测不再显示 Auto fired 行，属静默残留）。
     */
    private void clearAutoFireState() {
        bigGate.clear();
        smallGate.clear();
    }

    /**
     * 当前模式下把"被维持角度"与车体相对 yaw 互相换算的加性基准：
     * 正变换 {@code out = wrap(manualAngle + offset)}，逆变换 {@code manualAngle = out − offset}。
     *
     * <p>返回 {@code null} 表示被维持量不是"相对 yaw 的平移"：RELATIVE 下被维持量本身就是
     * 相对 yaw（需要饱和而非折返），ABS/GOAL 缺定位时语义未知（只能折算输出、不回写）。
     * 正/逆变换共用这一个基准，避免两处手写公式少一个负号。
     */
    private Double manualOffset(Pose2d pose, Pose2d goal) {
        switch (manualYawMode) {
            case ABSOLUTE:
                return (pose != null) ? -pose.heading.toDouble() : null;
            case GOAL:
                return (pose != null && goal != null) ? goalBodyYaw(pose, goal) : null;
            default:
                return null;
        }
    }

    /** 以车体相对 yaw 为基准，按当前模式换算被维持的角度 */
    // 包私有而非 private：供同包单元测试直接验证正/逆变换（无行为差异）
    void rebaseManualAngle(double bodyYaw, Pose2d pose, Pose2d goal) {
        Double offset = manualOffset(pose, goal);
        manualAngle = (offset != null) ? bodyYaw - offset : bodyYaw;
    }

    /**
     * 把当前模式被维持的角度映射为可达的车体相对 yaw，返回值即本帧下发目标。
     *
     * <p>平转机械限位为 ±{@code yawRange}（默认 ±π，<b>硬限位</b>），可达集合就是
     * {@code [-yawRange, yawRange]}，任一指向在该区间内的表示唯一，因此：
     * <ul>
     *   <li>REL：被维持量本身就是相对 yaw，必须<b>饱和钳制</b>（{@code manualAngle ∈ ±yawRange}），
     *       不能折叠到另一端，否则越过限位一步就会让炮台反向整圈扫回、且推杆方向反转；</li>
     *   <li>ABS / GOAL：被维持量是场地角 / 球门偏差，先按 θ（或 goalYaw）折算成相对 yaw，
     *       取 (-π, π] 内表示后钳制；<b>输出一旦被钳制（含 2π 折算）就把被维持量回写为
     *       输出对应的等价表示</b>——回写值与原值只可能相差 2π 的整数倍，锁定朝向与偏差
     *       语义都不变；不回写则用户顶在限位继续推杆会把状态越积越远，反向推杆先要
     *       "消化"这段累积量，表现为一段无响应的死区；</li>
     *   <li>ABS/GOAL 缺定位时被维持量语义不是相对 yaw，只折算并钳制输出、不写回状态。</li>
     * </ul>
     * 不采用「叠加 2π 选最近分支」：硬限位下分支值超出机械行程，钳制后会指向错误方向。
     *
     * @param pose 当前位姿；ABS/GOAL 模式缺失时退化为 ±yawRange 钳制
     * @param goal 当前球门；GOAL 模式缺失时退化为 ±yawRange 钳制
     * @return 本帧下发的车体相对 yaw（已钳制到 ±yawRange）
     */
    double clampManualAngle(Pose2d pose, Pose2d goal) {
        double range = YawController.yawRange;
        if (manualYawMode == YawMode.RELATIVE) {
            // 被维持量即车体相对 yaw：撞硬限位必须饱和，不能折叠到另一侧端点
            manualAngle = clampAngle(manualAngle, range);
            return manualAngle;
        }
        Double offset = manualOffset(pose, goal);
        if (offset == null) {
            // ABS/GOAL 缺定位：状态语义不是相对 yaw（可能停在其它 2π 分支），折算后钳制输出、不写回状态
            return clampToRange(manualAngle, range);
        }
        double out = clampToRange(manualAngle + offset, range);
        // 状态跟随本帧输出：out − offset 与原 manualAngle 只差 2π 整数倍，锁定语义不变；
        // 不回写则顶在限位继续推杆会把累积量越积越远，反向推杆先要"消化"，形成死区
        if (Double.isFinite(out)) {
            manualAngle = out - offset;
        }
        return out;
    }

    /** 饱和钳制到 ±range，不做 2π 折返（用于被维持量本身就是相对 yaw 的场景） */
    private static double clampAngle(double angle, double range) {
        return Math.max(-range, Math.min(range, angle));
    }

    /**
     * 先把角度折算到 (-π, π]（硬限位行程内的唯一可达表示），再饱和钳制到 ±range。
     * <p>不叠加 2π 分支：分支值超出机械行程，钳制后得到的是错误方向。
     */
    private static double clampToRange(double angle, double range) {
        return clampAngle(Vector2D.normalizeAngle(angle), range);
    }

    /** @return 最近一次 update() 之前直接用编码器实测的平转角（供首帧初始化） */
    private double measuredYaw() {
        return yawController.getMeasuredYaw();
    }

    private static Pose2d poseOrNull() {
        RobotPosition robotPosition = RobotPosition.getInstanceOrNull();
        Pose2d pose = (robotPosition != null) ? robotPosition.getPose2d() : null;
        return isFinitePose(pose) ? pose : null;
    }

    /** @return {@link RobotPosition} 的当前球门；无定位或球门无效时为 null */
    private static Pose2d resolveGoal() {
        RobotPosition robotPosition = RobotPosition.getInstanceOrNull();
        Pose2d goal = (robotPosition != null) ? robotPosition.getGoalPose() : null;
        return isFinitePose(goal) ? goal : null;
    }

    /** @return 位姿的 x/y/heading 是否均为有限值（防 EKF 数值异常污染整条控制链） */
    private static boolean isFinitePose(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.position.x)
                && Double.isFinite(pose.position.y)
                && Double.isFinite(pose.heading.toDouble());
    }

    /**
     * 两发射器指向球门的劣角平分线（车体相对，弧度）。
     * 与自动瞄准同一公式（共用 {@link #bisectorYaw}），但独立实现、不调用
     * {@link Shooter#solve}，避免改写其仰角档位滞回状态（{@code selectedModel}）。
     *
     * <p>退化（两发射器方位接近相反）时保持上一帧有效方位；从未有有效值则把当前平转目标
     * <b>锁存</b>为基准——与自动瞄准在 {@code maxShooterYawSpread} 超限时的处理一致，
     * 避免把球门方位错当成车头方向（{@code atan2(0,0)=0}）。
     *
     * <p><b>必须锁存，不能每帧回读 {@code getTargetYaw()}</b>：它正是本函数的返回值经
     * {@link #clampManualAngle} 钳制后的结果，每帧回读会形成
     * {@code bearing ← 上一帧输出} 的正反馈。由于 GOAL 模式每帧把
     * {@code manualAngle} 回写为 {@code out − bearing}，偏移量会被锁定不再吸收，
     * 摇杆回中后目标仍按该偏移量逐帧自增并撞向 ±yawRange 限位。锁存后 {@code bearing}
     * 为常量，公式退化为 {@code T = C + m}，只有摇杆能改变目标。
     */
    double goalBodyYaw(Pose2d pose, Pose2d goal) {
        double yaw = bisectorYaw(AimGeometry.bodyYawToGoal(pose, goal, RelPosBig),
                AimGeometry.bodyYawToGoal(pose, goal, RelPosSmall));
        if (!Double.isFinite(yaw)) {
            if (!Double.isFinite(lastGoalBodyYaw)) {
                // 首帧即退化：把当前平转目标锁存为基准（此后不再读取，避免正反馈）
                lastGoalBodyYaw = yawController.getTargetYaw();
            }
            return lastGoalBodyYaw;
        }
        lastGoalBodyYaw = yaw;
        return yaw;
    }

    /**
     * 两个方位角的劣角平分线。夹角超过 {@link #maxShooterYawSpread} 时单位向量和退化
     * （{@code atan2(0,0)=0} 会把平分线错当成车头方向），返回 NaN 表示不可解。
     * 自动瞄准与手动 GOAL 模式共用此判据，避免两条路径行为不一致。
     */
    static double bisectorYaw(double bigYaw, double smallYaw) {
        return AimGeometry.bisectorYaw(bigYaw, smallYaw, maxShooterYawSpread);
    }

    /** 单帧目标：该发射器本帧应使用的飞轮速度与俯仰（发射、预载、保持共用同一类型） */
    private static final class Target {
        final int speed;
        final double pitch;

        Target(int speed, double pitch) {
            this.speed = speed;
            this.pitch = pitch;
        }
    }

    /**
     * 按自动瞄准规则决定单个发射器本帧目标。
     *
     * <p>分支顺序不可调整：强制释放 > 平转不可行（保持上一帧）> 允许发射（采用解算值）
     * > 预载。平转不可行时取的是上一帧下发值（{@code getTargetSpeed/getTargetPitch}
     * 均为上一次 {@code update} 的目标），因此本帧不会产生新的机构目标跳变。
     */
    private Target resolveTarget(Shooter shooter, Shooter.ShotSolution sol,
                                 boolean forceRelease, boolean allowFire, boolean preloadEnabled) {
        if (forceRelease) {
            return new Target(sol.speed, sol.pitch);
        }
        if (!autoYawFeasible) {
            return new Target(shooter.getTargetSpeed(), shooter.getTargetPitch());
        }
        if (allowFire) {
            return new Target(sol.speed, sol.pitch);
        }
        return preloadFor(shooter, sol, preloadEnabled);
    }

    /**
     * 预载目标：关闭 → 0 速 + 最高档；不可发射区 → 0 速 + 最低档；否则<b>限速</b> − 解算档位不变。
     *
     * <p>预载与发射的俯仰姿态必须一致：{@code sol.speed} 超过 {@code preSpeedMax} 时只钳制<b>转速</b>，
     * 不能改仰角。旧实现此时返回 {@code getMaxPitch()}，使"按 A 的瞬间"俯仰目标跳变 →
     * {@code pitchCommandTime} 重置 → 需再等 {@code pitchSettleTime}（0.25 s）才可能发射；
     * 松开 A 又摆回最高档，下一次再等一遍，A 键抖动时反复触发。
     * 最高/最低档只保留给"预载关闭 / 禁射区"两个安全姿态。
     */
    private Target preloadFor(Shooter shooter, Shooter.ShotSolution sol, boolean preloadEnabled) {
        if (!preloadEnabled) {
            return new Target(0, shooter.getMaxPitch());
        }
        if (autoInNoShootArea) {
            return new Target(0, shooter.getMinPitch());
        }
        int preSpeedMax = shooter.getParams().preSpeedMax;
        return new Target(Math.min(sol.speed, preSpeedMax), sol.pitch);
    }

    /** 停止炮台：平转停止输出，两个发射器停转并关闭扳机 */
    public void stop() {
        yawController.stop();
        shooterBig.stop();
        shooterSmall.stop();

        aimSource = AimSource.NONE;
        autoYawFeasible = false;
        autoYawSpread = 0;
        autoBigSolution = null;
        autoSmallSolution = null;
        bigGate.clear();
        smallGate.clear();

        // 手动状态一并复位：下次进入手动时重新用实测角初始化
        manualInitialized = false;
        manualAngle = 0;
        lastManualTime = 0;
        // 同步扳机基准：stop() 已把扳机关闭，若不同步，
        // 下一次 update(trigger=false) 会产生假下降沿从而误清零球道计数
        lastTriggerBig = false;
        lastTriggerSmall = false;
    }

    /** 输出遥测（调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        yawController.setTelemetry();
        shooterBig.setTelemetry();
        shooterSmall.setTelemetry();

        if (aimSource == AimSource.MANUAL) {
            telemetry.addLine("-- turret manual aim --");
            telemetry.addData("Manual yaw mode", manualYawMode);
            telemetry.addData("Manual angle (deg)", "%.1f", Math.toDegrees(manualAngle));
            telemetry.addData("Manual yaw target (deg)", "%.1f", Math.toDegrees(manualYawTarget));
        }

        if (aimSource == AimSource.AUTO) {
            telemetry.addLine("-- turret auto aim --");
            telemetry.addData("AutoYaw target (deg)", "%.1f", Math.toDegrees(autoYawTarget));
            telemetry.addData("AutoYaw feasible", autoYawFeasible);
            telemetry.addData("AutoYaw spread (deg)", "%.1f", Math.toDegrees(autoYawSpread));
            telemetry.addData("AutoYaw on target", yawController.isOnTarget());
            telemetry.addData("Auto preload", autoPreloadEnabled);
            telemetry.addData("Auto in no-shoot area", autoInNoShootArea);
            telemetry.addData("Auto big fired", bigGate.isFired());
            telemetry.addData("Auto small fired", smallGate.isFired());
            if (autoBigSolution != null) {
                telemetry.addData("Auto big range (in)", "%.1f", autoBigSolution.range);
                telemetry.addData("Auto big pitch", "%.3f", autoBigSolution.pitch);
                telemetry.addData("Auto big speed (tick/s)", autoBigSolution.speed);
            }
            if (autoSmallSolution != null) {
                telemetry.addData("Auto small range (in)", "%.1f", autoSmallSolution.range);
                telemetry.addData("Auto small pitch", "%.3f", autoSmallSolution.pitch);
                telemetry.addData("Auto small speed (tick/s)", autoSmallSolution.speed);
            }
        }
    }
}