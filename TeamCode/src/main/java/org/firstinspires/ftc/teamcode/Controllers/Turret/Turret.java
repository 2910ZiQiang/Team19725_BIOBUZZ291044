package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
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

    /** 最近一次是否由自动瞄准 update 驱动（手动 update 会置 false） */
    private boolean autoAimActive = false;
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
    /** 最近一帧实际释放的扳机 */
    private boolean autoBigFired = false;
    private boolean autoSmallFired = false;
    /** 大球扳机本段释放的最短保持截止时刻 (秒，{@code now()} 口径)；0 表示不在保持窗口 */
    private double bigFireHoldUntil = 0;
    /** 小球扳机本段释放的最短保持截止时刻 (秒，{@code now()} 口径)；0 表示不在保持窗口 */
    private double smallFireHoldUntil = 0;

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
    /** 最近一次是否由 updateManual 驱动 */
    private boolean manualActive = false;
    /** 最近一次可解的球门方位（车体相对，弧度）：GOAL 模式方位退化时保持此值；NaN = 尚无有效值 */
    private double lastGoalBodyYaw = Double.NaN;

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
        autoAimActive = false;
        manualActive = false;
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
     *       {@link HypParams#NoShootArea}）时速度 0、仰角取最低档；解算速度不超过
     *       {@link Shooter.Params#preSpeedMax} 时直接采用；超过时用 preSpeedMax + 最高档。</li>
     * </ul>
     *
     * @param allowFire        是否允许发射（否则按预载规则驱动机构）
     * @param preload          是否启用预载（false 时预载速度恒为 0、仰角取最高档）
     * @param forceReleaseBig  是否强制释放大球扳机（无视就绪与平转超限）
     * @param forceReleaseSmall 是否强制释放小球扳机（无视就绪与平转超限）
     * @return 两发射器当前是否具备发射条件（平转到位 + 机构就绪），与 {@code allowFire}
     *         及强制释放无关；本帧是否实际扣扳机见遥测 {@code autoBigFired/autoSmallFired}
     */
    public FireReady update(boolean allowFire, boolean preload,
                            boolean forceReleaseBig, boolean forceReleaseSmall) {
        autoAimActive = true;
        autoPreloadEnabled = preload;
        manualActive = false;
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
            autoBigFired = false;
            autoSmallFired = false;
            bigFireHoldUntil = 0;
            smallFireHoldUntil = 0;

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
        // 平转机械限位为 ±yawRange（默认 ±π，硬限位，只能转一圈），目标必须落在该区间内。
        // 硬限位下 [-π, π] 是完整可达行程，任一指向的唯一可达表示就是折叠到 (-π, π] 的值；
        // 因此不再叠加 2π 分支：分支值超出机械行程，钳制后反而指向错误方向。
        // 注：正后方附近 ±π 两种表示逐帧翻转引起的整圈反转，由 YawController.yawSeamHysteresis
        // （默认 0.1 rad ≈5.7°）吸收；只有当两表示折算后的最短角差不超过该窗口时才保持旧目标。
        // 保持期间的实际偏差计入 YawController 的到位判据（> onTargetTolerance 不算到位、禁止自动发射），
        // 因此不会出现"炮台偏着却判到位"的带偏发射。
        // 超出窗口的目标变更仍按硬限位下的唯一路径绕行整圈——这是机械结构决定的，不是 bug。
        //
        // 但单位向量求和有一个退化点：两方位相差接近 180° 时和向量趋近于零，atan2(0,0) 返回 0，
        // 平分线失去意义。这时炮口会被推到车头方向、却照常判到位并自动扣扳机。
        // 因此先用夹角做一次有效性判定：夹角超过 maxShooterYawSpread 直接判不可行
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

        // ---- 各发射器独立决定本帧目标 ----
        int speedBig;
        double pitchBig;
        if (forceReleaseBig) {
            speedBig = bigSol.speed;
            pitchBig = bigSol.pitch;
        } else if (!autoYawFeasible) {
            speedBig = shooterBig.getTargetSpeed();
            pitchBig = shooterBig.getTargetPitch();
        } else if (allowFire) {
            speedBig = bigSol.speed;
            pitchBig = bigSol.pitch;
        } else {
            Preload preloadBig = preloadFor(shooterBig, bigSol, preload);
            speedBig = preloadBig.speed;
            pitchBig = preloadBig.pitch;
        }

        int speedSmall;
        double pitchSmall;
        if (forceReleaseSmall) {
            speedSmall = smallSol.speed;
            pitchSmall = smallSol.pitch;
        } else if (!autoYawFeasible) {
            speedSmall = shooterSmall.getTargetSpeed();
            pitchSmall = shooterSmall.getTargetPitch();
        } else if (allowFire) {
            speedSmall = smallSol.speed;
            pitchSmall = smallSol.pitch;
        } else {
            Preload preloadSmall = preloadFor(shooterSmall, smallSol, preload);
            speedSmall = preloadSmall.speed;
            pitchSmall = preloadSmall.pitch;
        }

        // 先以扳机关闭下发，拿到本帧的机构就绪结果，再决定是否扣扳机
        boolean bigReady = shooterBig.update(speedBig, pitchBig, false);
        boolean smallReady = shooterSmall.update(speedSmall, pitchSmall, false);

        boolean bigCanFire = autoYawFeasible && yawReady && bigReady;
        boolean smallCanFire = autoYawFeasible && yawReady && smallReady;

        // ---- 扳机释放 + 最短保持（吸收就绪状态逐帧抖动）----
        // 飞轮稳态计数会因目标转速的每一点变化而清零，自动瞄准下转速随射程逐帧重算，
        // 到位状态可能在边界附近逐帧真假交替。若每帧重新决定扳机，舵机会逐帧开合、打断装填。
        // 因此一旦开始释放，就至少保持 fireHoldTime；只有条件连续失效超过该时长才关闭。
        // 例外：取消允许发射、平转超限（指向已不可达）、强制释放松手都立即关闭，不进入保持窗口。
        //
        // 关键：保持窗口只吸收"转速/俯仰就绪"的抖动，不吸收"指向失效"。
        // 若窗口内不比 yawHoldOk，球门切换（HIVE 翻转 → 目标跳变近 180°）或底盘持续转动时，
        // 只要出现一帧就绪，扳机就会被连续打开整个 fireHoldTime（120°/s 下约 18° 偏角）。
        boolean yawHoldOk = autoYawFeasible && yawReady;
        double nowSec = now();
        if (forceReleaseBig) {
            autoBigFired = true;
            bigFireHoldUntil = 0;
        } else if (!allowFire || !autoYawFeasible) {
            autoBigFired = false;
            bigFireHoldUntil = 0;
        } else if (bigCanFire) {
            autoBigFired = true;
            bigFireHoldUntil = nowSec + fireHoldTime;
        } else {
            autoBigFired = yawHoldOk && nowSec < bigFireHoldUntil;
        }

        if (forceReleaseSmall) {
            autoSmallFired = true;
            smallFireHoldUntil = 0;
        } else if (!allowFire || !autoYawFeasible) {
            autoSmallFired = false;
            smallFireHoldUntil = 0;
        } else if (smallCanFire) {
            autoSmallFired = true;
            smallFireHoldUntil = nowSec + fireHoldTime;
        } else {
            autoSmallFired = yawHoldOk && nowSec < smallFireHoldUntil;
        }

        if (autoBigFired) {
            shooterBig.setTrigger(true);
        }
        if (autoSmallFired) {
            shooterSmall.setTrigger(true);
        }

        // 返回值统一为"是否具备发射条件"，与 allowFire / 强制释放无关；
        // 本帧是否实际扣扳机见 autoBigFired / autoSmallFired（遥测）
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
        return autoBigFired;
    }

    /**
     * @return 自动瞄准下本帧小球扳机是否处于释放状态
     *         （含 {@link #fireHoldTime} 的最短保持窗口与强制释放）
     */
    public boolean isSmallFiring() {
        return autoSmallFired;
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
        manualActive = true;
        autoAimActive = false;
        clearAutoFireState();

        double now = now();
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

        return new FireReady(yawReady && bigReady, yawReady && smallReady);
    }

    /**
     * 清空自动发射状态。切到手动或完全开放环目标时调用：只清保持窗口不够，
     * 还必须清 {@code autoBigFired/autoSmallFired}，否则 {@link #isBigFiring()} /
     * {@link #isSmallFiring()} 会残留上一帧的 true（且此时遥测不再显示 Auto fired 行，属静默残留）。
     */
    private void clearAutoFireState() {
        autoBigFired = false;
        autoSmallFired = false;
        bigFireHoldUntil = 0;
        smallFireHoldUntil = 0;
    }

    /** 以车体相对 yaw 为基准，按当前模式换算被维持的角度 */
    private void rebaseManualAngle(double bodyYaw, Pose2d pose, Pose2d goal) {
        double theta = (pose != null) ? pose.heading.toDouble() : 0.0;
        switch (manualYawMode) {
            case ABSOLUTE:
                manualAngle = (pose != null) ? theta + bodyYaw : bodyYaw;
                break;
            case GOAL:
                manualAngle = (pose != null && goal != null)
                        ? bodyYaw - goalBodyYaw(pose, goal)
                        : bodyYaw;
                break;
            default:
                manualAngle = bodyYaw;
                break;
        }
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
    private double clampManualAngle(Pose2d pose, Pose2d goal) {
        double range = YawController.yawRange;
        switch (manualYawMode) {
            case ABSOLUTE:
                if (pose != null) {
                    // 锁定场地绝对朝向：manualAngle 为场地角，减去车头航向即为车体相对 yaw
                    double theta = pose.heading.toDouble();
                    double out = clampToRange(manualAngle - theta, range);
                    // 状态跟随本帧输出：out + theta 与原 manualAngle 只差 2π 整数倍，锁定语义不变
                    if (Double.isFinite(out)) {
                        manualAngle = out + theta;
                    }
                    return out;
                }
                break;
            case GOAL:
                if (pose != null && goal != null) {
                    // manualAngle 为相对球门方位的偏差：偏差 0 即 (bearing − θ)
                    double bearing = goalBodyYaw(pose, goal);
                    double out = clampToRange(bearing + manualAngle, range);
                    // 状态跟随本帧输出：out − bearing 与原偏差只差 2π 整数倍
                    if (Double.isFinite(out)) {
                        manualAngle = out - bearing;
                    }
                    return out;
                }
                break;
            case RELATIVE:
                // 被维持量即车体相对 yaw：撞硬限位必须饱和，不能折叠到另一侧端点
                manualAngle = clampAngle(manualAngle, range);
                return manualAngle;
            default:
                break;
        }
        // ABS/GOAL 缺定位：状态语义不是相对 yaw（可能停在其它 2π 分支），折算后钳制输出、不写回状态
        return clampToRange(manualAngle, range);
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
        double tickPerRad = YawController.tickPerRad;
        if (Math.abs(tickPerRad) < 1e-9) {
            return 0.0;
        }
        return (yawController.motor.getCurrentPosition() - YawController.yawZeroTicks) / tickPerRad;
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
    private double goalBodyYaw(Pose2d pose, Pose2d goal) {
        double yaw = bisectorYaw(bodyYawToGoal(pose, goal, RelPosBig),
                bodyYawToGoal(pose, goal, RelPosSmall));
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
    private static double bisectorYaw(double bigYaw, double smallYaw) {
        if (Math.abs(Vector2D.normalizeAngle(bigYaw - smallYaw)) > maxShooterYawSpread) {
            return Double.NaN;
        }
        return Math.atan2(Math.sin(bigYaw) + Math.sin(smallYaw),
                Math.cos(bigYaw) + Math.cos(smallYaw));
    }

    /** @return 单个发射器中心指向球门的车体相对方位角 */
    private static double bodyYawToGoal(Pose2d pose, Pose2d goal, Vector2D rel) {
        double theta = pose.heading.toDouble();
        Vector2D rotated = Vector2D.rotate(rel != null ? rel : Vector2D.ZERO, theta);
        double shooterX = pose.position.x + rotated.getX();
        double shooterY = pose.position.y + rotated.getY();
        return Vector2D.normalizeAngle(
                Math.atan2(goal.position.y - shooterY, goal.position.x - shooterX) - theta);
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    /** 预载目标：该发射器在非发射状态下应使用的速度与仰角 */
    private static final class Preload {
        final int speed;
        final double pitch;

        Preload(int speed, double pitch) {
            this.speed = speed;
            this.pitch = pitch;
        }
    }

    /**
     * 按自动瞄准规则计算预载目标。
     *
     * @param shooter        目标发射器
     * @param sol            该发射器的解算结果
     * @param preloadEnabled 预载是否开启
     * @return 预载速度与仰角
     */
    private Preload preloadFor(Shooter shooter, Shooter.ShotSolution sol, boolean preloadEnabled) {
        if (!preloadEnabled) {
            return new Preload(0, shooter.getMaxPitch());
        }
        if (autoInNoShootArea) {
            return new Preload(0, shooter.getMinPitch());
        }
        int preSpeedMax = shooter.getParams().preSpeedMax;
        if (sol.speed <= preSpeedMax) {
            return new Preload(sol.speed, sol.pitch);
        }
        return new Preload(preSpeedMax, shooter.getMaxPitch());
    }

    /** 停止炮台：平转停止输出，两个发射器停转并关闭扳机 */
    public void stop() {
        yawController.stop();
        shooterBig.stop();
        shooterSmall.stop();

        autoAimActive = false;
        autoYawFeasible = false;
        autoYawSpread = 0;
        autoBigSolution = null;
        autoSmallSolution = null;
        autoBigFired = false;
        autoSmallFired = false;
        bigFireHoldUntil = 0;
        smallFireHoldUntil = 0;

        // 手动状态一并复位：下次进入手动时重新用实测角初始化
        manualInitialized = false;
        manualActive = false;
        manualAngle = 0;
        lastManualTime = 0;
    }

    /** 输出遥测（调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        yawController.setTelemetry();
        shooterBig.setTelemetry();
        shooterSmall.setTelemetry();

        if (manualActive) {
            telemetry.addLine("-- turret manual aim --");
            telemetry.addData("Manual yaw mode", manualYawMode);
            telemetry.addData("Manual angle (deg)", "%.1f", Math.toDegrees(manualAngle));
            telemetry.addData("Manual yaw target (deg)", "%.1f", Math.toDegrees(manualYawTarget));
        }

        if (autoAimActive) {
            telemetry.addLine("-- turret auto aim --");
            telemetry.addData("AutoYaw target (deg)", "%.1f", Math.toDegrees(autoYawTarget));
            telemetry.addData("AutoYaw feasible", autoYawFeasible);
            telemetry.addData("AutoYaw spread (deg)", "%.1f", Math.toDegrees(autoYawSpread));
            telemetry.addData("AutoYaw on target", yawController.isOnTarget());
            telemetry.addData("Auto preload", autoPreloadEnabled);
            telemetry.addData("Auto in no-shoot area", autoInNoShootArea);
            telemetry.addData("Auto big fired", autoBigFired);
            telemetry.addData("Auto small fired", autoSmallFired);
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