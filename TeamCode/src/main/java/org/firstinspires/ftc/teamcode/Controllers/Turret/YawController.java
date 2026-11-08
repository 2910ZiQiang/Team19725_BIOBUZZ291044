package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.PID.PIDSVAController;
import org.firstinspires.ftc.teamcode.utility.PID.SlotConfig;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 平转机构控制器：把目标平转角换算成电机目标 tick，用 PIDSVA 做位置闭环。
 *
 * <p><b>角度约定</b>：yaw = 0 表示两个发射器朝向车头方向，逆时针为正（与
 * {@code FTC_Format.md} 的相对坐标系一致）。目标平转角会被钳制到
 * ±{@link #yawRange} 内，避免撞机械限位。
 *
 * <p><b>电机方向</b>：由 {@link #motorReversed} 选择。它同时翻转功率与编码器读数，用来保证
 * "正功率 → tick 增加"——这是位置环稳定的前提（反之 P 环变成正反馈，输出会打满）。
 * {@link #tickPerRad} 只描述"tick → yaw"的映射（含符号），<b>不能</b>替代方向开关：
 * 闭环特征值为 −gK（g 为 d(tick)/d(power)），与 tickPerRad 的符号无关。
 *
 * <p><b>零位标定</b>：{@link #yawZeroTicks} 为"机械上 yaw = 0 时"的编码器读数，
 * 必须实机标定；{@link #tickPerRad} 同样需要标定（转动已知角度，读编码器增量）。
 *
 * <p>用法：每帧调用一次 {@link #update(double)}，返回平转机构是否已到位。
 */
@Config
public class YawController {

    /** 平转电机设备名 */
    private static final String MOTOR_NAME = "yawMotor";

    // ==================== 超参数 (FTC Dashboard 实时生效) ====================

    /** 平转机构最大角度绝对值（弧度），目标会被钳制到 ±该值 */
    public static double yawRange = Math.PI;

    /** 每个弧度对应的电机 tick 变化量（tick → yaw 的映射，含符号），需实机标定 */
    public static double tickPerRad = 1000.0;

    /**
     * 方向开关：置 true 时对调电机功率与编码器方向，保证"正功率 → tick 增加"。
     * 装配/接线相反（正功率使 tick 减少）时必须置 true，否则位置环为正反馈、输出打满。
     * 只在构造时下发，Dashboard 改动后需重新初始化 opmode 才生效（与 Shooter.motorReversed 一致）。
     */
    public static boolean motorReversed = false;

    /** yaw = 0（朝车头）时的编码器读数，需实机标定 */
    public static double yawZeroTicks = 0.0;

    /** 位置环 P 增益（输出为功率，误差为 tick） */
    public static double kP = 0.005;
    /** 位置环 I 增益 */
    public static double kI = 0.0;
    /** 位置环 D 增益 */
    public static double kD = 0.0;
    /** 静态摩擦前馈系数 */
    public static double kS = 0.05;
    /** 速度前馈系数，量纲为 (功率)/(tick/s) */
    public static double kV = 0.0002;
    /** 加速度前馈系数（当前按 0 加速度使用） */
    public static double kA = 0.0;

    /** 积分上限（防积分饱和） */
    public static double maxI = 1.0;
    /** I 分离阈值 (tick)：误差小于该值才累加积分 */
    public static double iZone = 200.0;

    /** 输出下限（功率） */
    public static double outputMin = -1.0;
    /** 输出上限（功率） */
    public static double outputMax = 1.0;

    /** 到位判据：位置容差（弧度），供"能否发射"判断使用 */
    public static double onTargetTolerance = 0.02;
    /** 到位判据：平转角速度容差 (rad/s)，超过则该帧不算到位 */
    public static double onTargetRateTolerance = 0.15;
    /** 到位判据：位置与角速度容差需连续满足的帧数（去抖，防止高速扫过时单帧误判到位） */
    public static int onTargetFrames = 3;

    /**
     * 折返抑制窗口（弧度）：目标在 ±yawRange 两端出现同一朝向的两种表示
     * （如 +π−ε ↔ −π+ε）时，若折算成最短角差不超过该值，就保持上一帧目标，不整圈反转。
     *
     * <p><b>必须与 {@link #onTargetTolerance} 独立</b>。到位容差 0.02 rad 只有 ±0.57°，
     * 定位/解算噪声（正后方附近通常有 1°~3°）很容易越过该窗口：此时两种表示折算后相差
     * 2ε 超过容差，判据不成立，炮台会按 2π−2ε（≈350°）整圈行程扫过去，扫回时再反向扫一次。
     *
     * <p>取 0.1 rad（≈5.7°）可覆盖常规定位噪声，抑制后的最大物理指向误差为 2ε ≤ 0.1 rad。
     * 代价是正后方附近的目标变更若超过该窗口仍会绕行整圈——硬限位下那是唯一可达路径。
     * <p>抑制期间的实际偏差会记入 {@link #getSeamError()} 并参与到位判据，因此"判到位"不会
     * 再出现"炮台实际偏着"的情况（见 {@code update()} 中的 positionOk）。
     */
    public static double yawSeamHysteresis = 0.1;

    // ==================== 运行时状态 ====================

    public final DcMotorEx motor;

    private final Telemetry telemetry;
    /** 复用同一个 slot：每帧用 withXxx 刷新参数后 resetSlot（见 SlotConfig 类注释） */
    private final PIDSVAController controller = new PIDSVAController();
    private final SlotConfig slot = new SlotConfig();

    /** 上一次 update 的时间戳 (秒) */
    private double lastTime;

    /** 最近一次下发的目标平转角（钳制后，弧度） */
    private double targetYaw = 0;
    /** 最近一次下发的功率（供遥测） */
    private double power = 0;
    /** 最近一次计算的到位标志 */
    private boolean onTarget = false;

    /** 最近一次 update() 采到的编码器读数 (tick)，供读取与遥测复用 */
    private double cachedTicks = 0;
    /** 最近一次 update() 采到的目标编码器读数 (tick)，供遥测复用 */
    private double cachedTargetTicks = 0;
    /** 上一次 update() 采到的编码器读数 (tick)，用于差分出平转角速度；NaN = 尚无历史 */
    private double lastTicks = Double.NaN;
    /** 最近一次 update() 采到的平转角速度 (rad/s)，供遥测复用 */
    private double cachedYawRate = 0;
    /** 连续满足位置与角速度容差的帧数（到位去抖计数） */
    private int onTargetCount = 0;
    /** 最近一次 update() 的折返抑制残差 (rad)：保持旧目标时与真实期望方向的物理偏差，未抑制时为 0 */
    private double cachedSeamError = 0;

    public YawController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        // 软件位置闭环：setPower 直接控制功率，编码器仍可读位置
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 位置保持需要刹车，避免断电后炮台自由转动
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 方向开关：保证"正功率 → tick 增加"（位置环稳定的前提）
        motor.setDirection(motorReversed
                ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        controller.withSlot0(new SlotConfig()
                .withKP(kP).withKI(kI).withKD(kD)
                .withMaxI(maxI).withIZone(iZone)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax));

        lastTime = now();
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    /**
     * 每帧调用一次：钳制目标角度并执行位置闭环。
     *
     * <p>目标在 ±{@code yawRange} 两端出现同一朝向的两种 2π 表示（例如 +π−ε ↔ −π+ε）
     * 且最短角差不超过 {@link #yawSeamHysteresis} 时，会保持上一帧目标，避免在正后方附近
     * 逐帧整圈反转（见方法内注释）。
     *
     * @param yaw 目标平转角（弧度，0 表示朝车头），超出 ±{@link #yawRange} 会被钳制
     * @return 平转机构是否已到达目标角度：位置误差小于 {@link #onTargetTolerance}、
     *         折返抑制残差不超过 {@link #onTargetTolerance}、且角速度小于
     *         {@link #onTargetRateTolerance}，并连续保持 {@link #onTargetFrames} 帧
     */
    public boolean update(double yaw) {
        double currentTime = now();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        // 首帧或长时间停顿后 dt 异常，用标称循环周期兜底，避免微分项发散
        if (dt <= 0 || dt > 1.0) {
            dt = 0.02;
        }

        // 非有限目标（如定位/解算异常）直接停转，避免 NaN 传入 PID 与电机
        if (!Double.isFinite(yaw)) {
            motor.setPower(0);
            power = 0;
            onTarget = false;
            onTargetCount = 0;
            lastTicks = Double.NaN;
            cachedYawRate = 0;
            cachedSeamError = 0;
            controller.reset();
            return false;
        }

        // 钳制到机械可达范围
        double requestedYaw = Math.max(-yawRange, Math.min(yawRange, yaw));

        // 折返（±2π）抑制：±yawRange 两端在物理上可能是"同一朝向的两种表示"。
        // 例如目标从 +π−ε 抖到 −π+ε 时，两个表示相差 2π，直接下发会让炮台按整圈行程
        // 反转（满功率扫过全程、并在两端之间反复来回，永远到不了位）。
        // 判据：与原目标的角度差超过 π（说明落在另一侧端点），但折算成最短角差后
        // 不超过折返抑制窗口 yawSeamHysteresis（说明物理朝向几乎没变）→ 视为同一朝向，保持上一帧目标。
        // 注意这里不能用位容差 onTargetTolerance：它只有 0.02 rad，正后方附近 1° 以上的
        // 定位噪声就足以让判据失效并触发整圈扫动。
        // 只抑制小角差抖动；真实的超窗口目标变更照常下发（此时绕行是其唯一可达路径）。
        // 抑制时记录"保持目标"与真实期望方向的物理角差（≤ yawSeamHysteresis），
        // 供到位判据使用：否则炮台会带着最多 0.1 rad 的偏角被判到位并自动扣扳机。
        double seamError = 0.0;
        double deltaYaw = requestedYaw - targetYaw;
        if (Math.abs(deltaYaw) > Math.PI
                && Math.abs(Vector2D.normalizeAngle(deltaYaw)) <= yawSeamHysteresis) {
            seamError = Math.abs(Vector2D.normalizeAngle(deltaYaw));
            requestedYaw = targetYaw;
        }
        cachedSeamError = seamError;
        targetYaw = requestedYaw;

        double targetTicks = yawZeroTicks + targetYaw * tickPerRad;
        // 本帧唯一的硬件采样点：编码器只读一次，getter 与遥测复用缓存
        double currentTicks = motor.getCurrentPosition();
        cachedTicks = currentTicks;
        cachedTargetTicks = targetTicks;

        // 平转角速度：由编码器差分得到（符号随 tickPerRad 自动正确）
        double tickRate = Double.isNaN(lastTicks) ? 0.0 : (currentTicks - lastTicks) / dt;
        lastTicks = currentTicks;
        cachedYawRate = (Math.abs(tickPerRad) > 1e-9) ? tickRate / tickPerRad : 0.0;

        // 每帧刷新参数，使 Dashboard 上的改动即时生效
        slot.withKP(kP).withKI(kI).withKD(kD)
                .withMaxI(maxI).withIZone(iZone)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax);
        controller.resetSlot(slot);

        // PIDSVA 位置闭环：SVA 前馈的参考速度/加速度由控制器内部按 setpoint 变化率推导（见 PID/Guide.md）
        power = controller.calculate(targetTicks, currentTicks, dt, false);
        motor.setPower(power);

        // 到位判据：位置误差在容差内且角速度足够小，并连续保持 onTargetFrames 帧
        // （仅判单帧位置误差时，炮台高速扫过目标会在容差带内瞬时判为到位，导致带偏发射）
        // 折返抑制残差也是"指向误差"：偏差超过容差时不算到位，禁止自动发射
        boolean positionOk =
                Math.abs(targetTicks - currentTicks) < onTargetTolerance * Math.abs(tickPerRad)
                && cachedSeamError <= onTargetTolerance;
        // 角速度判据下限：单个编码器 tick 折算到当前帧长的角速度的 3 倍。
        // 否则 tickPerRad 较小时，量化噪声本身（1~2 tick/帧）就会让判据永远不满足。
        double tickRad = (Math.abs(tickPerRad) > 1e-9) ? 1.0 / Math.abs(tickPerRad) : 0.0;
        double rateTolerance = Math.max(onTargetRateTolerance, 3.0 * tickRad / dt);
        boolean rateOk = Math.abs(cachedYawRate) < rateTolerance;
        if (positionOk && rateOk) {
            if (onTargetCount < onTargetFrames) {
                onTargetCount++;
            }
        } else {
            onTargetCount = 0;
        }
        onTarget = onTargetCount >= onTargetFrames;
        return onTarget;
    }

    /** 停止平转（功率归零并复位控制器状态），不改变机械位置 */
    public void stop() {
        motor.setPower(0);
        power = 0;
        onTarget = false;
        onTargetCount = 0;
        lastTicks = Double.NaN;
        cachedYawRate = 0;
        cachedSeamError = 0;
        controller.reset();
    }

    // ==================== 读取 ====================

    /** @return 最近一次 update() 采到的平转角（弧度，已扣除零位） */
    public double getCurrentYaw() {
        return (cachedTicks - yawZeroTicks) / tickPerRad;
    }

    /** @return 最近一次下发的目标平转角（弧度，钳制后） */
    public double getTargetYaw() {
        return targetYaw;
    }

    /** @return 最近一次计算的到位标志 */
    public boolean isOnTarget() {
        return onTarget;
    }

    /** @return 最近一次 update() 的折返抑制残差（弧度）：保持旧目标时与真实期望方向的物理偏差，未抑制时为 0 */
    public double getSeamError() {
        return cachedSeamError;
    }

    /** @return 最近一次下发的功率 */
    public double getPower() {
        return power;
    }

    /** 输出遥测（只读 update() 缓存，调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        telemetry.addData("Yaw target (deg)", "%.1f", Math.toDegrees(targetYaw));
        telemetry.addData("Yaw current (deg)", "%.1f", Math.toDegrees(getCurrentYaw()));
        telemetry.addData("Yaw error (deg)", "%.1f", Math.toDegrees(targetYaw - getCurrentYaw()));
        telemetry.addData("Yaw rate (rad/s)", "%.2f", cachedYawRate);
        telemetry.addData("Yaw power", "%.3f", power);
        telemetry.addData("Yaw on target", onTarget);
        telemetry.addData("Yaw seam error (deg)", "%.1f", Math.toDegrees(cachedSeamError));
        telemetry.addData("Yaw ticks", "%.0f / %.0f", cachedTicks, cachedTargetTicks);
    }
}