package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.PID.PIDSVAController;
import org.firstinspires.ftc.teamcode.utility.PID.SlotConfig;

/**
 * 平转机构控制器：把目标平转角换算成电机目标 tick，用 PIDSVA 做位置闭环。
 *
 * <p><b>角度约定</b>：yaw = 0 表示两个发射器朝向车头方向，逆时针为正（与
 * {@code FTC_Format.md} 的相对坐标系一致）。目标平转角会被钳制到
 * ±{@link #yawRange} 内，避免撞机械限位。
 *
 * <p><b>电机方向</b>：不提供方向开关。若电机装配方向相反，把 {@link #tickPerRad}
 * 取负即可——误差以 tick 为单位计算，PID 输出符号随之自动正确。
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
    public static double yawRange = Math.PI / 2;

    /** 每个弧度对应的电机 tick 变化量；装配方向相反时取负 */
    public static double tickPerRad = 1000.0;

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

    /** 到位判据容差（弧度），供"能否发射"判断使用 */
    public static double onTargetTolerance = 0.02;

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

    public YawController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        // 软件位置闭环：setPower 直接控制功率，编码器仍可读位置
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 位置保持需要刹车，避免断电后炮台自由转动
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
     * @param yaw 目标平转角（弧度，0 表示朝车头），超出 ±{@link #yawRange} 会被钳制
     * @return 平转机构是否已到达目标角度
     */
    public boolean update(double yaw) {
        double currentTime = now();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        // 首帧或长时间停顿后 dt 异常，用标称循环周期兜底，避免微分项发散
        if (dt <= 0 || dt > 1.0) {
            dt = 0.02;
        }

        // 钳制到机械可达范围
        targetYaw = Math.max(-yawRange, Math.min(yawRange, yaw));

        double targetTicks = yawZeroTicks + targetYaw * tickPerRad;
        // 本帧唯一的硬件采样点：编码器只读一次，getter 与遥测复用缓存
        double currentTicks = motor.getCurrentPosition();
        cachedTicks = currentTicks;
        cachedTargetTicks = targetTicks;

        // 每帧刷新参数，使 Dashboard 上的改动即时生效
        slot.withKP(kP).withKI(kI).withKD(kD)
                .withMaxI(maxI).withIZone(iZone)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax);
        controller.resetSlot(slot);

        // PIDSVA 位置闭环：SVA 前馈的参考速度/加速度由控制器内部按 setpoint 变化率推导（见 PID/Guide.md）
        power = controller.calculate(targetTicks, currentTicks, dt, false);
        motor.setPower(power);

        onTarget = Math.abs(targetTicks - currentTicks) < onTargetTolerance * Math.abs(tickPerRad);
        return onTarget;
    }

    /** 停止平转（功率归零并复位控制器状态），不改变机械位置 */
    public void stop() {
        motor.setPower(0);
        power = 0;
        onTarget = false;
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

    /** @return 最近一次下发的功率 */
    public double getPower() {
        return power;
    }

    /** 输出遥测（只读 update() 缓存，调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        telemetry.addData("Yaw target (deg)", "%.1f", Math.toDegrees(targetYaw));
        telemetry.addData("Yaw current (deg)", "%.1f", Math.toDegrees(getCurrentYaw()));
        telemetry.addData("Yaw error (deg)", "%.1f", Math.toDegrees(targetYaw - getCurrentYaw()));
        telemetry.addData("Yaw power", "%.3f", power);
        telemetry.addData("Yaw on target", onTarget);
        telemetry.addData("Yaw ticks", "%.0f / %.0f", cachedTicks, cachedTargetTicks);
    }
}