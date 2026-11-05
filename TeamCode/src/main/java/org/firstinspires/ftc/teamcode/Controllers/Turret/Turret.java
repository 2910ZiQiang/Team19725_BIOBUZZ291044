package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
 * <p><b>todo</b>：未来实现自动瞄准——重载 update，输入车辆位姿与是否允许发射，
 * 由两个发射器的相对位置反解平转角与射程，再按标定的射程-转速模型解出飞轮速度，
 * 平转角取两个发射器所需角度的平均（指向劣角），并在两机构均到位时自动释放扳机。
 */
@Config
public class Turret {

    // ==================== 超参数 ====================

    /**
     * 大球发射器中心在地面的投影，在相对坐标系中的坐标（英寸）。
     * 相对坐标系（RoadRunner）：x 指向车头，y 指向车体左侧。
     * 供自动瞄准解算发射器实际位置使用（当前尚未使用）。
     */
    public static Vector2D RelPosBig = new Vector2D(0, 0);

    /**
     * 小球发射器中心在地面的投影，在相对坐标系中的坐标（英寸）。
     * 供自动瞄准解算发射器实际位置使用（当前尚未使用）。
     */
    public static Vector2D RelPosSmall = new Vector2D(0, 0);

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
        boolean yawReady = yawController.update(yaw);
        boolean bigReady = shooterBig.update(speedBig, pitchBig, triggerBig);
        boolean smallReady = shooterSmall.update(speedSmall, pitchSmall, triggerSmall);

        return new FireReady(yawReady && bigReady, yawReady && smallReady);
    }

    /** 停止炮台：平转停止输出，两个发射器停转并关闭扳机 */
    public void stop() {
        yawController.stop();
        shooterBig.stop();
        shooterSmall.stop();
    }

    /** 输出遥测（调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        yawController.setTelemetry();
        shooterBig.setTelemetry();
        shooterSmall.setTelemetry();
    }
}