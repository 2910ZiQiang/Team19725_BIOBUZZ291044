package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.utility.DpadStepper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

/**
 * 飞轮速度环整定台：用方向键给定阶跃式的目标转速，观察 {@link Shooter} 速度环的
 * 上升、超调与稳态误差。
 *
 * <p>本 OpMode 直接复用真实代码路径（{@link Shooter#update} 与
 * {@link Shooter#BIG_PARAMS} / {@link Shooter#SMALL_PARAMS}），因此整定得到的
 * 参数可原样使用于正式程序。
 *
 * <p><b>手柄映射（一操）</b>
 * <ul>
 *   <li>方向键 上 / 下 → 大球飞轮目标转速 ±{@link #speedStep} (tick/s)</li>
 *   <li>方向键 右 / 左 → 小球飞轮目标转速 ±{@link #speedStep} (tick/s)</li>
 *   <li>A 键 → 两个飞轮目标转速归零</li>
 * </ul>
 * 目标转速被钳制到 {@code [0, Params.speedMax]}（分别取大球/小球各自的 speedMax）。
 * 按下即走一步，长按每 {@link #repeatPeriodMs} 毫秒再走一步。
 *
 * <p>本测试不调俯仰，俯仰舵机固定在下发的 {@link #holdPitch} 位置，扳机始终关闭。
 *
 * <p><b>整定</b>：在 FTC Dashboard 的 {@code Shooter} 分组中实时修改
 * kP / kI / kD / kS / kV / iZone，并把遥测中的 target / current / error
 * 加入 plot，观察阶跃响应。
 */
@Config
@TeleOp(name = "FlywheelTest", group = "Tests")
public class FlywheelTest extends LinearOpMode {

    // ---- 设备名（与 Turret.java 保持一致） ----
    private static final String BIG_MOTOR = "shooterBig";
    private static final String BIG_PITCH_SERVO = "pitchBig";
    private static final String BIG_TRIGGER_SERVO = "triggerBig";
    private static final String SMALL_MOTOR = "shooterSmall";
    private static final String SMALL_PITCH_SERVO = "pitchSmall";
    private static final String SMALL_TRIGGER_SERVO = "triggerSmall";

    /** 每次按键的转速增量 (tick/s) */
    public static int speedStep = 100;
    /** 长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 120;
    /** 俯仰舵机固定位置（本测试不调俯仰） */
    public static double holdPitch = 0.5;

    private Shooter shooterBig;
    private Shooter shooterSmall;

    /** 当前目标转速 (tick/s)，钳制前保留用户意图，便于在限幅处回退 */
    private int targetBig = 0;
    private int targetSmall = 0;

    private final DpadStepper vertical = new DpadStepper();
    private final DpadStepper horizontal = new DpadStepper();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooterBig = new Shooter(hardwareMap, telemetry,
                BIG_MOTOR, BIG_PITCH_SERVO, BIG_TRIGGER_SERVO, Shooter.BIG_PARAMS);
        shooterSmall = new Shooter(hardwareMap, telemetry,
                SMALL_MOTOR, SMALL_PITCH_SERVO, SMALL_TRIGGER_SERVO, Shooter.SMALL_PARAMS);
        shooterBig.stop();
        shooterSmall.stop();

        telemetry.addLine("GP1 Dpad U/D: big speed +/- | Dpad L/R: small speed +/- | A: stop");
        telemetry.addLine("PID params: Dashboard group Shooter");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- A 键：两个飞轮同时停转 ----
            if (gamepad1.a) {
                targetBig = 0;
                targetSmall = 0;
            }

            // ---- 方向键步进目标转速 ----
            int v = vertical.step(gamepad1.dpad_down, gamepad1.dpad_up, repeatPeriodMs);
            int h = horizontal.step(gamepad1.dpad_left, gamepad1.dpad_right, repeatPeriodMs);
            targetBig = clamp(targetBig + v * speedStep, 0, Shooter.BIG_PARAMS.flywheel.speedMax);
            targetSmall = clamp(targetSmall + h * speedStep, 0, Shooter.SMALL_PARAMS.flywheel.speedMax);

            // ---- 速度闭环（扳机始终关闭，俯仰固定） ----
            shooterBig.update(targetBig, holdPitch, false);
            shooterSmall.update(targetSmall, holdPitch, false);

            // ---- 遥测 ----
            telemetry.addData("Big target (tick/s)", targetBig);
            telemetry.addData("Big current (tick/s)", "%.0f", shooterBig.getVelocity());
            telemetry.addData("Big error (tick/s)", "%.0f", targetBig - shooterBig.getVelocity());
            telemetry.addData("Big power", "%.4f", shooterBig.getPower());
            telemetry.addData("Small target (tick/s)", targetSmall);
            telemetry.addData("Small current (tick/s)", "%.0f", shooterSmall.getVelocity());
            telemetry.addData("Small error (tick/s)", "%.0f", targetSmall - shooterSmall.getVelocity());
            telemetry.addData("Small power", "%.4f", shooterSmall.getPower());
            telemetry.addData("speedStep", speedStep);
            telemetry.update();
        }

        shooterBig.stop();
        shooterSmall.stop();
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
