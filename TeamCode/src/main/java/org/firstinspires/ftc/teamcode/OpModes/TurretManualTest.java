package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Controllers.Turret.YawController;

/**
 * 炮台手动控制台：用二操完整驱动 {@link Turret}（平转 + 两个发射器），
 * 用于整体联调与手感确认。
 *
 * <p><b>手柄映射（二操）</b>
 * <ul>
 *   <li>左摇杆 左右 → yaw 平转（映射到 {@code [-yawRange, +yawRange]}；
 *       约定"推右 → 顺时针（负 yaw）"）</li>
 *   <li>小球 / 大球仰角舵机位置 → 由 Dashboard 的 {@link #targetPitchSmall} /
 *       {@link #targetPitchBig} 直接设置，手柄不控制仰角</li>
 *   <li>方向键 上 / 下 → 大球飞轮目标转速 ±{@link #speedStep}，范围 0 至
 *       {@code Shooter.BIG_PARAMS.speedMax}</li>
 *   <li>方向键 右 / 左 → 小球飞轮目标转速 ±{@link #speedStep}，范围 0 至
 *       {@code Shooter.SMALL_PARAMS.speedMax}</li>
 *   <li>左 bumper → 小球扳机（按住释放）</li>
 *   <li>右 bumper → 大球扳机（按住释放）</li>
 *   <li>右扳机 RT → intake 电机功率（0~1，直通 {@code setPower}，不做闭环）</li>
 * </ul>
 * 方向键为"按下即走一步，长按每 {@link #repeatPeriodMs} 毫秒再走一步"。
 *
 * <p>遥测中的 {@code Fire ready big/small} 为 {@link Turret#update} 的返回值，
 * 即对应发射器当前是否具备发射条件（飞轮转速 + 俯仰 + 平转均到位）。
 */
@Config
@TeleOp(name = "TurretManualTest", group = "Tests")
public class TurretManualTest extends LinearOpMode {

    /** 每次按键的转速增量 (tick/s) */
    public static int speedStep = 100;
    /** 长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 120;

    /** 大球仰角舵机目标位置 [0,1]，由 Dashboard 直接设置 */
    public static double targetPitchBig = 0.5;
    /** 小球仰角舵机目标位置 [0,1]，由 Dashboard 直接设置 */
    public static double targetPitchSmall = 0.5;

    /** intake 电机在 Robot Configuration 中的名称（改动后需重新 Init） */
    public static String intakeMotorName = "sweeperMotor";

    private Turret turret;

    /** intake 电机：由二操 RT 直通功率控制 */
    private DcMotorEx intakeMotor;

    private double targetYaw = 0;
    private int targetSpeedBig = 0;
    private int targetSpeedSmall = 0;

    private final DpadStepper vertical = new DpadStepper();
    private final DpadStepper horizontal = new DpadStepper();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap, telemetry);
        turret.stop();

        // ---- intake 电机：功率直通（不做闭环，参照 IntakeTest 的 POWER 模式） ----
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        telemetry.addLine("GP2 LstickX: yaw | pitch from Dashboard (targetPitchBig/Small)");
        telemetry.addLine("GP2 Dpad U/D: big speed | Dpad L/R: small speed");
        telemetry.addLine("GP2 LB: small trigger | RB: big trigger");
        telemetry.addLine("GP2 RT: intake power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- 摇杆：仰角改由 Dashboard 设置，手柄只控制 yaw ----
            double pitchSmall = targetPitchSmall;
            double pitchBig = targetPitchBig;
            // 推右（axis = +1）→ 顺时针，对应负 yaw
            targetYaw = -gamepad2.left_stick_x * YawController.yawRange;

            // ---- 方向键步进飞轮目标转速 ----
            int v = vertical.step(gamepad2.dpad_down, gamepad2.dpad_up, repeatPeriodMs);
            int h = horizontal.step(gamepad2.dpad_left, gamepad2.dpad_right, repeatPeriodMs);
            targetSpeedBig = clamp(targetSpeedBig + v * speedStep, 0, Shooter.BIG_PARAMS.flywheel.speedMax);
            targetSpeedSmall = clamp(targetSpeedSmall + h * speedStep, 0, Shooter.SMALL_PARAMS.flywheel.speedMax);

            // ---- 扳机：按住释放 ----
            boolean triggerBig = gamepad2.right_bumper;
            boolean triggerSmall = gamepad2.left_bumper;

            // ---- intake：RT 直通功率（0~1） ----
            intakeMotor.setPower(-gamepad2.right_trigger);

            // ---- 下发炮台目标 ----
            Turret.FireReady ready = turret.update(targetYaw, pitchBig, pitchSmall,
                    targetSpeedBig, targetSpeedSmall, triggerBig, triggerSmall);

            // ---- 遥测 ----
            turret.setTelemetry();
            telemetry.addLine("--- manual targets ---");
            telemetry.addData("Target yaw (deg)", "%.1f", Math.toDegrees(targetYaw));
            telemetry.addData("Target speed big (tick/s)", targetSpeedBig);
            telemetry.addData("Target speed small (tick/s)", targetSpeedSmall);
            telemetry.addData("Target pitch big", "%.3f", targetPitchBig);
            telemetry.addData("Target pitch small", "%.3f", targetPitchSmall);
            telemetry.addData("Fire ready big", ready.big);
            telemetry.addData("Fire ready small", ready.small);
            telemetry.addData("speedStep", speedStep);
            telemetry.addData("Intake power (RT)", "%.3f", intakeMotor.getPower());
            telemetry.addData("Intake velocity (tick/s)", "%.0f", intakeMotor.getVelocity());
            telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
            telemetry.update();
        }

        intakeMotor.setPower(0);
        turret.stop();
    }

    /** 取各 Hub 电压最小值（近似电池电压） */
    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            min = Math.min(min, sensor.getVoltage());
        }
        return min;
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * 方向键"按下即走一步、长按按周期连续步进"的辅助器。
     * 松开或换向时计时复位，因此换向立刻响应。
     */
    private static class DpadStepper {
        private int lastDir = 0;
        private long nextTime = 0;

        /**
         * @param negative 负方向键是否按下
         * @param positive 正方向键是否按下
         * @param periodMs 长按连续步进周期（毫秒）
         * @return 本帧应施加的步进方向 (-1 / 0 / +1)
         */
        int step(boolean negative, boolean positive, int periodMs) {
            int dir = (positive ? 1 : 0) - (negative ? 1 : 0);
            if (dir == 0) {
                lastDir = 0;
                return 0;
            }
            long now = System.currentTimeMillis();
            if (dir != lastDir || now >= nextTime) {
                lastDir = dir;
                nextTime = now + periodMs;
                return dir;
            }
            return 0;
        }
    }
}
