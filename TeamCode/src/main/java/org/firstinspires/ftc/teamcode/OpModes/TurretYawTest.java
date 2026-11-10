package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.utility.DpadStepper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Turret.YawController;

/**
 * 平转机构 PID 整定台：用方向键给定阶跃式的目标角度，观察 {@link YawController}
 * 位置闭环的上升、超调与稳态误差。
 *
 * <p><b>操作</b>
 * <ul>
 *   <li>一操 方向键 右 / 左 → 目标平转角 ±{@link #angleStepDeg} 度
 *       （按下即走一步，长按每 {@link #repeatPeriodMs} 毫秒再走一步）</li>
 *   <li>一操 A 键 → 目标角度归零</li>
 *   <li>一操 B 键 → 复位控制器积分/微分状态（不改变目标与机械位置）</li>
 * </ul>
 *
 * <p><b>整定</b>：在 FTC Dashboard 的 {@code YawController} 分组中实时修改
 * kP / kI / kD / kS / kV / iZone，并把遥测中的 target / current / error / power
 * 加入 plot，观察阶跃响应。
 *
 * <p>注意 {@link YawController#yawRange} 会钳制目标角度；需要更大行程时先在
 * Dashboard 上把 {@code yawRange} 调大。目标角度会与钳制后的实际目标保持同步，
 * 因此到达限位后不会"空攒"步数。
 */
@Config
@TeleOp(name = "TurretYawTest", group = "Tests")
public class TurretYawTest extends LinearOpMode {

    /** 每次按键的角度增量（度） */
    public static double angleStepDeg = 10.0;
    /** 长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 150;

    private YawController yawController;
    /** 当前目标平转角（弧度）；每帧与钳制后的实际目标同步 */
    private double targetYaw = 0;

    private final DpadStepper stepper = new DpadStepper();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        yawController = new YawController(hardwareMap, telemetry);

        telemetry.addLine("GP1 Dpad L/R: target yaw -/+ step | A: target=0 | B: reset controller");
        telemetry.addData("angleStepDeg", angleStepDeg);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- 方向键步进目标角度 ----
            int dir = stepper.step(gamepad1.dpad_left, gamepad1.dpad_right, repeatPeriodMs);
            targetYaw += dir * Math.toRadians(angleStepDeg);

            // ---- 清零 / 复位 ----
            if (gamepad1.aWasPressed()) {
                targetYaw = 0;
            }
            if (gamepad1.bWasPressed()) {
                yawController.stop();
            }

            // ---- 位置闭环 ----
            yawController.update(targetYaw);
            // 与钳制后的实际目标同步，避免在限位处持续累积步数
            targetYaw = yawController.getTargetYaw();

            // ---- 遥测 ----
            telemetry.addData("Yaw target (deg)", "%.2f", Math.toDegrees(targetYaw));
            telemetry.addData("Yaw current (deg)", "%.2f", Math.toDegrees(yawController.getCurrentYaw()));
            telemetry.addData("Yaw error (deg)", "%.2f",
                    Math.toDegrees(targetYaw - yawController.getCurrentYaw()));
            telemetry.addData("Yaw power", "%.4f", yawController.getPower());
            telemetry.addData("Yaw on target", yawController.isOnTarget());
            telemetry.addData("Yaw ticks", yawController.motor.getCurrentPosition());
            telemetry.addData("angleStepDeg", angleStepDeg);
            telemetry.update();
        }

        yawController.stop();
    }

    /** 方向键步进器见 {@link DpadStepper} */
}