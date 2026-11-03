package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BallCounter;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BreakBeamSensor;

/**
 * 球道计数 + 容量保护联调程序（BallCounter + Sweeper）。
 *
 * <p>验证四件事：
 * <ol>
 *   <li>球道自动计数：球逐个挡光，Count 每次 +1；球停在光电门里不动只 +1；</li>
 *   <li>达到容量停止吸球：Count 到 capacity 后 Intake Locked = true，
 *       按住 LB 吸球电机目标仍为 0；出球/送球不受影响；</li>
 *   <li>炮台发射完毕清空：按 B 模拟一次“发射完毕”，Count 归零、锁定解除；</li>
 *   <li>操作手手动清空：按 Y，Count 归零、锁定解除。</li>
 * </ol>
 *
 * <p><b>与比赛主程序的区别</b>：本 OpMode 不初始化 RobotPosition，因此显式调用
 * {@code sweeper.setIntakeLocked(...)} 模拟“球道已满”。比赛主程序由
 * {@code RobotPosition.isBallLaneFull()} 自动完成同一判断（架构 PPT 的做法）。
 *
 * <p><b>手柄</b>（gamepad1）：LB = 吸球，RB = 出球，Y = 手动清空，
 * B = 模拟炮台发射完毕。
 *
 * <p><b>调参</b>：FTC Dashboard → Config → BallLaneTestOp
 * <ul>
 *   <li>{@code threshold}：归一化遮挡阈值 [0,1]；</li>
 *   <li>{@code alpha}：EMA 平滑系数 (0,1]；</li>
 *   <li>{@code capacity}：球道容量，默认取 {@link HypParams#BALL_CAPACITY}；</li>
 * </ul>
 *
 * <p>实机验证步骤：
 * <pre>
 *   1. 空球道，按 Y 清空 → Count 0；
 *   2. 逐个放入 1~4 个球，每个球只应 +1；第 4 个后 Full/Locked 变 true；
 *   3. 保持按住 LB，第 4 个球后吸球电机应停转（目标 0）；
 *   4. 按 B（或接入真实炮台后由 Turret 自动调用
 *      {@code RobotPosition.resetBallCount()} 触发）→ Count 0；
 *   5. 再按 LB，应能重新吸球。
 * </pre>
 */
@Config
@TeleOp(name = "BallLaneTestOp", group = "Tests")
public class BallLaneTestOp extends LinearOpMode {

    /** 模拟口在机器人配置中的设备名（硬件参数，改动需重新 init） */
    private static final String DEVICE_NAME = "breakBeam";

    // ================= FTC Dashboard 可调参数 =================

    /** 归一化遮挡阈值 [0, 1] */
    public static double threshold = 0.5;

    /** EMA 平滑系数 (0, 1] */
    public static double alpha = 0.8;

    /** 球道容量；改小会把当前计数钳制到新容量 */
    public static int capacity = HypParams.BALL_CAPACITY;

    private BreakBeamSensor beam;
    private BallCounter counter;
    private Sweeper sweeper;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        beam = new BreakBeamSensor(hardwareMap, DEVICE_NAME);
        beam.setThreshold(clamp01(threshold));
        beam.setAlpha(Math.max(0.01, Math.min(1.0, alpha)));
        counter = new BallCounter(beam, Math.max(1, capacity));
        sweeper = new Sweeper(hardwareMap, telemetry);

        telemetry.addData("Device", DEVICE_NAME);
        telemetry.addData("Capacity", counter.getCapacity());
        telemetry.addLine("LB: Eat | RB: Output | Y: Manual clear | B: Simulate turret shot complete");
        telemetry.addLine("Tune threshold/alpha/capacity in FTC Dashboard > Config");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- 清空：手动 / 模拟炮台发射完毕 ----
            if (gamepad1.yWasReleased()) {
                counter.reset();
            }
            if (gamepad1.bWasReleased()) {
                counter.onShotComplete();
            }

            // ---- Dashboard 参数每帧生效 ----
            beam.setThreshold(clamp01(threshold));
            beam.setAlpha(Math.max(0.01, Math.min(1.0, alpha)));
            counter.setCapacity(Math.max(1, capacity));

            // ---- 先计数，再根据 isFull() 决定是否允许吸球 ----
            boolean outputRequested = gamepad1.right_bumper;
            counter.update(outputRequested
                    ? BallCounter.MODE_OUTPUT : BallCounter.MODE_INTAKE);
            sweeper.setIntakeLocked(counter.isFull());

            if (gamepad1.left_bumper) {
                sweeper.setEat();
            }
            if (outputRequested) {
                sweeper.setOutput();
            }

            sweeper.update();

            // ---- 遥测 ----
            telemetry.addData("Ball Count", "%d / %d", counter.getCount(), counter.getCapacity());
            telemetry.addData("Full", counter.isFull());
            telemetry.addData("Remaining", counter.getRemaining());
            telemetry.addData("Intake Locked", sweeper.isIntakeLocked());
            telemetry.addData("Blocked", beam.isBlocked() ? 1 : 0);
            telemetry.addData("Raw Voltage", "%.3f", beam.getRawVoltage());
            telemetry.addData("Filtered Voltage", "%.3f", beam.getVoltage());
            telemetry.addData("Sweeper target (tick/s)", sweeper.getTargetVelocity());
            telemetry.update();
        }

        sweeper.setStop();
        sweeper.update();
    }

    /** 把数值限制到 [0, 1] */
    private static double clamp01(double value) {
        if (value < 0) {
            return 0;
        }
        if (value > 1) {
            return 1;
        }
        return value;
    }
}
