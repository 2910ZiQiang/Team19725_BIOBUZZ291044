package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Processors.Sensors.BallCounter;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BreakBeamSensor;
/**
 * BallCounter 小球计数器测试程序（基于对射光电门，模拟信号接口）。
 *
 * <p>BallCounter 只负责两件事：<b>上升沿计数</b>（0 = 进入 +1，1 = 出球 -1）
 * 与<b>被 reset 时清零</b>。满球停吸（BallMax）由 {@code Sweeper} 判断，
 * 发射完毕清空由 {@code Turret} 调用 {@code RobotPosition.resetBallCount()}，
 * 本测试程序因此不再控制 intake 电机，只验证计数与清零。
 *
 * <p><b>手柄</b>
 * <ul>
 *   <li>A → 切换 INTAKE(0) / OUTPUT(1) 计数方向</li>
 *   <li>B → 手动清空计数（等价于 reset）</li>
 * </ul>
 *
 * <p><b>实时显示</b>
 *   External State   本帧传入的计数方向（0 = 进入(intake)，1 = 出球(output)）
 *   Ball Count       累计计数的小球个数（始终 ≥ 0）
 *   New Ball         本帧是否出现上升沿（1 = 是）
 *   Blocked          光电门遮挡判定结果（1 = 遮挡，0 = 未遮挡）
 *   Raw Voltage      归一化原始电压（未滤波，= 伏特 / 3.3）
 *   Filtered Voltage 归一化滤波电压（EMA 输出）
 *   Threshold        当前归一化阈值
 *   Alpha            当前 EMA 平滑系数
 *
 * <p><b>调参方式</b>：FTC Dashboard → Config → BallCounterTestOp
 *   threshold       归一化遮挡阈值 [0, 1]，滤波电压低于该值即判定为遮挡
 *   alpha           EMA 平滑系数 (0, 1]，越接近 1 响应越快、抗噪越差
 *   output          计数方向：false = 进入模式（intake，+1），true = 出球模式（output，-1）
 *   resetCounter    置 true 会清零一次计数并重置滤波器（仅上升沿生效）
 *
 * <p><b>验证步骤</b>
 * <ol>
 *   <li>INTAKE 模式下让小球逐个挡过光电门，每个球应只使 Ball Count +1；</li>
 *   <li>让小球停在光电门中不动，Ball Count 应只 +1 且之后不再增加；</li>
 *   <li>切到 OUTPUT 模式让小球逐个反向经过，每个球应使 Ball Count -1（不会为负）；</li>
 *   <li>任意时刻按 B 或勾选 resetCounter：Ball Count 立即清零。
 * </ol>
 *
 * <p><b>注意</b>：BallCounter.update() 内部已调用 BreakBeamSensor.update()，
 * 本程序中不要再单独调用 beam.update()，否则滤波器每帧会被推进两次。
 *
 * <p><b>校准建议</b>：先在"有遮挡 / 无遮挡"两种情况下观察 Raw Voltage，
 * 确认未遮挡为高电压、遮挡为低电压；再按抖动情况调整 alpha
 * （0.5 附近抗抖更强，0.9 附近响应更快）。
 */
@Config
@TeleOp(name = "BallCounterTestOp", group = "Tests")
public class BallCounterTestOp extends LinearOpMode {

    /** 模拟口在机器人配置中的设备名（硬件参数，改动需重新 init） */
    private static final String DEVICE_NAME = "breakBeam";


    // ================= FTC Dashboard 可调参数 =================

    /** 归一化遮挡阈值 [0, 1]，滤波电压低于它即判定为遮挡 */
    public static double threshold = 0.5;

    /** EMA 平滑系数 (0, 1] */
    public static double alpha = 0.8;

    /** 计数方向：false = 进入模式（intake，+1），true = 出球模式（output，-1） */
    public static boolean output = false;

    /** 置 true 时清零一次计数并重置滤波器（仅上升沿生效） */
    public static boolean resetCounter = false;


    /** 上一次的 resetCounter 值，用于检测上升沿 */
    private boolean lastResetCounter = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 用已创建的光电门构造计数器，便于本测试程序读取原始电压
        BreakBeamSensor beam = new BreakBeamSensor(hardwareMap, DEVICE_NAME);
        BallCounter counter = new BallCounter(beam);

        telemetry.addData("Device", DEVICE_NAME);
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("A: INTAKE/OUTPUT | B: clear count");
        telemetry.addLine("Tune threshold/alpha/output/resetCounter in Dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // A 键切换计数方向
            if (gamepad1.aWasPressed()) {
                output = !output;
            }

            // 采样 + 上升沿计数（功能 1）；B 键或 Dashboard 上升沿清零
            int externalState = output ? BallCounter.MODE_OUTPUT : BallCounter.MODE_INTAKE;
            boolean newBall = counter.update(externalState);
            if (gamepad1.bWasPressed() || (resetCounter && !lastResetCounter)) {
                counter.reset();
            }
            lastResetCounter = resetCounter;

            // 每帧应用 Dashboard 参数（带范围保护，避免非法值抛异常）
            beam.setThreshold(clamp01(threshold));                  // threshold ∈ [0, 1]
            beam.setAlpha(Math.max(0.01, Math.min(1.0, alpha)));     // alpha ∈ (0, 1]

            // ---- 遥测 ----
            telemetry.addData("External State", externalState);
            telemetry.addData("Mode", output ? "OUTPUT (1)" : "INTAKE (0)");
            telemetry.addData("Ball Count", counter.getCount());
            telemetry.addData("New Ball", newBall ? 1 : 0);
            telemetry.addData("Blocked", beam.isBlocked() ? 1 : 0);
            telemetry.addData("Raw Voltage", "%.3f", beam.getRawVoltage());
            telemetry.addData("Filtered Voltage", "%.3f", beam.getVoltage());
            telemetry.addData("Threshold", "%.3f", beam.getThreshold());
            telemetry.addData("Alpha", "%.3f", beam.getAlpha());
            telemetry.update();
        }
    }

    /** 把数值限制到 [lo, hi] */
    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }

    /** 把数值限制到 [0, 1] */
    private static double clamp01(double value) {
        return clamp(value, 0.0, 1.0);
    }
}
