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
 * 每帧调用 counter.update(externalState) 采样并做上升沿检测计数，实时显示：
 *   External State   本帧传入的外部信息（0 = 进入(intake)，1 = 出球(output)）
 *   Mode             当前生效的计数方向（INTAKE (0) / OUTPUT (1)）
 *   Ball Count       累计计数的小球个数
 *   New Ball         本帧是否检测到新小球（上升沿，1 = 是）
 *   Blocked          光电门遮挡判定结果（1 = 遮挡，0 = 未遮挡）
 *   Raw Voltage      归一化原始电压（未滤波，= 伏特 / 3.3）
 *   Filtered Voltage 归一化滤波电压（EMA 输出）
 *   Threshold        当前归一化阈值
 *   Alpha            当前 EMA 平滑系数
 *
 * 调参方式：FTC Dashboard → Config → BallCounterTestOp
 *   threshold       归一化遮挡阈值 [0, 1]，滤波电压低于该值即判定为遮挡
 *   alpha           EMA 平滑系数 (0, 1]，越接近 1 响应越快、抗噪越差
 *   output          计数方向：false = 进入模式（intake，+1），true = 出球模式（output，-1）
 *   resetCounter    置 true 会清零一次计数并重置滤波器（仅上升沿生效）
 *
 * 验证步骤：
 *   1. 让小球逐个挡过光电门，每个球应只使 Ball Count +1；
 *   2. 让小球停在光电门中不动，Ball Count 应只 +1 且之后不再增加，
 *      Blocked 保持为 1，New Ball 仅在挡光的那一帧为 1；
 *   3. 球离开（Blocked 回到 0）后再挡下一个球，应再次 +1；
 *   4. 置 output = true 切到出球模式后，每挡一个球 Ball Count 应 -1，
 *      减到 0 后继续挡球不会变成负数。
 *
 * 注意：BallCounter.update() 内部已调用 BreakBeamSensor.update()，
 *      本程序中不要再单独调用 beam.update()，否则滤波器每帧会被推进两次。
 *
 * 校准建议：先在"有遮挡 / 无遮挡"两种情况下观察 Raw Voltage，
 *   确认未遮挡为高电压、遮挡为低电压；再按抖动情况调整 alpha
 *   （0.5 附近抗抖更强，0.9 附近响应更快）。
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

    /** 置 true 时清零一次计数并重置滤波器 */
    public static boolean resetCounter = false;


    /** 上一次的 resetCounter 值，用于检测上升沿 */
    private boolean lastResetCounter = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BreakBeamSensor beam = new BreakBeamSensor(hardwareMap, DEVICE_NAME);
        BallCounter counter = new BallCounter(beam);

        telemetry.addData("Device", DEVICE_NAME);
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Tune threshold/alpha/output/resetCounter in FTC Dashboard > Config");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 采样 + EMA 滤波 + 上升沿检测计数（必须每帧调用）
            // 外部信息：0 = 进入模式（intake，+1），1 = 出球模式（output，-1）
            int externalState = output ? BallCounter.MODE_OUTPUT : BallCounter.MODE_INTAKE;
            boolean newBall = counter.update(externalState);

            // 上升沿清零计数（同时重置底层滤波器）
            if (resetCounter && !lastResetCounter) {
                counter.reset();
            }
            lastResetCounter = resetCounter;

            // 每帧应用 Dashboard 参数（带范围保护，避免非法值抛异常）
            beam.setThreshold(clamp01(threshold));                  // threshold ∈ [0, 1]
            beam.setAlpha(Math.max(0.01, Math.min(1.0, alpha)));     // alpha ∈ (0, 1]

            // ---- 遥测 ----
            telemetry.addData("External State", externalState);
            telemetry.addData("Ball Count", counter.getCount());
            telemetry.addData("Mode", output ? "OUTPUT (1)" : "INTAKE (0)");
            telemetry.addData("New Ball", newBall ? 1 : 0);
            telemetry.addData("Blocked", beam.isBlocked() ? 1 : 0);
            telemetry.addData("Raw Voltage", "%.3f", beam.getRawVoltage());
            telemetry.addData("Filtered Voltage", "%.3f", beam.getVoltage());
            telemetry.addData("Threshold", "%.3f", beam.getThreshold());
            telemetry.addData("Alpha", "%.3f", beam.getAlpha());
            telemetry.update();
        }
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
