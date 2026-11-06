package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * 第 4 代（当前主流）飞轮整定/验收：<b>闭环验证</b>，参数来自 {@link Gen4Params}。
 *
 * <p>与 Tune6.1/6.5/6.6/6.7 的关系：那些程序负责「从开环数据辨识参数」；
 * 本程序负责「把参数放进完整闭环后，按 {@code doc/现在的控制系统.md} §2.12 的判据逐条核对」。
 * 之所以放这里而不是 OpModes/Tuning，是因为它要用 {@link Gen4Params}（Dashboard 可改）
 * 与多代对比共用同一条驱动通路。
 *
 * <p>两个阶段：
 * <ol>
 *   <li><b>稳态验收</b>：对 {@link #targets} 每个转速闭环 → 量 {@code v_sd}、{@code |误差|％}、
 *       {@code duty_sd}、{@code |I 积分|}、{@code settled 占比}。</li>
 *   <li><b>掉速恢复</b>：在 {@link #dipTarget} 上闭环，用 {@link #dipDuty} 压 {@link #dipSec} 秒
 *       （模拟发射掉速），量跌幅、恢复时间、过冲，并检查恢复期 {@code isSettled()} 是否翻 false。</li>
 * </ol>
 *
 * <p>判据参考（§2.12）：②{@code v_sd ≤ 0.6} 台阶、{@code |误差| ≤ 0.12%}、{@code duty_sd ≤ 0.025}；
 * ③{@code |∫| < 0.04/ki}；④跌幅 270~290、恢复中位 {@code ≤ 0.43 s}、过冲 {@code ≤ +20}；
 * ⑤稳态 0.1 s 内 settled、恢复期 false。①{@code kp×quantStep ≤ 0.05} 在报告里直接算。
 */
@TeleOp(name = "TuneGen4 闭环验收", group = "PastCompare")
public class TuneGen4Op extends LinearOpMode {

    // ------------------------------------------------------------ @Config

    /** 要验收的转速点 */
    public static double[] targets = {1600, 1900, 2250};
    /** 每个点的稳态判读窗口 (s) */
    public static double steadyWindowSec = 1.0;
    /** 每个点先跑多久再判读 (s) */
    public static double settleSec = 2.0;
    /** 每个点最长等待 (s) */
    public static double pointTimeoutSec = 8.0;

    /** 掉速阶段的目标转速 */
    public static double dipTarget = 1900;
    /** 掉速阶段：压到的开环占空比 */
    public static double dipDuty = 0.15;
    /** 掉速持续 (s) */
    public static double dipSec = 0.35;
    /** 掉速后观察 (s) */
    public static double recoverySec = 2.0;

    public static String motorName = "shooter";
    public static boolean motorReversed = true;
    /** 是否发布扁平数值遥测（由 FTC Dashboard 自带 Logging 记录） */
    public static boolean dashboardLog = true;
    public static double logPeriodMs = 10;

    static final String CSV_HEADER =
            "t,target,velocity,error,duty,ff,p,i,iActive,settled,vbus,vscale,phase";

    // ------------------------------------------------------------ state

    private double lastLogSec = -1;
    private PastFlywheelDriver driver;
    private PastCalculator.Gen4 calculator;

    private final StringBuilder sweepReport = new StringBuilder();
    private double dipDepth = 0;
    private double dipRecovery = Double.NaN;
    private double dipOvershoot = 0;
    private boolean sawUnsettled = false;
    private double lastLogT = -1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("TuneGen4（闭环验收，参数=Gen4Params）");
        telemetry.addLine(String.format(Locale.US, "点=%d 个  dip=%.2f/%.2fs  kp*step=%.4f%s",
                targets.length, dipDuty, dipSec,
                Gen4Params.kp * Gen4Params.quantStep,
                Gen4Params.kp * Gen4Params.quantStep > 0.05 ? "  <-- HIGH" : ""));
        telemetry.addLine("X 中止。");

        calculator = new PastCalculator.Gen4();
        try {
            driver = new PastFlywheelDriver(hardwareMap, motorName, motorReversed, calculator);
        } catch (RuntimeException e) {
            telemetry.addLine("!! 找不到电机 " + motorName + "：" + e.getMessage());
            telemetry.update();
            sleep(3000);
            return;
        }

        waitForStart();
        if (isStopRequested()) {
            return;
        }


        runSweep();
        if (opModeIsActive() && !gamepad1.x) {
            runDip();
        }
        if (opModeIsActive()) {
            report();
        }

        driver.setOpenLoopPower(0);
    }

    // ------------------------------------------------------------ 阶段 1

    private void runSweep() {
        for (double target : targets) {
            if (!opModeIsActive() || gamepad1.x) {
                return;
            }
            driver.resetController();

            // 先稳定
            double t0 = getRuntime();
            while (opModeIsActive() && !gamepad1.x && getRuntime() - t0 < settleSec) {
                driver.update((int) target);
                logFrame(getRuntime(), target, "settle");
                sleep(5);
            }

            // 再判读
            double start = getRuntime();
            int n = 0;
            double sumV = 0, sumV2 = 0, sumD = 0, sumD2 = 0, sumI = 0, sumAbsE = 0;
            int settledFrames = 0;
            while (opModeIsActive() && !gamepad1.x) {
                driver.update((int) target);
                double t = getRuntime();
                if (t - start > steadyWindowSec || t - start > pointTimeoutSec) {
                    break;
                }
                double v = driver.getVelocity();
                double d = calculator.getDuty();
                n++;
                sumV += v;
                sumV2 += v * v;
                sumD += d;
                sumD2 += d * d;
                sumI += calculator.getIntegral();
                sumAbsE += Math.abs(calculator.getError());
                if (calculator.isSettled()) {
                    settledFrames++;
                }
                logFrame(t, target, "measure");
                telemetry.addData("验收", "ω*=%.0f t=%.2f/%.2f v=%.0f", target, t - start, steadyWindowSec, v);
                telemetry.update();
                sleep(5);
            }

            if (n < 10) {
                continue;
            }
            double meanV = sumV / n;
            double sdV = Math.sqrt(Math.max(0, sumV2 / n - meanV * meanV));
            double meanD = sumD / n;
            double sdD = Math.sqrt(Math.max(0, sumD2 / n - meanD * meanD));
            double meanI = sumI / n;
            double errPct = 100.0 * Math.abs(meanV - target) / target;
            double settledPct = 100.0 * settledFrames / n;

            String line = String.format(Locale.US,
                    "ω*=%.0f  v=%.1f  err=%.3f%%  v_sd=%.2f  duty=%.4f  duty_sd=%.4f  ∫=%.4f  settled=%.0f%%",
                    target, meanV, errPct, sdV, meanD, sdD, meanI, settledPct);
            sweepReport.append(line).append('\n');
            telemetry.addData("meas_target", target);
            telemetry.addData("meas_v_mean", meanV);
            telemetry.addData("meas_err_pct", errPct);
            telemetry.addData("meas_v_sd", sdV);
            telemetry.addData("meas_duty", meanD);
            telemetry.addData("meas_duty_sd", sdD);
            telemetry.addData("meas_integral", meanI);
            telemetry.addData("meas_settled_pct", settledPct);
        }
        driver.setTargetVelocity(0);
        driver.setOpenLoopPower(0);
    }

    // ------------------------------------------------------------ 阶段 2

    private void runDip() {
        driver.resetController();
        double t0 = getRuntime();
        while (opModeIsActive() && !gamepad1.x && getRuntime() - t0 < settleSec) {
            driver.update((int) dipTarget);
            logFrame(getRuntime(), dipTarget, "dip_settle");
            sleep(5);
        }
        if (!opModeIsActive() || gamepad1.x) {
            return;
        }

        // 人为掉速：开环压占空比（不调用 update，否则被闭环覆写）
        driver.setOpenLoopPower(dipDuty);
        double dipStart = getRuntime();
        dipDepth = 0;
        while (opModeIsActive() && !gamepad1.x && getRuntime() - dipStart < dipSec) {
            double v = driver.getVelocity();
            dipDepth = Math.max(dipDepth, dipTarget - v);
            logFrame(getRuntime(), dipTarget, "dip");
            sleep(5);
        }

        // 回到闭环：观察恢复
        double recoverStart = getRuntime();
        boolean recovered = false;
        sawUnsettled = false;
        while (opModeIsActive() && !gamepad1.x && getRuntime() - recoverStart < recoverySec) {
            driver.update((int) dipTarget);
            double e = calculator.getError();
            double v = driver.getVelocity();
            if (!calculator.isSettled()) {
                sawUnsettled = true;
            }
            if (!recovered && Math.abs(e) <= calculator.getErrorBand()) {
                recovered = true;
                dipRecovery = getRuntime() - recoverStart;
            }
            if (recovered) {
                dipOvershoot = Math.max(dipOvershoot, v - dipTarget);
            }
            logFrame(getRuntime(), dipTarget, "recover");
            telemetry.addData("掉速恢复", "跌幅=%.0f 恢复=%.3f", dipDepth, dipRecovery);
            telemetry.update();
            sleep(5);
        }
        driver.setTargetVelocity(0);
        driver.setOpenLoopPower(0);
    }

    // ------------------------------------------------------------ 报告

    private void report() {
        for (String line : sweepReport.toString().split("\n")) {
            telemetry.addLine(line);
        }
        telemetry.addLine("---- 掉速 ----");
        telemetry.addData("跌幅", "%.0f tick/s（目标 270~290）", dipDepth);
        telemetry.addData("恢复时间", Double.isNaN(dipRecovery)
                ? "未恢复" : String.format(Locale.US, "%.3f s（目标 ≤0.43）", dipRecovery));
        telemetry.addData("过冲", "%.0f tick/s（目标 ≤+20）", dipOvershoot);
        telemetry.addData("恢复期 isSettled 曾翻 false", sawUnsettled ? "是（正确）" : "否（可疑）");
        telemetry.addData("kp×quantStep", "%.4f（合格 ≤0.05）",
                Gen4Params.kp * Gen4Params.quantStep);
        telemetry.addData("kp_step", Gen4Params.kp * Gen4Params.quantStep);
        telemetry.addData("dip_depth", dipDepth);
        telemetry.addData("dip_recovery", Double.isNaN(dipRecovery) ? 0 : dipRecovery);
        telemetry.addData("dip_overshoot", dipOvershoot);
        telemetry.addData("dip_saw_unsettled", sawUnsettled ? 1 : 0);
        telemetry.addData("dash log", dashboardLog ? "ON（Dashboard Logging）" : "OFF");
        telemetry.update();


        while (opModeIsActive() && !gamepad1.x) {
            idle();
        }
    }

    // ------------------------------------------------------------ 工具

    private void logFrame(double t, double target, String phase) {
        if (!dashboardLog) {
            return;
        }
        if (lastLogSec >= 0 && (t - lastLogSec) * 1000.0 < logPeriodMs) {
            return;
        }
        lastLogSec = t;
        telemetry.addData("target", target);
        telemetry.addData("velocity", calculator.getVelocityFiltered());
        telemetry.addData("error", calculator.getError());
        telemetry.addData("duty", calculator.getDuty());
        telemetry.addData("ff", calculator.getFfTerm());
        telemetry.addData("p", calculator.getPTerm());
        telemetry.addData("i", calculator.getITerm());
        telemetry.addData("iactive", b2d(calculator.isIntegralActive()));
        telemetry.addData("settled", b2d(calculator.isSettled()));
        telemetry.addData("vbus", driver.getBusVoltage());
        telemetry.addData("vscale", calculator.getVScale());
        telemetry.addData("phase", phaseNum(phase));
    }

    private static double phaseNum(String phase) {
        switch (phase) {
            case "settle":
            case "point":
                return 1;
            case "measure":
                return 2;
            case "step":
                return 3;
            case "dip":
                return 4;
            case "recover":
                return 5;
            case "dip_settle":
                return 6;
            default:
                return 0;
        }
    }

    private static double b2d(boolean b) {
        return b ? 1 : 0;
    }
}
