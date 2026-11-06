package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelCalculator;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams;

import java.util.Locale;

/**
 * 飞轮运转程序（第 4 代控制系统验证用）：Start 后飞轮<b>不转</b>，目标转速完全由按键给出。
 *
 * <p>面向「现场验证第 4 代控制律」而非整定：整定参数固化在
 * {@link org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams} 的 {@code BIG} 参数集里
 * （Dashboard 分组 {@code FlywheelParams}），本程序只做目标转速管理、发起闭环绕组、并按需落 CSV。
 *
 * <p><b>操作</b>
 * <ul>
 *   <li>Start → 目标转速 {@code 0}（飞轮不转；这是旧版最大的差别：旧版 Start 会直接拉到待发转速）</li>
 *   <li>A → 目标转速置为 {@link #aVelocity}（默认 1600 tick/s）</li>
 *   <li>B → 目标转速置为 {@link #bVelocity}（默认 1900 tick/s）</li>
 *   <li>十字键上 / 下 → 按 {@link #velocityStep} 步长增减目标转速，按住则按 {@link #repeatPeriodMs} 连续调节</li>
 *   <li>X → 立即停转（目标转速置 0，闭环立刻收油门，飞轮惯性滑行）</li>
 * </ul>
 *
 * <p>目标转速限幅在 [0, {@link #maxVelocity}]。
 *
 * <p><b>数据输出</b>：全部通过 telemetry 打出（不落自定义 CSV 文件），由 FTC Dashboard 自带的
 * Logging 记录。{@link #dashboardLog} 为 {@code true}（默认）时每 {@link #logPeriodMs} 毫秒发布一组
 * <b>扁平数值键</b>，Dashboard 记下来的 CSV 列名就是这些键：
 * {@code target, velocity, duty, voltage, error, current, integral, pTerm, iTerm, ffTerm,
 * vscale, iactive, settled}（时间列由 Dashboard 自己加），可直接喂给
 * {@code tools/tuning} 的出图脚本（见 {@code tools/tuning/PLOT_GUIDE.md}）。
 * 关掉开关则退回给人看的整表遥测（{@code shooter.setTelemetry()}）。
 *
 * <p>常规遥测为飞轮统一列表（5 行）：目标转速 / 当前转速 / power(占空比) / 电流 / 电压。
 */
@Config
@TeleOp(name = "FlywheelRun", group = "Tests")
public class FlywheelRun extends LinearOpMode {

    /** A 键目标转速 (tick/s) */
    public static int aVelocity = 1600;

    /** B 键目标转速 (tick/s) */
    public static int bVelocity = 1900;

    /** 方向键单次调节步长 (tick/s) */
    public static int velocityStep = 100;

    /** 目标转速上限 (tick/s) */
    public static int maxVelocity = 3000;

    /** 按住方向键时的连续调节周期 (毫秒) */
    public static long repeatPeriodMs = 100;

    /** 是否发布扁平数值遥测（由 Dashboard 自带 Logging 记录；关掉则只显示人看的整表） */
    public static boolean dashboardLog = true;

    /** 数值遥测的发布周期 (毫秒)；Dashboard 的 Logging 自己也有采样节流 */
    public static long logPeriodMs = 20;

    /**
     * 用哪一套飞轮参数：{@code "BIG"}（大球）或 {@code "SMALL"}（小球）。
     * 选 {@code SMALL} 时本程序会拿 {@link FlywheelParams#SMALL} 构造闭环，
     * 因此可以原样验证「小球那一套参数」。
     */
    public static String paramsSet = "BIG";

    /**
     * 飞轮电机设备名。大球是 {@code "shooter"}；若小飞轮是另一个电机，
     * 在这里填它在 Robot Config 里的名字。
     */
    public static String motorName = "shooter";

    private PrototypeShooter shooter;

    /** 上一次的方向键状态：-1 下 / 0 未按 / 1 上 */
    private int lastDir = 0;

    /** 连续调节计时器 */
    private final ElapsedTime repeatTimer = new ElapsedTime();

    /** 数值遥测发布计时器 */
    private final ElapsedTime logTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        final boolean useSmall = "SMALL".equalsIgnoreCase(paramsSet);
        final FlywheelParams chosen = useSmall ? FlywheelParams.SMALL : FlywheelParams.BIG;
        shooter = new PrototypeShooter(hardwareMap, telemetry, motorName, chosen);
        shooter.stop();

        telemetry.addLine("Start -> no spin (target = 0)");
        telemetry.addLine("A -> " + aVelocity + " tick/s    B -> " + bVelocity + " tick/s");
        telemetry.addLine("Dpad Up/Down: adjust target (hold to repeat)    X: stop");
        telemetry.addLine("Params: FlywheelParams." + (useSmall ? "SMALL" : "BIG")
                + "    Motor: " + motorName);
        telemetry.addLine(String.format(Locale.US,
                "  kS=%.5f kff=%.6f kp=%.2e ki=%.2e speedMax=%d reversed=%s",
                chosen.kS, chosen.kff, chosen.kp, chosen.ki, chosen.speedMax, chosen.motorReversed));
        if (useSmall && isSmallPlaceholder()) {
            telemetry.addLine("  NOTE: SMALL is still a copy of BIG (not calibrated yet)");
        }
        telemetry.addLine("Dash logging: " + (dashboardLog
                ? "ON — FLAT KEYS target/velocity/duty/... (record with Dashboard Logging)"
                : "OFF (pretty table only)"));
        if (!dashboardLog) {
            shooter.setTelemetry();
        }
        telemetry.update();

        waitForStart();

        // Start 后不转：目标转速为 0，等按键指令
        shooter.setTargetVelocity(0);
        logTimer.reset();

        while (opModeIsActive()) {
            if (gamepad1.xWasPressed()) {
                shooter.stop(); // 立即停转
            }
            if (gamepad1.aWasPressed()) {
                shooter.setTargetVelocity(clampVelocity(aVelocity));
            }
            if (gamepad1.bWasPressed()) {
                shooter.setTargetVelocity(clampVelocity(bVelocity));
            }

            // 方向键上下调节目标转速：按下立即走一步，按住则按 repeatPeriodMs 连续调节
            int dir = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
            if (dir != lastDir) {
                if (dir != 0) {
                    shooter.setTargetVelocity(clampVelocity(shooter.getTargetVelocity() + dir * velocityStep));
                }
                repeatTimer.reset();
            } else if (dir != 0 && repeatTimer.milliseconds() >= repeatPeriodMs) {
                shooter.setTargetVelocity(clampVelocity(shooter.getTargetVelocity() + dir * velocityStep));
                repeatTimer.reset();
            }
            lastDir = dir;

            shooter.update();
            if (dashboardLog) {
                if (logTimer.milliseconds() >= logPeriodMs) {
                    logTimer.reset();
                    publishFlat();
                }
            } else {
                shooter.setTelemetry();
            }
            telemetry.update();
        }

        // 退出前收油门
        shooter.stop();
        shooter.update();
    }

    /**
     * 发布一组扁平数值键（列名 = 键名，时间列由 Dashboard 自己加）。
     *
     * <p>键名与 {@code tools/tuning} 下的出图脚本对齐：转速类一律叫 {@code velocity}，
     * 占空比一律叫 {@code duty}，这样 {@code plot_csv.py} 无需改名即可出图。
     */
    private void publishFlat() {
        FlywheelCalculator calc = shooter.getCalculator();
        telemetry.addData("target", shooter.getTargetVelocity());
        telemetry.addData("velocity", shooter.getVelocity());
        telemetry.addData("duty", shooter.getDuty());
        telemetry.addData("voltage", shooter.getBusVoltage());
        telemetry.addData("error", shooter.getError());
        telemetry.addData("current", shooter.getCurrent());
        telemetry.addData("integral", calc.getIntegral());
        telemetry.addData("pTerm", calc.getPTerm());
        telemetry.addData("iTerm", calc.getITerm());
        telemetry.addData("ffTerm", calc.getFfTerm());
        telemetry.addData("vscale", calc.getVScale());
        telemetry.addData("iactive", calc.isIntegralActive() ? 1 : 0);
        telemetry.addData("settled", calc.isSettled() ? 1 : 0);
    }

    private static int clampVelocity(int velocity) {
        return Math.max(0, Math.min(maxVelocity, velocity));
    }

    /** 判断 SMALL 是否仍是 BIG 的占位拷贝（标定完成后自动变 false，提示行消失）。 */
    private static boolean isSmallPlaceholder() {
        FlywheelParams b = FlywheelParams.BIG;
        FlywheelParams s = FlywheelParams.SMALL;
        return Math.abs(b.kS - s.kS) < 1e-9
                && Math.abs(b.kff - s.kff) < 1e-12
                && Math.abs(b.kp - s.kp) < 1e-12
                && Math.abs(b.ki - s.ki) < 1e-12;
    }
}
