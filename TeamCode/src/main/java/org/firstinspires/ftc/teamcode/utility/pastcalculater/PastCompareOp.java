package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * 多代飞轮控制系统对比实验（第 1/2 代 · 第 3 代 · 第 4 代）。
 *
 * <p>两种跑法（Dashboard 勾 {@link #offlineReplay} 切换）：
 * <ul>
 *   <li><b>在线</b>：Dashboard 的 {@link #activeGen} 决定<b>哪一代真驱动电机</b>；三代计算器
 *       每帧吃同一组 {@code (target, velocity, vBus, dt)}，把三份占空比都写进 CSV。
 *       “每次跑一个”——换一代就在 Dashboard 改 {@link #activeGen}（或按 Y 循环切换）重跑，
 *       因为三代接收的输入完全相同，两次运行的 duty 轨迹可以直接叠在一起比。</li>
 *   <li><b>离线</b>：读 {@code <externalFilesDir>/tuning/<replayFile>} 的历史 CSV（必须含
 *       {@code velocity} 与 {@code target} 列，{@code t} 列可选），把同一段真实转速序列喂给三代，
 *       只比 duty——不需要机器人、不接电机。</li>
 * </ul>
 *
 * <p>按键：A → {@link #aVelocity}，B → {@link #bVelocity}，十字键上下 ±{@link #velocityStep}
 * （按住连调），X → 停（目标 0 并清积分），Y → 切换当前代（并复位三代滤波器）。
 */
@TeleOp(name = "PastCompare 多代对比", group = "PastCompare")
public class PastCompareOp extends LinearOpMode {

    // ------------------------------------------------------------ @Config

    /** true = 离线回放 {@link #replayFile}，不驱动任何电机 */
    public static boolean offlineReplay = false;
    /** 离线回放的文件名（相对 {@code <externalFilesDir>/tuning/}） */
    public static String replayFile = "FlywheelRun_1700000000000.csv";
    /** 在线时真正驱动电机的那一代 */
    public static PastGen activeGen = PastGen.GEN4_CURRENT;

    public static double aVelocity = 1600;
    public static double bVelocity = 1900;
    public static int velocityStep = 100;
    public static int maxVelocity = 3000;
    /** 按住十字键的连调周期 (ms) */
    public static double repeatPeriodMs = 100;
    /** 每帧都发太浪费；按此周期发布一组数值遥测 */
    public static double logPeriodMs = 10;
    /** 是否发布扁平数值遥测（由 FTC Dashboard 自带 Logging 记录） */
    public static boolean dashboardLog = true;

    public static String motorName = "shooter";
    public static boolean motorReversed = true;

    /**
     * 遥测键名（= Dashboard CSV 的列名）：{@code gen} 列是本次运行真正驱动的是哪一代
     * （离线回放记 0），其余按代分组：{@code duty_gX, ff_gX, p_gX, i_gX, drop_gX, iact_gX, stl_gX}
     * （{@code boost_gX} 只有第 3 代有；Gen4 另有 {@code vscale_g4}）。
     * 时间列由 Dashboard 自己加。
     */
    static final String CSV_HEADER =
            "t,gen,target,velocity,vbus,"
            + "duty_g1,ff_g1,p_g1,i_g1,drop_g1,iact_g1,stl_g1,"
            + "duty_g3,ff_g3,p_g3,i_g3,boost_g3,drop_g3,iact_g3,stl_g3,"
            + "duty_g4,ff_g4,p_g4,i_g4,drop_g4,iact_g4,stl_g4,vscale_g4";

    // ------------------------------------------------------------ state

    private PastCalculator gen1;
    private PastCalculator gen3;
    private PastCalculator.Gen4 gen4;
    private PastFlywheelDriver driver;

    private int targetVelocity = 0;
    private double lastLogSec = -1;
    private long frames = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gen1 = PastCalculator.create(PastGen.GEN1_PIDSVA);
        gen3 = PastCalculator.create(PastGen.GEN3_BOOST);
        gen4 = new PastCalculator.Gen4();
        gen4.reset(0);

        telemetry.addLine("多代对比 PastCompare");
        telemetry.addLine(offlineReplay
                ? "离线回放：" + replayFile
                : "在线驱动：" + activeGen.label);
        telemetry.addLine("A=" + (int) aVelocity + "  B=" + (int) bVelocity
                + "  步长=" + velocityStep);
        telemetry.addLine("Y 切换当前代 / X 停 / 十字键上下调目标");
        telemetry.addLine(offlineReplay ? "（离线模式不接电机）" : "电机口：" + motorName);

        if (!offlineReplay) {
            try {
                driver = new PastFlywheelDriver(hardwareMap, motorName, motorReversed,
                        calculatorOf(activeGen));
            } catch (RuntimeException e) {
                telemetry.addLine("!! 找不到电机 " + motorName + "：" + e.getMessage());
                telemetry.update();
                sleep(3000);
                return;
            }
        }

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        if (offlineReplay) {
            runReplay();
        } else {
            runOnline();
        }
    }

    // ------------------------------------------------------------ 在线

    private void runOnline() {
        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastY = false;
        double lastRepeat = 0;

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            if (gamepad1.a) {
                targetVelocity = (int) aVelocity;
            }
            if (gamepad1.b) {
                targetVelocity = (int) bVelocity;
            }
            if (gamepad1.x) {
                targetVelocity = 0;
            }
            if (gamepad1.y && !lastY) {
                switchGen();
            }
            lastY = gamepad1.y;

            // 十字键：按住连调
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean edge = (up && !lastUp) || (down && !lastDown);
            if (edge || ((up || down) && nowMs - lastRepeat >= repeatPeriodMs)) {
                if (up) {
                    targetVelocity = Math.min(maxVelocity, targetVelocity + velocityStep);
                } else if (down) {
                    targetVelocity = Math.max(0, targetVelocity - velocityStep);
                }
                lastRepeat = nowMs;
            }
            lastUp = up;
            lastDown = down;

            double velocity = driver.getVelocity();
            double vbus = driver.getBusVoltage();
            double dt = driver.nextDt();

            double d1 = gen1.calculate(targetVelocity, velocity, vbus, dt);
            double d3 = gen3.calculate(targetVelocity, velocity, vbus, dt);
            double d4 = gen4.calculate(targetVelocity, velocity, vbus, dt);

            // 只有当前代真正驱动电机（其余两代只演算）
            switch (activeGen) {
                case GEN1_PIDSVA:
                    driver.motor.setPower(d1);
                    break;
                case GEN3_BOOST:
                    driver.motor.setPower(d3);
                    break;
                default:
                    driver.motor.setPower(d4);
                    break;
            }

            frames++;
            logRow(getRuntime(), activeGen.id, targetVelocity, velocity, vbus, d1, d3, d4);
            showOnlineTelemetry(velocity, vbus, d1, d3, d4);
            sleep(5);
        }
        driver.motor.setPower(0);
    }

    private void switchGen() {
        PastGen next;
        switch (activeGen) {
            case GEN1_PIDSVA:
                next = PastGen.GEN3_BOOST;
                break;
            case GEN3_BOOST:
                next = PastGen.GEN4_CURRENT;
                break;
            default:
                next = PastGen.GEN1_PIDSVA;
                break;
        }
        activeGen = next;
        targetVelocity = 0;
        gen1.reset(driver.getVelocity());
        gen3.reset(driver.getVelocity());
        gen4.reset(driver.getVelocity());
    }

    private void showOnlineTelemetry(double velocity, double vbus, double d1, double d3, double d4) {
        telemetry.addData("当前驱动", activeGen.label);
        telemetry.addData("target / velocity", "%d / %.0f (%.1f%%)",
                targetVelocity, velocity,
                targetVelocity == 0 ? 0 : 100.0 * (velocity - targetVelocity) / targetVelocity);
        telemetry.addData("duty g1 / g3 / g4", "%.4f / %.4f / %.4f", d1, d3, d4);
        telemetry.addData("settled g1 / g3 / g4", "%s / %s / %s",
                gen1.isSettled(), gen3.isSettled(), gen4.isSettled());
        telemetry.addData("bus voltage", Double.isNaN(vbus) ? "无读数" : String.format(java.util.Locale.US, "%.2f V", vbus));
        telemetry.addData("current", String.format(java.util.Locale.US, "%.2f A", driver.getCurrent()));
        telemetry.addData("dash log", dashboardLog ? "ON（Dashboard Logging）" : "OFF");
        telemetry.addData("frames", frames);
        telemetry.update();
    }

    // ------------------------------------------------------------ 离线回放

    private void runReplay() {
        File dir = hardwareMap.appContext.getExternalFilesDir(null);
        File file = new File(new File(dir, "tuning"), replayFile);
        if (!file.isFile()) {
            telemetry.addLine("回放文件不存在：" + file.getAbsolutePath());
            telemetry.update();
            sleep(4000);
            return;
        }

        List<String> lines = readAllLines(file);
        if (lines.isEmpty()) {
            telemetry.addLine("回放文件是空的：" + file.getAbsolutePath());
            telemetry.update();
            sleep(4000);
            return;
        }

        int idxT = -1;
        int idxTarget = -1;
        int idxVelocity = -1;
        int idxVbus = -1;
        int headerIdx = -1;
        for (int i = 0; i < lines.size(); i++) {
            String line = lines.get(i).trim();
            if (line.isEmpty() || line.startsWith("#")) {
                continue;
            }
            String[] cols = line.split(",");
            for (int c = 0; c < cols.length; c++) {
                String key = cols[c].trim().toLowerCase();
                if (key.equals("t") || key.equals("time") || key.equals("time_s")) {
                    idxT = c;
                } else if (key.equals("target")) {
                    idxTarget = c;
                } else if (key.equals("velocity")) {
                    idxVelocity = c;
                } else if (key.equals("vbus") || key.equals("voltage")) {
                    idxVbus = c;
                }
            }
            headerIdx = i;
            break;
        }

        if (idxVelocity < 0) {
            telemetry.addLine("回放文件缺少 velocity 列：" + file.getAbsolutePath());
            telemetry.update();
            sleep(4000);
            return;
        }

        double lastT = 0;
        int rows = 0;
        for (int i = headerIdx + 1; i < lines.size(); i++) {
            if (!opModeIsActive()) {
                break;
            }
            String line = lines.get(i).trim();
            if (line.isEmpty() || line.startsWith("#")) {
                continue;
            }
            String[] cols = line.split(",");
            double t = idxT >= 0 && idxT < cols.length ? parse(cols[idxT], lastT + 0.02) : lastT + 0.02;
            double dt = t - lastT;
            lastT = t;
            double velocity = parse(cols[idxVelocity], 0);
            double target = idxTarget >= 0 && idxTarget < cols.length ? parse(cols[idxTarget], 0) : 0;
            double vbus = idxVbus >= 0 && idxVbus < cols.length ? parse(cols[idxVbus], Double.NaN) : Double.NaN;

            double d1 = gen1.calculate(target, velocity, vbus, dt);
            double d3 = gen3.calculate(target, velocity, vbus, dt);
            double d4 = gen4.calculate(target, velocity, vbus, dt);
            frames++;
            rows++;
            logRow(t, 0, target, velocity, vbus, d1, d3, d4);

            if (rows % 20 == 0) {
                telemetry.addData("回放进度", "%d/%d", rows, lines.size() - headerIdx - 1);
                telemetry.addData("t", "%.2f s", t);
                telemetry.addData("duty g1 / g3 / g4", "%.4f / %.4f / %.4f", d1, d3, d4);
                telemetry.addData("dash log", dashboardLog ? "ON" : "OFF");
                telemetry.update();
            }
        }

        telemetry.addData("回放完成", "%d 行", rows);
        telemetry.addData("dash log", dashboardLog ? "ON（Dashboard Logging）" : "OFF");
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }

    private static double parse(String s, double fallback) {
        try {
            return Double.parseDouble(s.trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static List<String> readAllLines(File file) {
        List<String> out = new ArrayList<>();
        try (BufferedReader r = new BufferedReader(
                new InputStreamReader(new FileInputStream(file), StandardCharsets.UTF_8))) {
            String line;
            while ((line = r.readLine()) != null) {
                out.add(line);
            }
        } catch (Exception e) {
            // 读不了就当空文件，由调用方提示
        }
        return out;
    }

    // ------------------------------------------------------------ 公共

    /**
     * 把一行数据发布成扁平数值遥测（键名见 {@link #CSV_HEADER}），由 Dashboard 自带
     * Logging 记录。三代的分量同时发布，所以一次运行就能对比。
     */
    private void logRow(double t, int genId, double target, double velocity, double vbus,
                        double d1, double d3, double d4) {
        if (!dashboardLog) {
            return;
        }
        if (lastLogSec >= 0 && (t - lastLogSec) * 1000.0 < logPeriodMs) {
            return;
        }
        lastLogSec = t;

        telemetry.addData("gen", genId);
        telemetry.addData("target", target);
        telemetry.addData("velocity", velocity);
        telemetry.addData("vbus", num(vbus));

        publish("duty", 1, d1);
        publish("ff", 1, gen1.getFfTerm());
        publish("p", 1, gen1.getPTerm());
        publish("i", 1, gen1.getITerm());
        telemetry.addData("drop_g1", b2d(gen1.isInDrop()));
        telemetry.addData("iact_g1", b2d(gen1.isIntegralActive()));
        telemetry.addData("stl_g1", b2d(gen1.isSettled()));

        publish("duty", 3, d3);
        publish("ff", 3, gen3.getFfTerm());
        publish("p", 3, gen3.getPTerm());
        publish("i", 3, gen3.getITerm());
        publish("boost", 3, gen3.getBoostTerm());
        telemetry.addData("drop_g3", b2d(gen3.isInDrop()));
        telemetry.addData("iact_g3", b2d(gen3.isIntegralActive()));
        telemetry.addData("stl_g3", b2d(gen3.isSettled()));

        publish("duty", 4, d4);
        publish("ff", 4, gen4.getFfTerm());
        publish("p", 4, gen4.getPTerm());
        publish("i", 4, gen4.getITerm());
        telemetry.addData("drop_g4", b2d(gen4.isInDrop())); // 恒 0：第 4 代已删状态机
        telemetry.addData("iact_g4", b2d(gen4.isIntegralActive()));
        telemetry.addData("stl_g4", b2d(gen4.isSettled()));
        telemetry.addData("vscale_g4", gen4 instanceof PastCalculator.Gen4
                ? ((PastCalculator.Gen4) gen4).getVScale() : 1.0);
    }

    private void publish(String what, int gen, double value) {
        telemetry.addData(what + "_g" + gen, value);
    }

    private static double b2d(boolean b) {
        return b ? 1 : 0;
    }

    private static double num(double v) {
        return Double.isNaN(v) || Double.isInfinite(v) ? 0.0 : v;
    }

    private PastCalculator calculatorOf(PastGen gen) {
        switch (gen) {
            case GEN1_PIDSVA:
                return gen1;
            case GEN3_BOOST:
                return gen3;
            default:
                return gen4;
        }
    }
}
