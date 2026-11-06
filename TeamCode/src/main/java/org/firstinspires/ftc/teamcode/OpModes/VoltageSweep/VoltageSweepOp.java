package org.firstinspires.ftc.teamcode.OpModes.VoltageSweep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelCalculator;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * 母线电压适配性扫描（第 4 代控制系统专用）：用<b>第二个电机当电气负载</b>把母线电压拉到指定值，
 * 在若干个电压点上重复同一套动作（1600 稳态 / 1900 稳态 / 一次模拟掉速），
 * 用来回答「同一个目标转速，在不同电量下占空比是否等价」——即
 * {@code doc/现在的控制系统.md} §2.12 判据⑥。
 *
 * <h2>为什么不直接等电池放电</h2>
 * 电池放电要几十分钟、电压还在持续变，一次实验内无法重复。
 * 这里用「负载电机抽电流 → 母线内阻上压降变大 → 母线电压下降」的办法，
 * 并用一个<b>电压闭环</b>（PI + 速率限幅）把母线电压稳定在设定值 ±0.15 V，
 * 于是每个电压点都<b>可复现、可重复</b>，同一块电池一次跑完 4 个点。
 *
 * <h2>计划（自动生成，见遥测与 CSV 注释）</h2>
 * 对 {@link #vbusTargets} 里的每个电压 V：
 * <pre>
 *   ① V_settle  飞轮 0，负载闭环把母线拉到 V               {@link #voltageSettleSec} s
 *   ② V_a1600   飞轮 {@link #aVelocity}，稳态                  {@link #steadySec} s
 *   ③ V_b1900   飞轮 {@link #bVelocity}，稳态                  {@link #steadySec} s
 *   ④ V_pre     飞轮 {@link #bVelocity}，掉速前基线            {@link #preDisturbSec} s
 *   ⑤ V_dip     开环把占空比压到 {@link #dipDuty}，产生真实掉速  {@link #dipSec} s
 *   ⑥ V_rec     回到闭环，看恢复时间/过冲                     {@link #recoverySec} s
 *   ⑦ V_rel     飞轮 0，释放                                     {@link #releaseSec} s
 * </pre>
 * 最后按段汇总：母线电压、`duty·V_bus`、`|I|`、稳态 sd、`settled` 占比。
 *
 * <h2>CSV</h2>
 * {@code <外部目录>/tuning/VoltageSweep_<时间戳>.csv}，表头
 * {@code t,segment_id,vbus_target,vbus_meas,load_power,target,velocity,duty,error,current,iActive,settled}
 * （`segment_id` = 计划里第几段，段名写在 CSV 注释行）。
 * 出图：{@code python tools/tuning/plot_csv.py <csv> --segment segment_id}。
 *
 * <h2>按键</h2>
 * <ul>
 *   <li>Start → 开始自动扫描（Init 期间可按 Y 进手动负载模式做接线自检）</li>
 *   <li>X → 立刻中止：负载归零、飞轮停，然后停在汇总屏</li>
 * </ul>
 *
 * <p><b>安全</b>：母线低于 {@link #minSafeVbus} 立即中止并卸负载；{@link #loadPowerMax} 限死负载电机占空比。
 * 详见同目录 {@code GUIDE.md}。
 */
@Config
@TeleOp(name = "VoltageSweep", group = "Tests")
public class VoltageSweepOp extends LinearOpMode {

    // ---------------------------------------------------------------- 计划参数

    /** 稳态段 A 的目标转速 (tick/s) */
    public static int aVelocity = 1600;

    /** 稳态段 B 的目标转速 (tick/s)，同时也是掉速段的工作点 */
    public static int bVelocity = 1900;

    /** 要扫的母线电压点 (V)：按书写顺序依次跑 */
    public static double[] vbusTargets = {8.5, 10.0, 11.5, 12.9};

    /** 每个电压点开始前的「拉到目标电压」时间 (s)（飞轮不转，只跑负载闭环） */
    public static double voltageSettleSec = 2.0;

    /** 每个稳态段时长 (s)（分析取末尾 1~2 s） */
    public static double steadySec = 3.0;

    /** 掉速前的基线时长 (s) */
    public static double preDisturbSec = 1.0;

    /** 模拟掉速的时长 (s)（开环低占空比） */
    public static double dipSec = 0.35;

    /** 掉速期间的开环占空比（0.15 ≈ 完全松油门，产生 200~400 tick/s 跌深） */
    public static double dipDuty = 0.15;

    /** 掉速后的恢复观察窗 (s) */
    public static double recoverySec = 2.0;

    /** 每个电压点结束后的释放时间 (s)（负载归零、飞轮停） */
    public static double releaseSec = 0.5;

    /** 是否跑掉速段（只想看稳态就关掉，实验时间减半） */
    public static boolean disturbanceEnabled = true;

    // ---------------------------------------------------------------- 负载电机

    /** 负载电机在 Robot Configuration 里的名字 */
    public static String loadMotorName = "loadMotor";

    /** 负载电机方向反了就置 true */
    public static boolean loadDirectionReversed = false;

    /** 负载电机占空比上限（防止一路拉死） */
    public static double loadPowerMax = 1.0;

    /** 负载占空比变化率上限 (占空比/s)：越小越稳，但拉电压越慢 */
    public static double loadRateLimit = 2.5;

    /** 电压环比例增益 (占空比/V) */
    public static double kpVolt = 0.45;

    /** 电压环积分增益 (占空比/(V·s))：决定「稳态电压误差」被吃干净的速度 */
    public static double kiVolt = 0.8;

    /** 母线低于该电压立即中止 (V) */
    public static double minSafeVbus = 7.5;

    /** 「稳定在目标 ±该值」判据 (V)，只用于遥测显示 */
    public static double vbusTolerance = 0.15;

    // ---------------------------------------------------------------- 记录

    /**
     * 是否发布扁平数值遥测（由 FTC Dashboard 自带 Logging 记录）。
     *
     * <p>键名（Dashboard CSV 的列名）：{@code segment_id, vbus_target, vbus_meas, load_power,
     * target, velocity, duty, error, current, iactive, settled}，时间列由 Dashboard 自己加。
     */
    public static boolean dashboardLog = true;

    /** 数值遥测发布周期 (ms) */
    public static long logPeriodMs = 10;

    /** 一个计划段。kind：0 = 常态段，1 = 开环掉速段 */
    private static final class Segment {
        final int id;
        final String name;
        final double vbusTarget;
        final int wheelTarget;
        final double durSec;
        final boolean dip;

        Segment(int id, String name, double vbusTarget, int wheelTarget,
                double durSec, boolean dip) {
            this.id = id;
            this.name = name;
            this.vbusTarget = vbusTarget;
            this.wheelTarget = wheelTarget;
            this.durSec = durSec;
            this.dip = dip;
        }
    }

    private PrototypeShooter shooter;
    private DcMotorEx loadMotor;

    private final List<Segment> plan = new ArrayList<>();
    /** 每行：{segment_id, t, vbus_target, vbus_meas, velocity, duty, current, error, settled} */
    private final List<double[]> samples = new ArrayList<>();

    /** 电压环状态 */
    private double loadPower = 0;
    private double loadIntegral = 0;

    /** 手动负载模式（接线自检） */
    private boolean manualMode = false;
    private double manualPower = 0;

    /** 上一次写 CSV / 计算 dt 的时刻 */
    private double lastLogT = -1;
    private double lastLoopT = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new PrototypeShooter(hardwareMap, telemetry);
        shooter.stop();

        loadMotor = hardwareMap.tryGet(DcMotorEx.class, loadMotorName);
        if (loadMotor == null) {
            telemetry.addLine("!! 找不到负载电机 \"" + loadMotorName + "\"");
            telemetry.addLine("   在 Robot Configuration 里命名一个空闲电机口，并在 Dashboard 改 loadMotorName");
            telemetry.addLine("   没有负载电机也可以跑（只当普通稳速测试），但电压不会被拉低");
            telemetry.update();
        } else {
            loadMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            loadMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            loadMotor.setDirection(loadDirectionReversed
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            loadMotor.setPower(0);
        }

        buildPlan();

        telemetry.addLine("VoltageSweep: 电压点 " + fmtArray(vbusTargets) + " V");
        telemetry.addLine("计划 " + plan.size() + " 段，预计 " + String.format(Locale.US, "%.1f", totalSec()) + " s");
        telemetry.addLine("Start = 开始扫描    Init 期间 Y = 手动负载(接线自检)    X = 中止");
        for (Segment s : plan) {
            telemetry.addLine("  #" + s.id + " " + s.name);
        }
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.yWasPressed() && loadMotor != null) {
                manualMode = !manualMode;
                manualPower = 0;
                loadMotor.setPower(0);
            }
            if (manualMode) {
                manualPower = clamp(manualPower
                        + (gamepad1.dpad_up ? 0.05 : 0) - (gamepad1.dpad_down ? 0.05 : 0), 0, loadPowerMax);
                loadMotor.setPower(manualPower * (loadDirectionReversed ? -1 : 1));
                telemetry.addLine("手动负载模式: 占空比 " + String.format(Locale.US, "%.2f", manualPower)
                        + "   vbus " + vbusText());
            } else {
                telemetry.addLine("A / Start = 开始扫描    Y = 手动负载");
            }
            telemetry.update();
            sleep(40);
        }
        if (!opModeIsActive()) {
            return;
        }

        boolean aborted = false;
        boolean userStop = false;
        Segment prev = null;
        for (Segment seg : plan) {
            if (!opModeIsActive()) {
                break;
            }
            // 掉速段 → 恢复段：重新播种滤波器，避免带着「开环期间停更」的状态继续
            if (prev != null && prev.dip && !seg.dip) {
                shooter.resetController();
            }
            prev = seg;

            double t0 = getRuntime();
            while (opModeIsActive() && getRuntime() - t0 < seg.durSec) {
                double now = getRuntime();
                double dt = lastLoopT < 0 ? 0.01 : clamp(now - lastLoopT, 0.001, 0.1);
                lastLoopT = now;
                double vbus = readBusVoltage();

                if (vbus < minSafeVbus) {
                    aborted = true;
                    break;
                }
                if (gamepad1.x) {
                    aborted = true;
                    userStop = true;
                    break;
                }

                // 电压环：母线高于目标 → 加负载（多抽电流把电压压下来）
                updateLoad(seg.vbusTarget, vbus, dt);

                if (seg.dip) {
                    // 开环掉速：不走闭环（update() 会覆盖输出）
                    shooter.setOpenLoopPower(dipDuty);
                } else {
                    shooter.setTargetVelocity(seg.wheelTarget);
                    shooter.update();
                }

                showTelemetry(seg, t0, vbus);

                if (dashboardLog && now - lastLogT >= logPeriodMs / 1000.0) {
                    lastLogT = now;
                    logRow(seg, vbus);
                }
            }
            if (aborted) {
                break;
            }
        }

        // ---- 收尾：卸负载、停飞轮 ----
        setLoadPower(0);
        shooter.stop();
        shooter.update();
        if (userStop) {
            telemetry.addLine("!! 用户按 X 中止（已完成 " + samples.size() + " 个采样点）");
        } else if (aborted) {
            telemetry.addLine("!! 母线 " + vbusText() + " < minSafeVbus=" + minSafeVbus + "，已中止");
        }
        Map<Integer, String> names = new LinkedHashMap<>();
        for (Segment s : plan) {
            names.put(s.id, s.name);
        }
        String summary = summarize(names);

        // 停机后把汇总留在屏幕上（直到按 X 或 Stop）
        while (opModeIsActive() && !isStopRequested()) {
            for (String line : summary.split("\n")) {
                telemetry.addLine(line);
            }
            telemetry.addLine("X = 退出");
            telemetry.update();
            if (gamepad1.x) {
                break;
            }
            sleep(50);
        }
        setLoadPower(0);
    }

    // ---------------------------------------------------------------- 计划

    private void buildPlan() {
        plan.clear();
        int id = 0;
        for (double vt : vbusTargets) {
            String tag = "v" + String.format(Locale.US, "%.1f", vt).replace('.', 'p');
            plan.add(new Segment(id++, tag + "_settle", vt, 0, voltageSettleSec, false));
            plan.add(new Segment(id++, tag + "_a" + aVelocity, vt, aVelocity, steadySec, false));
            plan.add(new Segment(id++, tag + "_b" + bVelocity, vt, bVelocity, steadySec, false));
            if (disturbanceEnabled) {
                plan.add(new Segment(id++, tag + "_pre", vt, bVelocity, preDisturbSec, false));
                plan.add(new Segment(id++, tag + "_dip", vt, bVelocity, dipSec, true));
                plan.add(new Segment(id++, tag + "_rec", vt, bVelocity, recoverySec, false));
            }
            plan.add(new Segment(id++, tag + "_rel", vt, 0, releaseSec, false));
        }
    }

    private double totalSec() {
        double s = 0;
        for (Segment seg : plan) {
            s += seg.durSec;
        }
        return s;
    }

    // ---------------------------------------------------------------- 电压环

    private double readBusVoltage() {
        // 与第 4 代控制律读同一个量：所有 voltageSensor 里最小的正读数（无读数时为 NaN）
        return shooter.getBusVoltage();
    }

    private void updateLoad(double vbusTarget, double vbus, double dt) {
        if (loadMotor == null || !Double.isFinite(vbus)) {
            setLoadPower(0);
            return;
        }
        double err = vbus - vbusTarget; // >0 母线偏高 → 需要更多负载
        loadIntegral = clamp(loadIntegral + kiVolt * err * dt, 0, loadPowerMax);
        double want = clamp(loadIntegral + kpVolt * err, 0, loadPowerMax);
        double step = clamp(want - loadPower, -loadRateLimit * dt, loadRateLimit * dt);
        setLoadPower(clamp(loadPower + step, 0, loadPowerMax));
    }

    private void setLoadPower(double power) {
        loadPower = clamp(power, 0, loadPowerMax);
        if (loadMotor != null) {
            loadMotor.setPower(loadPower * (loadDirectionReversed ? -1 : 1));
        }
    }

    // ---------------------------------------------------------------- 遥测 / CSV

    private void showTelemetry(Segment seg, double t0, double vbus) {
        double err = seg.vbusTarget - vbus;
        telemetry.addData("段", "#" + seg.id + " " + seg.name,
                String.format(Locale.US, "%.2f/%.2f s", getRuntime() - t0, seg.durSec));
        telemetry.addData("母线 目标 / 实测 / 差", "%.2f / %.2f / %+.2f V",
                seg.vbusTarget, vbus, err);
        telemetry.addData("母线合格", Math.abs(err) <= vbusTolerance ? "YES" : "NO");
        telemetry.addData("负载占空比", "%.3f", loadPower);
        telemetry.addData("飞轮 目标 / 实测 / duty", "%d / %.1f / %.3f",
                seg.wheelTarget, shooter.getVelocity(), dutyOf(seg));
        telemetry.addData("电流 / 误差", "%.2f A / %.1f", shooter.getCurrent(), shooter.getError());
        FlywheelCalculator c = shooter.getCalculator();
        telemetry.addData("iActive / settled", "%s / %s",
                c.isIntegralActive(), c.isSettled());
        telemetry.update();
    }

    private void logRow(Segment seg, double vbus) {
        FlywheelCalculator c = shooter.getCalculator();
        double duty = dutyOf(seg);
        telemetry.addData("segment_id", seg.id);
        telemetry.addData("vbus_target", seg.vbusTarget);
        telemetry.addData("vbus_meas", vbus);
        telemetry.addData("load_power", loadPower);
        telemetry.addData("target", seg.wheelTarget);
        telemetry.addData("velocity", shooter.getVelocity());
        telemetry.addData("duty", duty);
        telemetry.addData("error", seg.wheelTarget - shooter.getVelocity());
        telemetry.addData("current", shooter.getCurrent());
        telemetry.addData("iactive", c.isIntegralActive() ? 1 : 0);
        telemetry.addData("settled", c.isSettled() ? 1 : 0);
        samples.add(new double[]{
                seg.id, getRuntime(), seg.vbusTarget, vbus,
                shooter.getVelocity(), duty, shooter.getCurrent(),
                seg.wheelTarget - shooter.getVelocity(), c.isSettled() ? 1 : 0});
    }

    /** 掉速段电机走开环，`getDuty()` 是上一帧的闭环值 → 直接读电机实际输出。 */
    private double dutyOf(Segment seg) {
        return seg.dip ? shooter.motor.getPower() : shooter.getDuty();
    }

    // ---------------------------------------------------------------- 汇总

    /** 按段汇总：母线、duty·V_bus、|I|、稳态 sd、settled 占比。 */
    private String summarize(Map<Integer, String> names) {
        StringBuilder sb = new StringBuilder();
        sb.append("== VoltageSweep 汇总（每段只用后半段：跳过前 50% 的暂态） ==\n");
        sb.append(String.format(Locale.US,
                "%-16s%6s%9s%9s%9s%9s%9s%8s%8s%8s",
                "段", "n", "Vbus", "VbusSD", "v_avg", "vSD", "duty", "d×Vbus", "|I|", "settled%"));
        Map<Integer, List<double[]>> bySeg = new LinkedHashMap<>();
        for (double[] r : samples) {
            bySeg.computeIfAbsent((int) r[0], k -> new ArrayList<>()).add(r);
        }
        for (Map.Entry<Integer, List<double[]>> e : bySeg.entrySet()) {
            List<double[]> rows = e.getValue();
            int from = rows.size() / 2; // 后半段
            if (rows.size() - from < 5) {
                from = 0;
            }
            int n = rows.size() - from;
            double vbusSum = 0, vSum = 0, v2Sum = 0, dSum = 0, dvSum = 0, iSum = 0, stSum = 0, vbus2 = 0;
            for (int i = from; i < rows.size(); i++) {
                double[] r = rows.get(i);
                double vbus = r[3], v = r[4], duty = r[5], cur = r[6];
                if (!Double.isFinite(vbus)) {
                    vbus = Double.NaN;
                }
                vbusSum += Double.isNaN(vbus) ? 0 : vbus;
                vbus2 += Double.isNaN(vbus) ? 0 : vbus * vbus;
                vSum += v;
                v2Sum += v * v;
                dSum += duty;
                dvSum += duty * (Double.isNaN(vbus) ? 0 : vbus);
                iSum += Math.abs(cur);
                stSum += r[8];
            }
            double vbusMean = vbusSum / n;
            double vbusSd = Math.sqrt(Math.max(0, vbus2 / n - vbusMean * vbusMean));
            double vMean = vSum / n;
            double vSd = Math.sqrt(Math.max(0, v2Sum / n - vMean * vMean));
            sb.append(String.format(Locale.US, "%-16s%6d%9.3f%9.3f%9.1f%9.2f%9.4f%8.3f%8.2f%7.0f%%\n",
                    names.getOrDefault(e.getKey(), "#" + e.getKey()), n,
                    vbusMean, vbusSd, vMean, vSd, dSum / n, dvSum / n, iSum / n, 100.0 * stSum / n));
        }

        // 判据：同目标转速下 duty·V_bus 应跨电压点一致（§2.12⑥，≤3%）
        sb.append("-- 跨电压一致性（同目标转速下 duty·V_bus，判据 ≤3%）--\n");
        for (int target : new int[]{aVelocity, bVelocity}) {
            double lo = Double.NaN, hi = Double.NaN;
            for (Map.Entry<Integer, List<double[]>> e : bySeg.entrySet()) {
                List<double[]> rows = e.getValue();
                int from = rows.size() / 2;
                if (rows.size() - from < 5) {
                    from = 0;
                }
                double d = 0, v = 0, dv = 0;
                int n = 0;
                for (int i = from; i < rows.size(); i++) {
                    double[] r = rows.get(i);
                    if (Math.abs(r[4] - target) > 0.02 * target) {
                        continue; // 只看该目标转速的段
                    }
                    if (!Double.isFinite(r[3])) {
                        continue;
                    }
                    dv += r[5] * r[3];
                    d += r[5];
                    v += r[3];
                    n++;
                }
                if (n < 5) {
                    continue;
                }
                double avg = dv / n;
                if (Double.isNaN(lo) || avg < lo) {
                    lo = avg;
                }
                if (Double.isNaN(hi) || avg > hi) {
                    hi = avg;
                }
            }
            if (!Double.isNaN(lo)) {
                double pct = 100.0 * (hi - lo) / lo;
                sb.append(String.format(Locale.US,
                        "目标 %d：duty·V_bus %.3f ~ %.3f，差 %.2f%%  %s\n",
                        target, lo, hi, pct, pct <= 3.0 ? "PASS" : "FAIL(>3%)"));
            }
        }
        double kpStep = shooter.getParams().kp * shooter.getParams().quantStep;
        sb.append(String.format(Locale.US, "-- kp×台阶 = %.4f (%s，判据 ≤0.05) --\n",
                kpStep, kpStep <= 0.05 ? "PASS" : "FAIL"));
        return sb.toString();
    }

    // ---------------------------------------------------------------- 工具

    private String vbusText() {
        double v = readBusVoltage();
        return Double.isNaN(v) ? "NaN" : String.format(Locale.US, "%.2f V", v);
    }

    private static String fmtArray(double[] a) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < a.length; i++) {
            if (i > 0) {
                sb.append('/');
            }
            sb.append(String.format(Locale.US, "%.1f", a[i]));
        }
        return sb.toString();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
