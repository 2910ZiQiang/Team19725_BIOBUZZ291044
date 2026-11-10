package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import org.firstinspires.ftc.teamcode.utility.DpadStepper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.Algebra.LinearStatistics;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 瞄准线性回归模型的自动标定台。
 *
 * <p>操作手手动驾驶底盘、手动控制集球器与炮台，按不同射程/仰角/飞轮转速逐发打门；
 * 每次扳机释放（上升沿）自动记录「位姿 + 射程 + 指令/实际转速 + 仰角 + 瞄准偏差 + 是否已稳定」，
 * 随后用主控按键把某发标记为「命中」。程序按仰角档位对命中样本做一元线性回归
 * {@code speed = slope·range + intercept}，并用相邻档实测射程推断各档的
 * {@code minRange/maxRange}，最终以 telemetry（FTC Dashboard Logging）输出建议值。
 *
 * <p>本程序<b>不</b>写回 {@code Shooter} 的静态参数：结果需人工抄回静态块。
 *
 * <p><b>主控（gamepad1）</b>
 * <ul>
 *   <li>左摇杆 → 底盘平移；右摇杆 X → 底盘旋转</li>
 *   <li>LB / RB → 集球器吃球 / 输出</li>
 *   <li>A / B → 把最近一发「大球 / 小球」标为命中（未标记的样本不参与回归）</li>
 *   <li>X → 位姿重置到本方的 {@link HypParams#ResetPoseRed} / {@link HypParams#ResetPoseBlue}</li>
 *   <li>Y → 撤销最近一条样本（两个发射器按时间取最新）</li>
 * </ul>
 *
 * <p><b>副控（gamepad2）</b>
 * <ul>
 *   <li>左摇杆 X → 平转<b>角速度</b>（摇杆回中即停在当前角度），与底盘的速度控制同构</li>
 *   <li>X → 循环平转模式 RELATIVE → ABSOLUTE → GOAL（相对朝向 / 绝对朝向 / 与球门夹角）</li>
 *   <li>方向键 上/下 → 大球目标转速 ±{@link #speedStep}；左/右 → 小球目标转速 ±{@link #speedStep}</li>
 *   <li>RB / LB → 大球 / 小球扳机（按住释放；每次上升沿记一发）</li>
 * </ul>
 * 仰角舵机位置由 Dashboard 的 {@link #targetPitchBig} / {@link #targetPitchSmall} 直接设置
 * （与 {@code TurretManualTest} 一致，手柄不控制仰角）；标定时请把它们设成现有档位的
 * {@code pitchPos}，否则该发样本匹配不到档位、不参与回归。
 *
 * <p><b>现场流程</b>：Dashboard → Logging → Start → 打点 → Stop → 导出 CSV；
 * 同时检查 Driver Station 上的 {@code >>>} 建议行，按格式抄回 {@code Shooter} 静态块。
 */
@Config
@TeleOp(name = "AimModelCalibration", group = "Tuning")
public class AimModelCalibrationTuner extends LinearOpMode {

    // ==================== Dashboard 可调参数 ====================

    /** 球门与定位 pipeline 使用的颜色；<b>改动后需重新 Init</b>（Chassis 在 init 阶段读取） */
    public static TeamColor teamColor = TeamColor.RED;

    /** 样本参与回归的最大瞄准偏差（度）；超过视为"没瞄准"，不参与回归 */
    public static double aimToleranceDeg = 2.0;
    /** true = 只接受发射瞬间飞轮已稳定（{@code Shooter.isReady()}）的样本 */
    public static boolean requireSettled = true;

    /** 手动平转角速度上限（度/秒）；摇杆满偏对应此速率，回中即停止 */
    public static double maxYawRateDegPerSec = 120.0;

    /** 大球仰角舵机目标位置 [0,1]，由 Dashboard 直接设置 */
    public static double targetPitchBig = 0.5;
    /** 小球仰角舵机目标位置 [0,1]，由 Dashboard 直接设置 */
    public static double targetPitchSmall = 0.5;

    /** 俯仰舵机位置匹配到现有档位的容差（舵机单位） */
    public static double pitchMatchTol = 0.02;

    /** 每次方向键的转速增量 (tick/s) */
    public static int speedStep = 100;
    /** 长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 120;

    /** 拟合所需最小射程跨度（英寸） */
    public static double minSpanIn = 8.0;
    /** 首/末档边界向观测极值外扩的裕量（英寸） */
    public static double edgeMarginIn = 5.0;
    /** 每个发射器保留的最大样本数（超出丢最旧） */
    public static int maxShotsKept = 60;

    // ==================== 部件 ====================

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;

    // ==================== 运行时状态 ====================

    private int targetSpeedBig = 0;
    private int targetSpeedSmall = 0;

    private final DpadStepper vertical = new DpadStepper();
    private final DpadStepper horizontal = new DpadStepper();

    private final List<ShotSample> shotsBig = new ArrayList<>();
    private final List<ShotSample> shotsSmall = new ArrayList<>();

    private boolean lastTriggerBig = false;
    private boolean lastTriggerSmall = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastX2 = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ActionRunner actionRunner = new ActionRunner();
        // 初始位姿与 X 重同步使用同一个参照点：上电时请把车放在 ResetPose 位置
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, resetPose());
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        sweeper.setStop();
        sweeper.update();
        turret.stop();

        telemetry.addLine("AimModelCalibration: team=" + teamColor);
        telemetry.addLine("GP1: drive | LB/RB eat/output | A/B mark HIT big/small | X reset pose | Y undo");
        telemetry.addLine("GP2: LstickX yaw RATE | X cycle yaw mode | Dpad UD big speed | Dpad LR small speed | RB/LB trigger");
        telemetry.addLine("Pitch is set on Dashboard (targetPitchBig/Small), not by sticks");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- 定位（单帧推进点）----
            Pose2d pose = RobotPosition.getInstance().update();

            // ---- 底盘与集球器 ----
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.left_bumper) {
                sweeper.setEat();
            } else if (gamepad1.right_bumper) {
                sweeper.setOutput();
            } else {
                sweeper.setStop();
            }

            // ---- 平转角速度：摇杆回中即停在当前角度（模式由 GP2.X 循环切换）----
            double yawRate = -gamepad2.left_stick_x * Math.toRadians(maxYawRateDegPerSec);
            if (gamepad2.x && !lastX2) {
                turret.cycleManualYawMode();
            }
            lastX2 = gamepad2.x;

            // ---- 飞轮目标转速步进 ----
            int v = vertical.step(gamepad2.dpad_down, gamepad2.dpad_up, repeatPeriodMs);
            int h = horizontal.step(gamepad2.dpad_left, gamepad2.dpad_right, repeatPeriodMs);
            targetSpeedBig = clamp(targetSpeedBig + v * speedStep,
                    0, Shooter.BIG_PARAMS.flywheel.speedMax);
            targetSpeedSmall = clamp(targetSpeedSmall + h * speedStep,
                    0, Shooter.SMALL_PARAMS.flywheel.speedMax);

            // ---- 下发炮台目标 ----
            boolean triggerBig = gamepad2.right_bumper;
            boolean triggerSmall = gamepad2.left_bumper;
            turret.updateManual(yawRate, targetPitchBig, targetPitchSmall,
                    targetSpeedBig, targetSpeedSmall, triggerBig, triggerSmall);
            sweeper.update();

            // ---- 扳机上升沿：记一发 ----
            if (triggerBig && !lastTriggerBig) {
                recordShot(true, pose);
            }
            if (triggerSmall && !lastTriggerSmall) {
                recordShot(false, pose);
            }
            lastTriggerBig = triggerBig;
            lastTriggerSmall = triggerSmall;

            // ---- 命中标记 / 撤销 / 位姿重同步 ----
            if (gamepad1.a && !lastA) {
                markLastHit(shotsBig);
            }
            if (gamepad1.b && !lastB) {
                markLastHit(shotsSmall);
            }
            if (gamepad1.y && !lastY) {
                undoLatest();
            }
            if (gamepad1.x && !lastX) {
                RobotPosition.getInstance().ResetPoseTo(resetPose());
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastY = gamepad1.y;
            lastX = gamepad1.x;

            // ---- 回归 + 边界推断（n 很小，每帧重算开销可忽略）----
            BandFit[] fitsBig = fitBands(shotsBig, Shooter.BIG_PARAMS);
            BandFit[] fitsSmall = fitBands(shotsSmall, Shooter.SMALL_PARAMS);

            // ---- 遥测 ----
            telemetry.addLine("--- AimModelCalibration (team " + teamColor + ") ---");
            emitStatus(pose);
            emitBandFits("Big", fitsBig);
            emitBandFits("Small", fitsSmall);
            emitLastShot("Big", shotsBig);
            emitLastShot("Small", shotsSmall);
            addRecentShotLines("Big", shotsBig);
            addRecentShotLines("Small", shotsSmall);
            addSuggestionLines("BIG", fitsBig);
            addSuggestionLines("SMALL", fitsSmall);
            telemetry.update();
        }

        // ---- 结束：再算一次最终建议（不再访问硬件）----
        chassis.stop();
        turret.stop();
        BandFit[] finalBig = fitBands(shotsBig, Shooter.BIG_PARAMS);
        BandFit[] finalSmall = fitBands(shotsSmall, Shooter.SMALL_PARAMS);
        telemetry.addLine("========== FINAL ==========");
        addSuggestionLines("BIG", finalBig);
        addSuggestionLines("SMALL", finalSmall);
        telemetry.update();
    }

    // ==================== 样本采集 ====================

    /** 单发样本；{@link #hit} 由操作手事后标记（未标记的样本不参与回归） */
    private static final class ShotSample {
        final long timeMs;
        final double range;
        final int targetSpeed;
        final double actualSpeed;
        final double pitch;
        final double aimErrorDeg;
        final boolean ready;
        final double busV;
        boolean hit = false;

        ShotSample(long timeMs, double range, int targetSpeed, double actualSpeed,
                   double pitch, double aimErrorDeg, boolean ready, double busV) {
            this.timeMs = timeMs;
            this.range = range;
            this.targetSpeed = targetSpeed;
            this.actualSpeed = actualSpeed;
            this.pitch = pitch;
            this.aimErrorDeg = aimErrorDeg;
            this.ready = ready;
            this.busV = busV;
        }
    }

    private void recordShot(boolean big, Pose2d pose) {
        Pose2d goal = goalPose();
        Vector2D shooterPos = shooterPosition(big, pose);
        double range = Vector2D.getDistance(
                shooterPos, new Vector2D(goal.position.x, goal.position.y));
        if (!(range > 0 && range <= 200)) {
            return;
        }

        Shooter shooter = big ? turret.shooterBig : turret.shooterSmall;
        double bearing = Math.atan2(goal.position.y - shooterPos.getY(),
                goal.position.x - shooterPos.getX());
        double aimErrorDeg = Math.toDegrees(Vector2D.normalizeAngle(
                bearing - (pose.heading.toDouble() + turret.yawController.getCurrentYaw())));

        ShotSample sample = new ShotSample(
                System.currentTimeMillis(),
                range,
                shooter.getTargetSpeed(),
                shooter.getVelocity(),
                shooter.getTargetPitch(),
                aimErrorDeg,
                shooter.isReady(),
                shooter.getBusVoltage());

        List<ShotSample> list = big ? shotsBig : shotsSmall;
        list.add(sample);
        while (list.size() > maxShotsKept) {
            list.remove(0);
        }
    }

    /** @return 发射器中心（车体系靠 {@code RelPos} 偏置，按航向旋到场地系） */
    private static Vector2D shooterPosition(boolean big, Pose2d pose) {
        Vector2D rel = big ? Turret.RelPosBig : Turret.RelPosSmall;
        Vector2D rotated = Vector2D.rotate(rel, pose.heading.toDouble());
        return new Vector2D(pose.position.x + rotated.getX(),
                pose.position.y + rotated.getY());
    }

    /** @return {@link RobotPosition} 由 HIVE 状态解出的当前球门（炮台 GOAL 模式使用同一来源） */
    private Pose2d goalPose() {
        return RobotPosition.getInstance().getGoalPose();
    }

    private Pose2d resetPose() {
        return teamColor == TeamColor.RED ? HypParams.ResetPoseRed : HypParams.ResetPoseBlue;
    }

    private static void markLastHit(List<ShotSample> shots) {
        if (shots.isEmpty()) {
            return;
        }
        shots.get(shots.size() - 1).hit = true;
    }

    private void undoLatest() {
        ShotSample lastBig = shotsBig.isEmpty() ? null : shotsBig.get(shotsBig.size() - 1);
        ShotSample lastSmall = shotsSmall.isEmpty() ? null : shotsSmall.get(shotsSmall.size() - 1);
        if (lastBig == null && lastSmall == null) {
            return;
        }
        if (lastSmall == null || (lastBig != null && lastBig.timeMs >= lastSmall.timeMs)) {
            shotsBig.remove(shotsBig.size() - 1);
        } else {
            shotsSmall.remove(shotsSmall.size() - 1);
        }
    }

    // ==================== 回归与边界推断 ====================

    /** 单个仰角档位的拟合结果与建议边界 */
    private static final class BandFit {
        final int index;
        final double pitchPos;
        final boolean valid;
        final int n;
        final double slope;
        final double intercept;
        final double r2;
        final double obsMin;
        final double obsMax;
        final double origMinRange;
        final double origMaxRange;
        final double origSlope;
        final double origIntercept;
        double minRange;
        double maxRange;
        String note;

        BandFit(int index, double pitchPos, boolean valid, int n, double slope, double intercept,
                double r2, double obsMin, double obsMax, double origMinRange, double origMaxRange,
                double origSlope, double origIntercept, String note) {
            this.index = index;
            this.pitchPos = pitchPos;
            this.valid = valid;
            this.n = n;
            this.slope = slope;
            this.intercept = intercept;
            this.r2 = r2;
            this.obsMin = obsMin;
            this.obsMax = obsMax;
            this.origMinRange = origMinRange;
            this.origMaxRange = origMaxRange;
            this.origSlope = origSlope;
            this.origIntercept = origIntercept;
            this.minRange = origMinRange;
            this.maxRange = origMaxRange;
            this.note = note;
        }

        double suggestedSlope() {
            return valid ? slope : origSlope;
        }

        double suggestedIntercept() {
            return valid ? intercept : origIntercept;
        }

        double suggestedMinRange() {
            return minRange;
        }

        double suggestedMaxRange() {
            return maxRange;
        }
    }

    private static BandFit[] fitBands(List<ShotSample> samples, Shooter.Params params) {
        Shooter.PitchModel[] models = params.pitchModels;
        if (models == null || models.length == 0) {
            return new BandFit[0];
        }

        BandFit[] fits = new BandFit[models.length];
        for (int i = 0; i < models.length; i++) {
            Shooter.PitchModel model = models[i];
            LinearStatistics stats = new LinearStatistics();
            int n = 0;
            double obsMin = Double.POSITIVE_INFINITY;
            double obsMax = Double.NEGATIVE_INFINITY;

            for (ShotSample s : samples) {
                if (!s.hit) {
                    continue;
                }
                if (requireSettled && !s.ready) {
                    continue;
                }
                if (Math.abs(s.aimErrorDeg) > aimToleranceDeg) {
                    continue;
                }
                if (Math.abs(s.pitch - model.pitchPos) > pitchMatchTol) {
                    continue;
                }
                stats.add(s.range, s.targetSpeed);
                n++;
                if (s.range < obsMin) {
                    obsMin = s.range;
                }
                if (s.range > obsMax) {
                    obsMax = s.range;
                }
            }

            double slope = stats.weight();
            double intercept = stats.bias();
            double r2 = stats.calculateR2();
            boolean valid = true;
            String note = "";
            if (n < 2) {
                valid = false;
                note = "样本不足(n<2)";
            } else if (obsMax - obsMin < minSpanIn) {
                valid = false;
                note = "射程跨度不足";
            } else if (slope <= 0) {
                valid = false;
                note = "斜率非正";
            } else if (slope * obsMax + intercept > params.flywheel.speedMax) {
                note = "末段将撞speedMax";
            }

            fits[i] = new BandFit(i, model.pitchPos, valid, n, slope, intercept, r2,
                    obsMin, obsMax, model.minRange, model.maxRange,
                    model.speedSlope, model.speedIntercept, note);
        }

        // ---- 相邻档边界：实测射程极值中点，仅当两档都 valid ----
        for (int i = 0; i + 1 < fits.length; i++) {
            if (fits[i].valid && fits[i + 1].valid) {
                double boundary = 0.5 * (fits[i].obsMax + fits[i + 1].obsMin);
                fits[i].maxRange = boundary;
                fits[i + 1].minRange = boundary;
            }
        }
        // ---- 首/末档外扩 ----
        if (fits[0].valid) {
            fits[0].minRange = Math.max(0.0, fits[0].obsMin - edgeMarginIn);
        }
        BandFit last = fits[fits.length - 1];
        if (last.valid) {
            last.maxRange = last.obsMax + edgeMarginIn;
        }
        // ---- 边界自检：数据矛盾时回退原值 ----
        for (BandFit f : fits) {
            if (!(f.minRange < f.maxRange)) {
                f.minRange = f.origMinRange;
                f.maxRange = f.origMaxRange;
                f.note = f.note.isEmpty() ? "边界矛盾，保留原值" : f.note + "；边界矛盾，保留原值";
            }
        }
        return fits;
    }

    // ==================== 遥测 ====================

    private void emitStatus(Pose2d pose) {
        telemetry.addData("AimCalPoseX", "%.1f", pose.position.x);
        telemetry.addData("AimCalPoseY", "%.1f", pose.position.y);
        telemetry.addData("AimCalPoseHeading", "%.1f", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("AimCalGoalX", "%.1f", goalPose().position.x);
        telemetry.addData("AimCalGoalY", "%.1f", goalPose().position.y);
        telemetry.addData("AimCalYawMode", turret.getManualYawMode());
        telemetry.addData("AimCalCmdYawDeg", "%.1f",
                Math.toDegrees(turret.yawController.getTargetYaw()));
        telemetry.addData("AimCalActYawDeg",
                "%.1f", Math.toDegrees(turret.yawController.getCurrentYaw()));
        telemetry.addData("AimCalRelBigX", "%.2f", Turret.RelPosBig.getX());
        telemetry.addData("AimCalRelBigY", "%.2f", Turret.RelPosBig.getY());
        telemetry.addData("AimCalRelSmallX", "%.2f", Turret.RelPosSmall.getX());
        telemetry.addData("AimCalRelSmallY", "%.2f", Turret.RelPosSmall.getY());
        telemetry.addData("AimCalShotsBig_total", shotsBig.size());
        telemetry.addData("AimCalShotsBig_hits", countHits(shotsBig));
        telemetry.addData("AimCalShotsSmall_total", shotsSmall.size());
        telemetry.addData("AimCalShotsSmall_hits", countHits(shotsSmall));
        telemetry.addData("AimCalTgtSpeedBig", targetSpeedBig);
        telemetry.addData("AimCalTgtSpeedSmall", targetSpeedSmall);
        telemetry.addData("AimCalPitchBig", "%.3f", targetPitchBig);
        telemetry.addData("AimCalPitchSmall", "%.3f", targetPitchSmall);
    }

    private void emitBandFits(String tag, BandFit[] fits) {
        for (BandFit f : fits) {
            String key = "AimCal" + tag + "B" + f.index + "_";
            telemetry.addData(key + "n", f.n);
            telemetry.addData(key + "valid", f.valid ? 1 : 0);
            telemetry.addData(key + "slope", "%.2f", f.suggestedSlope());
            telemetry.addData(key + "intercept", "%.1f", f.suggestedIntercept());
            telemetry.addData(key + "r2", "%.3f", f.r2);
            telemetry.addData(key + "obsMin", f.n > 0 ? String.format(Locale.US, "%.1f", f.obsMin) : "NaN");
            telemetry.addData(key + "obsMax", f.n > 0 ? String.format(Locale.US, "%.1f", f.obsMax) : "NaN");
            telemetry.addData(key + "minRange", "%.1f", f.suggestedMinRange());
            telemetry.addData(key + "maxRange", "%.1f", f.suggestedMaxRange());
        }
    }

    private void emitLastShot(String tag, List<ShotSample> shots) {
        if (shots.isEmpty()) {
            return;
        }
        ShotSample s = shots.get(shots.size() - 1);
        String key = "AimCalLast" + tag + "_";
        telemetry.addData(key + "range", "%.1f", s.range);
        telemetry.addData(key + "targetSpeed", s.targetSpeed);
        telemetry.addData(key + "actualSpeed", "%.0f", s.actualSpeed);
        telemetry.addData(key + "pitch", "%.3f", s.pitch);
        telemetry.addData(key + "aimErr", "%.1f", s.aimErrorDeg);
        telemetry.addData(key + "ready", s.ready ? 1 : 0);
        telemetry.addData(key + "hit", s.hit ? 1 : 0);
    }

    private void addRecentShotLines(String tag, List<ShotSample> shots) {
        telemetry.addLine("--- " + tag + " recent ---");
        for (int i = Math.max(0, shots.size() - 4); i < shots.size(); i++) {
            ShotSample s = shots.get(i);
            telemetry.addLine(String.format(Locale.US,
                    "%s r=%.1f v=%d/%.0f p=%.3f err=%.1f %s",
                    s.hit ? "HIT " : "----",
                    s.range, s.targetSpeed, s.actualSpeed, s.pitch, s.aimErrorDeg,
                    s.ready ? "RDY" : "   "));
        }
    }

    private void addSuggestionLines(String tag, BandFit[] fits) {
        if (fits.length == 0) {
            telemetry.addLine(">>> " + tag + ": pitchModels 为空，无可标定档位");
            return;
        }
        for (BandFit f : fits) {
            String note = f.note.isEmpty() ? "" : "  (" + f.note + ")";
            telemetry.addLine(String.format(Locale.US,
                    ">>> %s_MODEL[%d] new PitchModel(%.3f, %.1f, %.1f, %.2f, %.1f)  n=%d R2=%.2f%s",
                    tag, f.index, f.pitchPos,
                    f.suggestedMinRange(), f.suggestedMaxRange(),
                    f.suggestedSlope(), f.suggestedIntercept(),
                    f.n, f.r2, note));
        }
    }

    private static int countHits(List<ShotSample> shots) {
        int n = 0;
        for (ShotSample s : shots) {
            if (s.hit) {
                n++;
            }
        }
        return n;
    }

    // ==================== 工具 ====================

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** 方向键步进器见 {@link DpadStepper} */
}
