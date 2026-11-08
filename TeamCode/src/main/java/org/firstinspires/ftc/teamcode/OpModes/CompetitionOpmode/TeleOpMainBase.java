package org.firstinspires.ftc.teamcode.OpModes.CompetitionOpmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.RobotStateStore;

/**
 * 比赛主程序（TeleOp）的公共骨架：底盘 + 吃球 + 炮台（自动瞄准 / 手动瞄准）。
 *
 * <p>颜色相关的差异（默认停车位姿、车头瞄准的球门）由子类通过
 * {@link #defaultStopPose()} 与 {@link #aimGoal(boolean)} 提供。
 *
 * <p><b>主控（gamepad1）</b>
 * <ul>
 *   <li>左摇杆 → 底盘平移；右摇杆 X → 底盘旋转</li>
 *   <li>LT / RT → 按住时车头自动指向<b>炮台当前球门</b>（与自动瞄准同一来源，见
 *       {@link #chassisAimGoal(boolean)}），松开恢复常规驾驶</li>
 *   <li>X → 切换无头模式；A → 重置定位到本方 ResetPose</li>
 *   <li>LB → 吃球；RB → 吐出</li>
 * </ul>
 *
 * <p><b>副控（gamepad2）</b>
 * <ul>
 *   <li>B（按下） → 切换 <b>自动瞄准 / 手动瞄准</b>，默认自动瞄准</li>
 *   <li>Y（按下） → 切换预载开关（仅自动瞄准生效）</li>
 *   <li>自动瞄准：A（按住） → 允许发射；LB → 小球强制释放；RB → 大球强制释放</li>
 *   <li>X（按下） → 循环手动平转模式 RELATIVE → ABSOLUTE → GOAL（仅手动瞄准生效）</li>
 *   <li>手动瞄准：左摇杆 Y → 小球仰角，右摇杆 Y → 大球仰角；
 *       左摇杆 X → 平转角速度（回中停在当前角度）；十字键 上/下 → 大球转速步进，
 *       左/右 → 小球转速步进；LB / RB → 小球 / 大球扳机</li>
 * </ul>
 *
 * <p><b>送球联动</b>：任一发射器释放扳机时，吃球器自动 {@code setGiveArtifact()} 送球；
 * 停止发射后停转（主控 LB/RB 手动吃球/吐出仍然优先除外）。
 */
@Config
public abstract class TeleOpMainBase extends LinearOpMode {

    // ==================== 超参数（Dashboard 可调） ====================

    /** 手动瞄准：每次方向键步进的飞轮转速增量 (tick/s) */
    public static int speedStep = 100;
    /** 手动瞄准：长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 120;
    /** 手动瞄准：平转角速度上限（度/秒），摇杆满偏对应此速率 */
    public static double maxYawRateDegPerSec = 120.0;

    // ==================== 部件 ====================

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;

    private final TeamColor teamColor;

    // ==================== 手动瞄准运行时状态 ====================

    private int targetSpeedBig = 0;
    private int targetSpeedSmall = 0;
    private final DpadStepper vertical = new DpadStepper();
    private final DpadStepper horizontal = new DpadStepper();

    /** true = 自动瞄准（炮台按定位解算），false = 手动瞄准（副控直接给定目标） */
    private boolean autoAim = true;
    /** 预载开关，仅自动瞄准生效；Y 切换 */
    private boolean preloadEnabled = true;

    protected TeleOpMainBase(TeamColor teamColor) {
        this.teamColor = teamColor;
    }

    /** 无持久化位姿时使用的默认停车位姿 */
    protected abstract Pose2d defaultStopPose();

    /**
     * 无定位实例时车头瞄准的兜底球门。
     *
     * <p><b>正常情况下本方法不参与瞄准</b>：车头与炮台必须指向同一个球门，
     * 因此 {@link #chassisAimGoal(boolean)} 优先取 {@link RobotPosition#getGoalPose()}
     * （炮台自动瞄准用的同一个来源）。只有 {@link RobotPosition} 尚未初始化时才退回到这里，
     * 而那种情况下炮台自己也会走"无有效定位"的安全分支，不存在两条瞄准链。
     *
     * @param leftTrigger true = 左扳机 (LT)，false = 右扳机 (RT)
     */
    protected abstract Pose2d aimGoal(boolean leftTrigger);

    /**
     * 车头瞄准目标：与炮台自动瞄准取<b>同一个</b>球门。
     *
     * <p>历史实现里车头按固定的 Up/Down 门转向（由 LT / RT 选择），而炮台读的是
     * {@link RobotPosition#getGoalPose()} 按 HIVE 状态自动选择的门。两者分居场地两端时，
     * 会出现"车头对着 A 门、炮口对着 B 门"的矛盾状态，操作手无法从画面判断以哪个为准。
     * 现在统一为同一来源；LT / RT 保留为"按住即让车头指向当前球门"。
     *
     * @param leftTrigger true = 左扳机 (LT)，false = 右扳机 (RT)，仅在无定位实例时用于兜底
     * @return 车头瞄准目标
     */
    protected Pose2d chassisAimGoal(boolean leftTrigger) {
        RobotPosition robotPosition = RobotPosition.getInstanceOrNull();
        Pose2d goal = (robotPosition != null) ? robotPosition.getGoalPose() : null;
        return (goal != null) ? goal : aimGoal(leftTrigger);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- 读回自动阶段结束时刻持久化的机器状态作为初始位姿 ----
        Pose2d storedPose = RobotStateStore.loadPose();
        Pose2d initPose = (storedPose != null) ? storedPose : defaultStopPose();

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, initPose);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.stop();

        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Team Color", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
        telemetry.addData("Init Pose Source", storedPose != null ? "Stored robot state" : "Default StopPose");
        telemetry.addData("Init Pose", "%.2f, %.2f, %.1f deg",
                initPose.position.x, initPose.position.y, Math.toDegrees(initPose.heading.toDouble()));
        telemetry.addData("--- P1: 底盘 / 吃球 ---", "");
        telemetry.addData("Left Stick", "Chassis Drive");
        telemetry.addData("Right Stick X", "Chassis Rotation (disabled while aiming)");
        telemetry.addData("Left / Right Trigger", "Aim current goal, shared with turret (hold)");
        telemetry.addData("X", "Toggle No-Head Mode");
        telemetry.addData("A", "Reset Pose to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red") + " ResetPose");
        telemetry.addData("Left Bumper", "Sweeper Eat");
        telemetry.addData("Right Bumper", "Sweeper Output");
        telemetry.addData("--- P2: 炮台 ---", "");
        telemetry.addData("B", "Toggle Auto / Manual Aim (auto by default)");
        telemetry.addData("Y", "Toggle Preload (auto aim only)");
        telemetry.addData("Auto: A", "Fire (hold)");
        telemetry.addData("Auto: LB / RB", "Force release small / big");
        telemetry.addData("Manual: Sticks", "LstickY small pitch | RstickY big pitch | LstickX yaw rate");
        telemetry.addData("Manual: X", "Cycle yaw mode REL / ABS / GOAL");
        telemetry.addData("Manual: Dpad", "U/D big speed | L/R small speed");
        telemetry.addData("Manual: LB / RB", "Small / big trigger");
        telemetry.update();
        telemetry.setMsTransmissionInterval(250);

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            // ======== P1：底盘 ========
            if (gamepad1.left_trigger > 0.5) {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, chassisAimGoal(true));
            } else if (gamepad1.right_trigger > 0.5) {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, chassisAimGoal(false));
            } else {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            if (gamepad1.xWasReleased()) {
                chassis.exchangeUseNoHeadMode();
            }

            if (gamepad1.aWasReleased()) {
                Pose2d resetPose = (teamColor == TeamColor.BLUE) ?
                        HypParams.ResetPoseBlue : HypParams.ResetPoseRed;
                RobotPosition.getInstance().ResetPoseTo(resetPose);
            }

            // ======== P1：吃球 ========
            if (gamepad1.left_bumper) {
                sweeper.setEat();
            }
            if (gamepad1.right_bumper) {
                sweeper.setOutput();
            }

            // ======== P2：炮台 ========
            if (gamepad2.bWasReleased()) {
                autoAim = !autoAim;
            }
            if (gamepad2.yWasReleased()) {
                preloadEnabled = !preloadEnabled;
            }
            if (gamepad2.xWasReleased()) {
                turret.cycleManualYawMode();
            }

            Turret.FireReady ready;
            if (autoAim) {
                // 自动瞄准：A 按住即允许发射；LB/RB 强制释放小球/大球
                //todo：同时停止底盘
                ready = turret.update(gamepad2.a, preloadEnabled,
                        gamepad2.right_bumper, gamepad2.left_bumper);
            } else {
                ready = updateManualTurret();
            }

            // ======== 扳机释放 → 吃球器送球；停止发射后停转 ========
            // 自动瞄准以 Turret 实际释放状态为准：它含 fireHoldTime 的最短保持窗口，
            // 否则就绪状态在档位边界逐帧抖动会让送球跟着断续，打断装填。
            // 手动瞄准的扳机就是副控按键本身。
            boolean feeding = autoAim
                    ? (turret.isBigFiring() || turret.isSmallFiring())
                    : (gamepad2.right_bumper || gamepad2.left_bumper);
            if (feeding) {
                sweeper.setGiveArtifact();
            } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                // 未发射且主控没有手动吃球/吐出：停转
                sweeper.setStop();
            }

            // ======== 更新 & 遥测 ========
            actionRunner.update();
            sweeper.update();

            telemetry.addData("Team", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());
            telemetry.addData("Aim mode", autoAim ? "AUTO" : "MANUAL");
            if (autoAim) {
                telemetry.addData("Preload", preloadEnabled ? "ON" : "OFF");
            }
            // 车头与炮台共用的球门（单一来源，便于操作手确认"打的是哪一个门"）
            Pose2d sharedGoal = RobotPosition.getInstance().getGoalPose();
            telemetry.addData("Aim goal (shared)", "(%+.1f, %+.1f)",
                    sharedGoal.position.x, sharedGoal.position.y);
            telemetry.addData("Fire ready big", ready.big);
            telemetry.addData("Fire ready small", ready.small);

            telemetry.addData("Pose X", "%.2f in", RobotPosition.getInstance().getX());
            telemetry.addData("Pose Y", "%.2f in", RobotPosition.getInstance().getY());
            telemetry.addData("Pose Theta", "%.2f deg", Math.toDegrees(RobotPosition.getInstance().getTheta()));

            chassis.telemetry();
            sweeper.setTelemetry();
            turret.setTelemetry();

            if (!autoAim) {
                telemetry.addLine("--- manual targets ---");
                telemetry.addData("Manual yaw mode", turret.getManualYawMode());
                telemetry.addData("Target speed big (tick/s)", targetSpeedBig);
                telemetry.addData("Target speed small (tick/s)", targetSpeedSmall);
                telemetry.addData("speedStep", speedStep);
            }

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        chassis.stop();
        sweeper.setStop();
        sweeper.update();
        turret.stop();
    }

    /**
     * 手动瞄准：与 {@code TurretManualTest} 相同的控制方式，直接给定 yaw / 仰角 / 转速 / 扳机。
     *
     * @return 本帧两个发射器是否具备发射条件
     */
    private Turret.FireReady updateManualTurret() {
        // ---- 摇杆：上推时 axis = -1，取负号使"上 → 增大" ----
        double pitchSmall = axisToRange(-gamepad2.left_stick_y,
                Shooter.SMALL_PARAMS.pitchMin, Shooter.SMALL_PARAMS.pitchMax);
        double pitchBig = axisToRange(-gamepad2.right_stick_y,
                Shooter.BIG_PARAMS.pitchMin, Shooter.BIG_PARAMS.pitchMax);
        // 推右（axis = +1）→ 顺时针，对应负角速度；摇杆回中即停在当前角度
        double yawRate = -gamepad2.left_stick_x * Math.toRadians(maxYawRateDegPerSec);

        // ---- 十字键步进飞轮目标转速 ----
        int v = vertical.step(gamepad2.dpad_down, gamepad2.dpad_up, repeatPeriodMs);
        int h = horizontal.step(gamepad2.dpad_left, gamepad2.dpad_right, repeatPeriodMs);
        targetSpeedBig = clamp(targetSpeedBig + v * speedStep, 0, Shooter.BIG_PARAMS.flywheel.speedMax);
        targetSpeedSmall = clamp(targetSpeedSmall + h * speedStep, 0, Shooter.SMALL_PARAMS.flywheel.speedMax);

        // ---- 扳机：按住释放（LB 小球 / RB 大球） ----
        return turret.updateManual(yawRate, pitchBig, pitchSmall,
                targetSpeedBig, targetSpeedSmall, gamepad2.right_bumper, gamepad2.left_bumper);
    }

    /** 摇杆位置 [-1, 1] 线性映射到 [lo, hi] */
    private static double axisToRange(double axis, double lo, double hi) {
        double t = Math.max(0, Math.min(1, (axis + 1) / 2));
        return lo + t * (hi - lo);
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
