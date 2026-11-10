package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.AdaptiveParkAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.EatAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToOtherShootAreaViaHiveAction;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToStopPose;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToUnvisitedEatPointAvoidHiveAction;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Parameter.AutoParams;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.firstinspires.ftc.teamcode.utility.RobotStateStore;

/**
 * 红队自动阶段主程序。
 *
 * <p><b>动作循环</b>（动作队列空闲时自动投递下一段）：
 * <pre>
 *   绕开 Hive 前往未访问取球点 → 吃球 → 经 Hive 正下方前往另一发射区域 → 回到第一步 …
 * </pre>
 *
 * <p><b>自适应停靠</b>：剩余时间降至 {@link HypParams#PARK_TIME_THRESHOLD_MS}（3 s）时，
 * 立即清空当前动作队列，改投 {@link AdaptiveParkAction}——
 * 检测整机外接矩形是否接触场地边框：接触则开到「不接触」的位置并以此为本局停靠点
 * （停靠点随当时位姿变化，不是固定点）；不接触则原地停止。
 *
 * <p><b>盟友禁区守卫</b>：每帧查询 {@code AutoParams} 中当前时间窗生效的盟友活动区
 * （即我方禁止活动区），一旦整机轮廓闯入，立即清空动作队列并改投「开到最近安全位姿」
 * 的脱困动作；脱困期间暂停常规动作投递，脱离后自动恢复。
 *
 * <p>状态机：{@code TO_EAT_POINT → EATING → TO_SHOOT_AREA → TO_EAT_POINT … → PARKING → STOP}
 */
@Autonomous(name = "AutoRed", group = "Auto")
public class AutoRed extends LinearOpMode {

    /** 自动阶段状态机 */
    private enum AutoState {
        /** 前往未访问取球点（绕开 Hive） */
        TO_EAT_POINT,
        /** 吃球 */
        EATING,
        /** 经 Hive 正下方前往另一发射区域 */
        TO_SHOOT_AREA,
        /** 自适应停靠 */
        PARKING,
        /** 结束 */
        STOP
    }

    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Chassis chassis;
    private Sweeper sweeper;

    /** 队伍颜色（红方，Limelight pipeline 0） */
    private final TeamColor teamColor = TeamColor.RED;

    /** 红队起始位姿 */
    private final Pose2d startPose = HypParams.startPoseRed;

    /** 预瞄准球门：红队 AudienceUp（与 ActionTestBase 默认值一致） */
    private final Pose2d goal = HypParams.RedAudienceUp;

    /** 当前状态，初始为「前往取球点」 */
    private AutoState currentState = AutoState.TO_EAT_POINT;

    /** 是否已投递自适应停靠动作（保证只投递一次） */
    private boolean parkScheduled = false;

    /** 是否正在执行「脱离盟友禁区」的动作（期间暂停常规动作投递） */
    private boolean zoneEscapeActive = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        actionRunner = new ActionRunner();

        // 初始化底盘与定位：Chassis 内部完成 RobotPosition 初始化（含 Limelight + 自适应 EKF）
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, startPose);
        drive = RobotPosition.getInstance().getDrive();
        sweeper = new Sweeper(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team", teamColor);
        telemetry.update();

        waitForStart();

        // ===== 自动阶段开始：清空上一局遗留状态 =====
        RobotStateStore.clear();
        EatPointTracker.clear();

        // 比赛计时从 START 按下后开始（自适应停靠共用同一个计时器）
        ElapsedTime matchTime = new ElapsedTime();

        // 首个动作：绕开 Hive 前往此前未到达的取球点（构造时以当前融合位姿为起点）
        actionRunner.add(new GoToUnvisitedEatPointAvoidHiveAction(teamColor));

        while (opModeIsActive() && currentState != AutoState.STOP) {
            // 更新位姿（每帧唯一一次推进定位器）
            RobotPosition.getInstance().update();

            // 已流逝时间与剩余时间（毫秒）
            long matchMs = (long) matchTime.milliseconds();
            long remainingMs = HypParams.AUTONOMOUS_DURATION_MS - matchMs;

            // ===== 规定时间前 3 秒：中断当前动作，转入自适应停靠 =====
            if (!parkScheduled && remainingMs <= HypParams.PARK_TIME_THRESHOLD_MS) {
                actionRunner.clear();
                actionRunner.add(new AdaptiveParkAction(
                        drive, matchTime,
                        HypParams.AUTONOMOUS_DURATION_MS,
                        HypParams.PARK_TIME_THRESHOLD_MS));
                parkScheduled = true;
                currentState = AutoState.PARKING;
            }

            // ===== 盟友活动区（我方禁止活动区）守卫 =====
            // 生效禁区由 AutoParams 按时间窗给出；整机轮廓一旦闯入，立即中断当前动作，
            // 改投「开到最近安全位姿」的脱困动作，脱困期间不再投递常规动作。
            Pose2d pose = RobotPosition.getInstance().getPose2d();
            AutoParams.AllyZone conflict = parkScheduled || !AutoParams.useAllyZones
                    ? null
                    : AutoParams.firstConflictingZone(teamColor, pose, matchMs);
            if (conflict != null && !zoneEscapeActive) {
                zoneEscapeActive = true;
                actionRunner.clear();
                actionRunner.add(new GoToStopPose(
                        drive, AutoParams.nearestSafePose(teamColor, pose, matchMs)));
            } else if (conflict == null) {
                zoneEscapeActive = false;
            }

            // ===== 动作队列空闲且未在脱困时，投递下一段动作 =====
            if (!actionRunner.isBusy() && !zoneEscapeActive) {
                switch (currentState) {
                    case TO_EAT_POINT:
                        actionRunner.add(new EatAction(sweeper));
                        currentState = AutoState.EATING;
                        break;

                    case EATING:
                        actionRunner.add(new GoToOtherShootAreaViaHiveAction(teamColor, goal));
                        currentState = AutoState.TO_SHOOT_AREA;
                        break;

                    case TO_SHOOT_AREA:
                        actionRunner.add(new GoToUnvisitedEatPointAvoidHiveAction(teamColor));
                        currentState = AutoState.TO_EAT_POINT;
                        break;

                    case PARKING:
                        currentState = AutoState.STOP;
                        break;

                    case STOP:
                    default:
                        break;
                }
            }

            actionRunner.update();

            telemetry.addData("State", currentState);
            telemetry.addData("Remaining(ms)", remainingMs);
            telemetry.addData("Active Zones", AutoParams.activeZoneNames(teamColor, matchMs));
            telemetry.addData("In Forbidden", conflict != null);
            telemetry.addData("Escaping", zoneEscapeActive);
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading(deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }

        // ===== 自动阶段结束：记录结束时刻的位姿，供手动阶段初始化时读回 =====
        RobotStateStore.savePose(RobotPosition.getInstance().getPose2d());

        // 确保底盘与集球器停稳
        sweeper.setStop();
        sweeper.update();
        chassis.stop();
    }
}
