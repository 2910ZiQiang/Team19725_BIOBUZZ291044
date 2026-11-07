package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 自动瞄准 Action：原地转向，使车头始终指向当前 HIVE 抬升侧对应的球门。
 *
 * <p><b>定位来源</b>：每帧直接读取 {@link AdaptiveEKFLocalizer#getPose()}
 * （Pinpoint 里程计 + Limelight MT1 视觉的 EKF 融合位姿，权威位姿来源），
 * 瞄准用的当前位置/航向均以该融合位姿为准。
 *
 * <p><b>球门选择</b>：依据 {@link RobotPosition#getHiveState()} 的跟踪状态
 * （MT1 有观测时更新，无观测时保持上一状态，永不为 UNKNOWN）：
 * <ul>
 *   <li>{@code AUDIENCE_UP}   → 本颜色的 AudienceUp 球门</li>
 *   <li>{@code AUDIENCE_DOWN} → 本颜色的 AudienceDown 球门</li>
 *   <li>{@code MIDDLE}        → 保持上一次锁定的球门（无历史时默认 Up），
 *       继续对齐，由上层状态机决定是否发射</li>
 * </ul>
 *
 * <p><b>控制律</b>：期望航向 {@code des = atan2(goalY-y, goalX-x)}，
 * 航向误差归一化到 [-π, π] 后做 PD 控制，仅下发角速度，平移速度为 0（非阻塞，每帧执行）。
 * 当航向误差与实际角速度连续小于容差并保持 {@link #settleMs} 毫秒后完成；
 * 超过 {@link #timeoutMs} 毫秒也强制结束，避免状态机卡死。
 */
@Config
public class AutoAimAction implements Action {

    /** 航向环 P 增益 (rad/s per rad) */
    public static double aimKp = 3.0;
    /** 航向环 D 增益 (rad/s per rad/s) */
    public static double aimKd = 0.15;
    /** 角速度指令上限 (rad/s) */
    public static double aimMaxOmega = Math.PI;
    /** 航向收敛容差 (rad) */
    public static double headingTol = Math.toRadians(2.0);
    /** 角速度收敛容差 (rad/s)，取自融合定位输出 */
    public static double omegaTol = 0.15;
    /** 误差与角速度同时满足容差的持续时长 (ms)，满足即判定瞄准完成 */
    public static long settleMs = 200;
    /** 超时时间 (ms)，超时强制结束（交由状态机处理） */
    public static long timeoutMs = 3000;

    private final TeamColor teamColor;
    private final MecanumDrive drive;

    private ElapsedTime timer;
    private boolean initialized = false;
    private double lastError = 0;
    private double lastTime = 0;
    private long alignedSinceMs = -1;
    private Pose2d lockedGoal = null;

    public AutoAimAction(TeamColor teamColor) {
        this.teamColor = teamColor;
        this.drive = RobotPosition.getInstance().getDrive();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // 首帧初始化计时（Action 构造时刻早于实际执行，不能在构造函数计时）
        if (!initialized) {
            timer = new ElapsedTime();
            lastTime = timer.seconds();
            initialized = true;
        }

        RobotPosition robotPosition = RobotPosition.getInstance();
        // 瞄准位姿直接取自 AdaptiveEKFLocalizer 融合定位器
        AdaptiveEKFLocalizer localizer = robotPosition.getFusionLocalizer();
        Pose2d pose = localizer.getPose();

        // ---- 依据跟踪的 HIVE 状态选择球门 ----
        Pose2d goal = selectGoal(robotPosition.getHiveState());

        // ---- 期望航向与误差（[-π, π]，始终走最短转向方向）----
        double desiredHeading = Math.atan2(
                goal.position.y - pose.position.y,
                goal.position.x - pose.position.x);
        double error = Math.atan2(
                Math.sin(desiredHeading - pose.heading.toDouble()),
                Math.cos(desiredHeading - pose.heading.toDouble()));

        // ---- PD 控制律 ----
        double now = timer.seconds();
        double dt = now - lastTime;
        lastTime = now;
        if (dt <= 0 || dt > 1.0) {
            dt = 0.02; // 首帧/异常间隔用标称周期兜底，避免微分项发散
        }
        double dError = (error - lastError) / dt;
        lastError = error;

        double omega;
        if (Math.abs(error) <= headingTol) {
            omega = 0.0; // 进入死区即停，避免抖振
        } else {
            omega = aimKp * error + aimKd * dError;
            omega = Math.max(-aimMaxOmega, Math.min(aimMaxOmega, omega));
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), omega));

        // ---- 收敛判定：航向误差与实际角速度同时持续小于容差 ----
        long elapsedMs = (long) timer.milliseconds();
        boolean aligned = Math.abs(error) <= headingTol
                && Math.abs(robotPosition.getOmega()) <= omegaTol;
        if (aligned) {
            if (alignedSinceMs < 0) {
                alignedSinceMs = elapsedMs;
            }
            if (elapsedMs - alignedSinceMs >= settleMs) {
                finish(packet, pose, goal, error, false);
                return false;
            }
        } else {
            alignedSinceMs = -1;
        }

        // ---- 超时保护 ----
        if (elapsedMs >= timeoutMs) {
            finish(packet, pose, goal, error, true);
            return false;
        }

        packet.put("AutoAim", "Aiming");
        packet.put("AutoAim goal x", goal.position.x);
        packet.put("AutoAim goal y", goal.position.y);
        packet.put("AutoAim error(deg)", Math.toDegrees(error));
        packet.put("AutoAim hiveState", robotPosition.getHiveState().name());
        return true;
    }

    /**
     * 按队伍颜色与 HIVE 抬升状态选择目标球门。
     * MIDDLE 时保持上一次锁定的球门；无历史时默认 AudienceUp。
     */
    //todo: 重新考虑middle的目标锁定逻辑
    private Pose2d selectGoal(MT1Localizer.HiveState hiveState) {
        Pose2d goal;
        if (teamColor == TeamColor.RED) {
            goal = (hiveState == MT1Localizer.HiveState.AUDIENCE_DOWN)
                    ? HypParams.RedAudienceDown : HypParams.RedAudienceUp;
        } else {
            goal = (hiveState == MT1Localizer.HiveState.AUDIENCE_DOWN)
                    ? HypParams.BlueAudienceDown : HypParams.BlueAudienceUp;
        }
        // AUDIENCE_UP 与 MIDDLE 均选 Up；MIDDLE 时若曾锁定 Down 则保持锁定，避免目标来回跳变
        if (hiveState == MT1Localizer.HiveState.MIDDLE && lockedGoal != null) {
            goal = lockedGoal;
        }
        lockedGoal = goal;
        return goal;
    }

    /** 结束时主动刹停并输出最终遥测。 */
    private void finish(TelemetryPacket packet, Pose2d pose, Pose2d goal,
                        double error, boolean timeout) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        packet.put("AutoAim", timeout ? "Timeout" : "Aligned");
        packet.put("AutoAim final error(deg)", Math.toDegrees(error));
        packet.put("AutoAim goal x", goal.position.x);
        packet.put("AutoAim goal y", goal.position.y);
        packet.put("AutoAim pose x", pose.position.x);
        packet.put("AutoAim pose y", pose.position.y);
    }
}
