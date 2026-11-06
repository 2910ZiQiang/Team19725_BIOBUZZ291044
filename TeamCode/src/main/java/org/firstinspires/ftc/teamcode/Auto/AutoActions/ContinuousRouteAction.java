package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.firstinspires.ftc.teamcode.utility.HiveRoutePlanner;

import java.util.List;

/**
 * 【激进版】把「前往取球点 + 路过式吃球 + 前往另一发射区域」合成 <b>一个 Action</b>，
 * 且全程<b>只有一条轨迹</b>，途经点之间不减速到零。
 *
 * <p><b>为什么必须是一条 builder 链</b>：
 * {@link MecanumDrive#actionBuilder} 传入 {@code beginEndVel = 0}，而 Road Runner 的
 * {@code profile()} 用同一个值作为起点和终点速度——因此每一份
 * {@code actionBuilder(...).build()} 出来的轨迹<b>必然以 v = 0 结束</b>。
 * 任何 Action 组合器（{@code SequentialAction} / {@code ParallelAction}）都无法消除
 * 两条独立轨迹之间的物理停车。只有把全部途经点串进同一个 {@code TrajectoryActionBuilder}，
 * 才会共用一条时间最优 profile：途经点处按速度/加速度约束通过，不停车。
 *
 * <p><b>吃球怎么做到「不停车」</b>：不再把取球点当终点，而是当<b>途经点</b>；
 * 在进入取球点前 {@link #eatTriggerDistanceIn} 英寸处用
 * {@code afterDisp(0, eatSeq)} 注册标记，Road Runner 会把该 Action 包进
 * {@code ParallelAction} 与轨迹<b>并行执行</b>——轨迹继续跑，不等待。
 *
 * <p><b>为什么标记值必须持续多帧</b>：{@code Sweeper.update()} 每帧会把目标速度归零
 * （见 {@code Sweeper#update}），因此一次性 {@code InstantAction} 无效；
 * 这里直接复用 {@code EatAction}（每帧 {@code setEat() + update()}）。
 *
 * <p><b>已知代价 / 实机注意</b>：
 * <ul>
 *   <li>吃球时长由 {@link #eatDurationMs} 控制，应约等于「触发点到路线结束」的行驶时间；
 *       若明显长于剩余轨迹时间，轨迹跑完后 {@code ParallelAction} 仍会等它结束——
 *       车已停在发射区但整段 Action 未结束，会推迟下一周期。</li>
 *   <li>「路过式」要求集球器能在行驶中把球吃进去；若必须静止对准取球点，
 *       请改用保守版 {@link AutoCycleAction}。</li>
 *   <li>{@code startHiveThroughHive} 控制第二段是否走 Hive 正下方（与现主程序一致为 true）。</li>
 * </ul>
 *
 * <p><b>接入方式</b>（不在本类中修改主程序，供 {@code AutoBlue} 后续替换状态机）：
 * <pre>
 * actionRunner.add(new ContinuousRouteAction(teamColor, sweeper, goal));
 * </pre>
 * 注意本类同样在<b>首次 run()</b> 时以当时位姿规划，可直接放入队列。
 */
@Config
public class ContinuousRouteAction implements Action {

    /** 在到达取球点前多少英寸开始吃球（触发点沿最后一段接近方向回退） */
    public static double eatTriggerDistanceIn = 12.0;

    /** 吃球持续时长 (ms)，应约等于触发点到轨迹结束的行驶时间 */
    public static long eatDurationMs = 1200;

    /** 第二段是否经 Hive 正下方（true=通过，false=绕开） */
    public static boolean secondLegThroughHive = true;

    /** 是否在路线末尾追加自动瞄准 */
    public static boolean includeAim = true;

    /** 队伍颜色 */
    private final TeamColor teamColor;

    /** 集球器 */
    private final Sweeper sweeper;

    /** 预瞄准的球门目标点 */
    private final Pose2d goal;

    /** 首次 run() 时构建的组合 Action */
    private Action impl;

    /**
     * @param teamColor 队伍颜色
     * @param sweeper   集球器控制器
     * @param goal      预瞄准的单个球门目标点
     */
    public ContinuousRouteAction(TeamColor teamColor, Sweeper sweeper, Pose2d goal) {
        this.teamColor = teamColor;
        this.sweeper = sweeper;
        this.goal = goal;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (impl == null) {
            impl = build();
        }
        packet.put("ContinuousRoute", "Running");
        return impl.run(packet);
    }

    /**
     * 以首次执行时的位姿为起点，把两段路线与吃球标记串成一条 builder 链。
     */
    private Action build() {
        Pose2d start = RobotPosition.getInstance().getPose2d();
        MecanumDrive drive = RobotPosition.getInstance().getDrive();

        // 目标位姿：未访问取球点（含朝向）与另一发射区域（含预瞄航向）
        Pose2d eatTarget = HiveRouteAction.unvisitedEatPointTargetPose(teamColor);
        int eatIndex = HiveRouteAction.unvisitedEatPointIndex(teamColor);
        Pose2d shootTarget = HiveRouteAction.otherShootAreaTargetPose(teamColor, goal);

        // 两段途经点：第一段绕开 Hive，第二段经 Hive 正下方（可配）
        List<Pose2d> leg1 = HiveRoutePlanner.waypoints(teamColor, start, eatTarget, false);
        List<Pose2d> leg2 = HiveRoutePlanner.waypoints(
                teamColor, eatTarget, shootTarget, secondLegThroughHive);

        TrajectoryActionBuilder builder = drive.actionBuilder(start);

        // ---- 第一段：除末项（= 取球点）外全部作为途经点逐个加入，共用一条 profile ----
        for (int i = 0; i < leg1.size() - 1; i++) {
            Pose2d wp = leg1.get(i);
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }

        // ---- 吃球标记：在进入取球点前 eatTriggerDistanceIn 英寸触发，并行不打断轨迹 ----
        Action eatSeq = new SequentialAction(
                new EatAction(sweeper, eatDurationMs),
                new InstantAction(() -> EatPointTracker.markVisited(eatIndex)));

        Pose2d approachFrom = (leg1.size() >= 2) ? leg1.get(leg1.size() - 2) : start;
        Pose2d trigger = triggerPose(approachFrom, eatTarget);
        if (trigger != null) {
            builder = builder
                    .strafeToLinearHeading(trigger.position, trigger.heading.toDouble())
                    .afterDisp(0.0, eatSeq);
        }

        // ---- 取球点本身作为途经点（不是终点）→ 车不停继续前进 ----
        builder = builder.strafeToLinearHeading(eatTarget.position, eatTarget.heading.toDouble());
        if (trigger == null) {
            // 接近段太短、无法提前触发时，退化为「到达取球点即开始吃」
            builder = builder.afterDisp(0.0, eatSeq);
        }

        // ---- 第二段：经 Hive 正下方去另一发射区域，仍属同一条 builder 链 ----
        for (Pose2d wp : leg2) {
            builder = builder.strafeToLinearHeading(wp.position, wp.heading.toDouble());
        }

        // ---- 构建：以上全部途经点合成唯一一条轨迹（beginEndVel = 0，只在最末端停车）----
        Action route = builder.build();
        if (includeAim) {
            route = new SequentialAction(route, new AutoAimAction(teamColor));
        }
        return route;
    }

    /**
     * 计算吃球触发位姿：沿 {@code from → eatTarget} 方向，在取球点前 {@link #eatTriggerDistanceIn}
     * 英寸处，航向与终点航向一致（保持最后一段为朝向取料口的直线）。
     *
     * @param from      最后一段靠近取球点的起点（上一途经点或起点）
     * @param eatTarget 取球点位姿
     * @return 触发位姿；若接近距离不足（无法提前触发）返回 null
     */
    private static Pose2d triggerPose(Pose2d from, Pose2d eatTarget) {
        double dx = eatTarget.position.x - from.position.x;
        double dy = eatTarget.position.y - from.position.y;
        double dist = Math.hypot(dx, dy);
        if (dist < 1e-6) {
            return null;
        }
        // 触发点不能落在上一个途经点之前，至少保留 1 in 的接近段
        double back = Math.min(eatTriggerDistanceIn, dist - 1.0);
        if (back <= 0.0) {
            return null;
        }
        return new Pose2d(
                eatTarget.position.x - dx / dist * back,
                eatTarget.position.y - dy / dist * back,
                eatTarget.heading.toDouble());
    }
}
