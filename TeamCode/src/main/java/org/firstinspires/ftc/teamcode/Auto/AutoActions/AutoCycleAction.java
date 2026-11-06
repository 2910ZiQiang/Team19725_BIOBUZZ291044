package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.utility.DeferredAction;

import java.util.ArrayList;
import java.util.List;

/**
 * 【保守版】把自动阶段的一个完整周期合成为 <b>一个 Action</b>。
 *
 * <p>周期内容（与现 {@code AutoBlue} 状态机等价的顺序，仍然在取球点停车吃球）：
 * <pre>
 *   前往未访问取球点（绕开 Hive） → 吃球 → 经 Hive 正下方前往另一发射区域 → （可选）自动瞄准
 * </pre>
 *
 * <p><b>与「每段单独入队」的差别</b>：
 * <ul>
 *   <li>{@link SequentialAction} 的实现是 {@code tailrec}——上一子 Action 返回 false 的
 *       <b>同一帧</b>立刻执行下一个，消除 {@code ActionRunner} 每帧最多启动一个 Action
 *       带来的 1 帧（约 20 ms）空档；该空档内没有任何代码给底盘下发电机指令；</li>
 *   <li>{@code ActionRunner.isBusy()} 在整个周期内保持 true，
 *       不会在周期中途短暂变 false 而让 {@code Chassis} 误判「队列空闲」、放行手柄或主动刹停。</li>
 * </ul>
 *
 * <p><b>关键实现</b>：各移动段用 {@link DeferredAction} 包装，保证每段在被执行的第一帧
 * 才以<b>当时的真实位姿</b>规划轨迹（详见 {@link DeferredAction} 的说明）。
 * 若直接提前构造，第二段会用「吃球之前」的旧位姿计算终点。
 *
 * <p><b>注意</b>：本 Action 只能消除调度间隙，<b>不能</b>消除轨迹之间的减速到零——
 * 吃球段与移动段各自都是一条独立轨迹（{@code beginEndVel = 0}）。
 * 需要全程不停车请改用 {@link ContinuousRouteAction}。
 *
 * <p><b>接入方式</b>（不在本类中修改主程序，供 {@code AutoBlue} 后续替换状态机）：
 * <pre>
 * actionRunner.add(new AutoCycleAction(teamColor, sweeper, goal));
 * // 之后在循环里只需：if (!actionRunner.isBusy()) 追加下一周期
 * </pre>
 */
@Config
public class AutoCycleAction implements Action {

    /** 是否在周期末尾追加一次自动瞄准（原地转向对齐球门） */
    public static boolean includeAim = true;

    /** 队伍颜色 */
    private final TeamColor teamColor;

    /** 集球器 */
    private final Sweeper sweeper;

    /** 预瞄准的球门目标点（用于发射区域终点航向，与 {@code Chassis} 自动瞄准一致） */
    private final Pose2d goal;

    /** 首次 run() 时构建的组合 Action */
    private Action impl;

    /**
     * @param teamColor 队伍颜色
     * @param sweeper   集球器控制器
     * @param goal      预瞄准的单个球门目标点
     */
    public AutoCycleAction(TeamColor teamColor, Sweeper sweeper, Pose2d goal) {
        this.teamColor = teamColor;
        this.sweeper = sweeper;
        this.goal = goal;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (impl == null) {
            impl = build();
        }
        packet.put("AutoCycle", "Running");
        return impl.run(packet);
    }

    /**
     * 组合周期。每个移动段都延迟到执行时再构造，避免用旧位姿规划。
     */
    private Action build() {
        List<Action> steps = new ArrayList<>();

        steps.add(new DeferredAction(() -> new GoToUnvisitedEatPointAvoidHiveAction(teamColor)));
        steps.add(new EatAction(sweeper));
        steps.add(new DeferredAction(() -> new GoToOtherShootAreaViaHiveAction(teamColor, goal)));

        if (includeAim) {
            steps.add(new AutoAimAction(teamColor));
        }

        return new SequentialAction(steps);
    }
}
