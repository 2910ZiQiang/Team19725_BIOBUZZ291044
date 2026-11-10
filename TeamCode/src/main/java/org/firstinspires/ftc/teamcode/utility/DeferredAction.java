package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

/**
 * 「延迟构造」的 Action 包装：首次 {@link #run} 时才创建真正的子 Action。
 *
 * <p><b>为什么需要</b>：本项目所有轨迹类 Action（{@code HiveRouteAction} 系列、
 * {@code AutoAimAction} 等）都在<b>构造函数</b>里读取 {@code RobotPosition} 的当前位姿并
 * 一次性 {@code build()} 出轨迹，即「轨迹在构造时刻固化」。
 * 若把它们提前放进 {@code SequentialAction}，后续 Action 会用<b>上一段动作开始前</b>的旧位姿
 * 规划，导致路线错位。用本包装可保证每个子 Action 在被执行的第一帧才构造，从而拿到当时的真实位姿。
 *
 * <p><b>典型用法</b>：
 * <pre>
 * Action cycle = new SequentialAction(
 *         new DeferredAction(() -> new GoToUnvisitedEatPointAvoidHiveAction(teamColor)),
 *         new EatAction(sweeper),
 *         new DeferredAction(() -> new GoToOtherShootAreaViaHiveAction(teamColor, goal)));
 * </pre>
 *
 * <p>无状态副作用：本包装只负责延时，不改变子 Action 的语义。
 */
public class DeferredAction implements Action {

    /** 子 Action 工厂，在首次 run() 时调用一次 */
    private final Supplier<Action> factory;

    /** 已创建的子 Action，null 表示尚未创建 */
    private Action impl;

    public DeferredAction(@NonNull Supplier<Action> factory) {
        this.factory = factory;
    }

    /** @return 是否已创建子 Action（供遥测判断当前处于「待构造」还是「执行中」） */
    public boolean isCreated() {
        return impl != null;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (impl == null) {
            impl = factory.get();
        }
        return impl.run(packet);
    }

    @Override
    public void preview(Canvas fieldOverlay) {
        // 尚未构造时没有可预览的轨迹；构造后再交给子 Action
        if (impl != null) {
            impl.preview(fieldOverlay);
        }
    }
}
