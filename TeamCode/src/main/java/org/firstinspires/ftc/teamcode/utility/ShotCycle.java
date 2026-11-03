package org.firstinspires.ftc.teamcode.utility;

/**
 * 发射周期（volley）状态机：把“扳机是否释放”的布尔命令转换为
 * “一次发射是否刚刚完毕”的单帧事件。
 *
 * <p>这是纯计算工具类，不接触任何硬件，也不依赖 Controllers，
 * 因此可以在单元测试里用显式时间戳驱动（见 {@code ShotCycleTest}）。
 *
 * <p><b>定义</b>：
 * <ul>
 *   <li>扳机 false → true：发射开始，记录起始时刻；</li>
 *   <li>扳机 true 持续期间：正在发射，不产生事件；</li>
 *   <li>扳机 true → false：一次发射结束。只有持续时间 ≥ {@link #minDurationSec}
 *       才产生“发射完毕”事件，用于滤掉操作手/程序抖动造成的短脉冲。</li>
 * </ul>
 *
 * <p><b>为什么用“释放后”而不是“释放中”</b>：炮台在扳机释放期间只是持续供球，
 * 此时球道里可能还有球；只有扳机重新关闭，才认为这一轮发射流程结束，
 * 上层可以安全地清空球道计数。
 */
public class ShotCycle {

    /** 默认最短发射持续时间（秒） */
    public static final double DEFAULT_MIN_DURATION_SEC = 0.15;

    /** 最短发射持续时间（秒） */
    private double minDurationSec;

    /** 上一帧的扳机命令 */
    private boolean firing;

    /** 本次发射开始的时刻（秒） */
    private double firingStartSec;

    /** 本次发射持续了多久（秒），仅用于遥测/调试 */
    private double lastDurationSec;

    /** 是否有一个尚未被读取的“发射完毕”事件 */
    private boolean completed;

    public ShotCycle() {
        this(DEFAULT_MIN_DURATION_SEC);
    }

    /**
     * @param minDurationSec 最短发射持续时间（秒，≥ 0）
     */
    public ShotCycle(double minDurationSec) {
        setMinDurationSec(minDurationSec);
    }

    /**
     * 推进一帧（使用系统时钟）。
     *
     * @param trigger 本帧扳机是否释放（true = 释放/发射）
     */
    public void update(boolean trigger) {
        update(trigger, nowSec());
    }

    /**
     * 推进一帧（显式时间戳，供单元测试/离线回放使用）。
     *
     * @param trigger 本帧扳机是否释放（true = 释放/发射）
     * @param nowSec  当前时刻（秒），必须单调不减
     */
    public void update(boolean trigger, double nowSec) {
        if (trigger && !firing) {
            // 上升沿：开始一次发射
            firing = true;
            firingStartSec = nowSec;
        } else if (!trigger && firing) {
            // 下降沿：发射结束，只有足够长的脉冲才算一次有效发射
            firing = false;
            lastDurationSec = Math.max(0.0, nowSec - firingStartSec);
            if (lastDurationSec >= minDurationSec) {
                completed = true;
            }
        }
    }

    /**
     * 读取并清除“发射完毕”事件。
     *
     * @return 自上次调用以来是否完成过一次有效发射；只返回一次 true
     */
    public boolean consumeCompleted() {
        boolean result = completed;
        completed = false;
        return result;
    }

    /** @return 是否有尚未读取的“发射完毕”事件（只读，不清除） */
    public boolean isCompleted() {
        return completed;
    }

    /** @return 当前是否处于发射（扳机释放）状态 */
    public boolean isFiring() {
        return firing;
    }

    /** @return 上一次发射持续时间（秒），无记录时为 0 */
    public double getLastDurationSec() {
        return lastDurationSec;
    }

    /** @return 当前最短发射持续时间（秒） */
    public double getMinDurationSec() {
        return minDurationSec;
    }

    /**
     * @param minDurationSec 最短发射持续时间（秒，≥ 0）
     */
    public void setMinDurationSec(double minDurationSec) {
        if (minDurationSec < 0) {
            throw new IllegalArgumentException("minDurationSec must be >= 0, got: " + minDurationSec);
        }
        this.minDurationSec = minDurationSec;
    }

    /** 复位全部状态（不产生事件）。 */
    public void reset() {
        firing = false;
        firingStartSec = 0;
        lastDurationSec = 0;
        completed = false;
    }

    private static double nowSec() {
        return System.nanoTime() / 1e9;
    }
}
