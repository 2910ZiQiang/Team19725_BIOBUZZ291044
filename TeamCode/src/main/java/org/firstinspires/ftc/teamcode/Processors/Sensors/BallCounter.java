package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;

/**
 * 小球计数器：基于对射光电门（BreakBeamSensor）的上升沿检测（边缘检测）
 * 统计球道内的小球个数，并提供容量判定，供上层实现“满 4 球停止吸球”。
 *
 * <p><b>本类负责的四件事中的两件</b>：
 * <ol>
 *   <li>球道自动计数：每个“未遮挡 → 遮挡”的上升沿计 1 个球；</li>
 *   <li>容量判定：{@link #isFull()} 在计数达到 {@link #getCapacity()} 时为 true。</li>
 * </ol>
 * “满球停止吸球”由 {@code RobotPosition.isBallLaneFull()} 透传给 Sweeper；
 * “炮台发射完毕清空”与“操作手手动清空”都调用
 * {@link #reset()} / {@link #onShotComplete()}。这样本类不接触任何执行器，
 * 符合 FTC_Format.md 的分层要求。
 *
 * <p><b>两路 0/1 信号的区别（不要混淆）</b>：
 * <ul>
 *   <li>外部模式信息（update 的 externalState 参数）：0 = 进入模式（intake），
 *       1 = 出球模式（output）。它只决定“上升沿来了计数 +1 还是 -1”，
 *       不表示球有没有挡住光电门。</li>
 *   <li>光电门信号（BreakBeamSensor 的 isBlocked()）：由模拟电压经 EMA 滤波 +
 *       阈值判定得到，true = 遮挡（电压低于阈值），用于检测上升沿（“球挡光”事件）。</li>
 * </ul>
 *
 * <p><b>计数规则</b>：
 * <ul>
 *   <li>进入模式：每个上升沿 count + 1，最大到 capacity（不会超过容量）；</li>
 *   <li>出球模式：每个上升沿 count - 1，最小到 0（不会为负）；</li>
 *   <li>球停在光电门中不动：遮挡持续为 true，只在第一帧产生一次上升沿，
 *       因此停留多久都只计 1 次。</li>
 * </ul>
 *
 * <p><b>去抖</b>：光电门去抖已由 BreakBeamSensor 内部的 EMA + 阈值完成
 * （threshold / alpha 可在 FTC Dashboard 调参），本类只做边缘检测与方向计数。
 *
 * <p><b>用法</b>（OpMode 主循环，每帧一次）：
 * <pre>
 *   BreakBeamSensor beam = new BreakBeamSensor(hardwareMap, "breakBeam");
 *   BallCounter counter = new BallCounter(beam, HypParams.BALL_CAPACITY);
 *   while (opModeIsActive()) {
 *       boolean output = gamepad1.right_bumper;
 *       counter.update(output ? BallCounter.MODE_OUTPUT : BallCounter.MODE_INTAKE);
 *       // RobotPosition 内部持有并透传计数，Sweeper 自动读取 isBallLaneFull()
 *       if (gamepad1.yWasReleased()) RobotPosition.getInstance().resetBallCount();
 *       // 炮台发射完毕时 Turret 会调用 RobotPosition.resetBallCount()
 *       ...
 *   }
 * </pre>
 */
public class BallCounter {

    /** 外部信息：进入模式（intake，球挡光计数 +1） */
    public static final int MODE_INTAKE = 0;
    /** 外部信息：出球模式（output，球挡光计数 -1） */
    public static final int MODE_OUTPUT = 1;

    private final BreakBeamSensor breakBeam;

    /** 球道容量（最多计到该值），由上层按比赛规则/战术配置 */
    private int capacity;

    /** 上一帧的遮挡状态，用于与当前帧比较以检测跳变 */
    private boolean lastBlocked;

    /** 累计计数的小球个数，始终满足 0 ≤ count ≤ capacity */
    private int count;

    /**
     * 使用 {@link HypParams#BALL_CAPACITY} 作为容量创建计数器。
     *
     * @param breakBeam 已创建好的对射光电门（本类内部负责调用其 update()）
     */
    public BallCounter(BreakBeamSensor breakBeam) {
        this(breakBeam, HypParams.BALL_CAPACITY);
    }

    /**
     * @param breakBeam 已创建好的对射光电门（本类内部负责调用其 update()）
     * @param capacity  球道容量（> 0）
     */
    public BallCounter(BreakBeamSensor breakBeam, int capacity) {
        if (breakBeam == null) {
            throw new IllegalArgumentException("breakBeam must not be null");
        }
        if (capacity <= 0) {
            throw new IllegalArgumentException("capacity must be > 0, got: " + capacity);
        }
        this.breakBeam = breakBeam;
        this.capacity = capacity;
        this.lastBlocked = false;
        this.count = 0;
    }

    // ==================== 工厂方法：光电门由本类自己创建 ====================
    //
    // 分层约定（见算法架构图）：
    //     RobotPosition  ->  BallCounter  ->  BreakBeamSensor  ->  AnalogInput
    // 上层（RobotPosition）只允许调用 BallCounter，不得跨级接触 BreakBeamSensor，
    // 也不得自己读 AnalogInput。因此“根据 HardwareMap 找到光电门并构造传感器”
    // 这件事放在本类内部完成。

    /** 光电门在 Robot Configuration 中的默认模拟口设备名 */
    public static final String DEFAULT_BREAK_BEAM_DEVICE = "breakBeam";

    /** 按默认设备名与默认容量创建；光电门未配置时返回 null */
    public static BallCounter createIfPresent(HardwareMap hardwareMap) {
        return createIfPresent(hardwareMap, DEFAULT_BREAK_BEAM_DEVICE, HypParams.BALL_CAPACITY);
    }

    /** 按默认设备名创建指定容量；光电门未配置时返回 null */
    public static BallCounter createIfPresent(HardwareMap hardwareMap, int capacity) {
        return createIfPresent(hardwareMap, DEFAULT_BREAK_BEAM_DEVICE, capacity);
    }

    /**
     * 从硬件表查找光电门并创建计数器。光电门是可选硬件，
     * 未配置时返回 null 而不是抛异常（避免仅做定位/底盘调试的 OpMode 起不来）。
     */
    public static BallCounter createIfPresent(HardwareMap hardwareMap, String deviceName,
                                              int capacity) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("hardwareMap must not be null");
        }
        if (deviceName == null || deviceName.isEmpty()) {
            throw new IllegalArgumentException("deviceName must not be empty");
        }
        AnalogInput input = hardwareMap.tryGet(AnalogInput.class, deviceName);
        return fromAnalogInput(input, capacity);
    }

    /**
     * 由已获取的模拟输入创建计数器。这是本类唯一接触光电门硬件的入口，
     * 公开出来是为了让单元测试可以传入假 AnalogInput，无需真机。
     */
    public static BallCounter fromAnalogInput(AnalogInput input, int capacity) {
        if (input == null) {
            return null;
        }
        double alpha = Math.max(0.01, Math.min(1.0, HypParams.breakBeamAlpha));
        double threshold = Math.max(0.0, Math.min(1.0, HypParams.breakBeamThreshold));
        return new BallCounter(new BreakBeamSensor(input, alpha, threshold), capacity);
    }

    /** @return 本类持有的光电门（仅供诊断/调参显示；上层正常逻辑不应跨级使用） */
    public BreakBeamSensor getBreakBeam() {
        return breakBeam;
    }

    /**
     * 按进球队方向更新一帧。RobotPosition 每帧调用此重载；
     * 需要按出球方向倒计数时使用 {@link #update(int)}。
     *
     * @return 本帧是否检测到上升沿
     */
    public boolean update() {
        return update(MODE_INTAKE);
    }

    /**
     * 采样一次并做上升沿检测计数。应在 OpMode 主循环中每帧调用一次，
     * 且必须在读取 {@link #isFull()} 之前调用。
     *
     * @param externalState 外部信息：0 = 进入模式（intake），1 = 出球模式（output）
     * @return 本帧是否检测到上升沿（球挡光）；无论计数是否因饱和/归零而变化都返回 true
     */
    public boolean update(int externalState) {
        if (externalState != MODE_INTAKE && externalState != MODE_OUTPUT) {
            throw new IllegalArgumentException(
                    "externalState must be 0 (intake) or 1 (output), got: " + externalState);
        }

        boolean output = (externalState == MODE_OUTPUT);
        boolean blocked = breakBeam.update();

        // 上升沿：0 → 1 跳变。持续为 1（球停住不动）不会重复触发
        boolean risingEdge = blocked && !lastBlocked;
        if (risingEdge) {
            if (output) {
                if (count > 0) {   // 不允许减到负数
                    count--;
                }
            } else if (count < capacity) {
                count++;           // 达到容量后不再累加，防止机械挤压造成虚高
            }
        }

        lastBlocked = blocked;
        return risingEdge;
    }

    /** @return 当前计数（0 ≤ count ≤ capacity） */
    public int getCount() {
        return count;
    }

    /** @return 球道容量 */
    public int getCapacity() {
        return capacity;
    }

    /**
     * 修改容量。若当前计数大于新容量，计数被钳制到新容量。
     *
     * @param capacity 新容量（> 0）
     */
    public void setCapacity(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("capacity must be > 0, got: " + capacity);
        }
        this.capacity = capacity;
        if (count > capacity) {
            count = capacity;
        }
    }

    /** @return 是否已满（计数达到容量）；上层据此锁定吸球 */
    public boolean isFull() {
        return count >= capacity;
    }

    /** @return 还能再装几个球（0 表示已满） */
    public int getRemaining() {
        return Math.max(0, capacity - count);
    }

    /**
     * 清零计数并复位边缘检测状态（不影响光电门的 threshold/alpha 配置）。
     * 用于操作手手动清空，也用于炮台发射完毕后的清空。
     *
     * <p><b>不读取硬件</b>：以最近一帧 {@link BreakBeamSensor#isBlocked()} 的结果作为
     * 边缘检测基准，下一帧 update() 若物理状态未变就不会产生虚假上升沿。
     * 因此如果清空时光电门里仍停着一个球，该球不会被再次计数（避免重复计数）；
     * 手动清空应尽量在球道空时执行。
     */
    public void reset() {
        count = 0;
        breakBeam.reset();
        lastBlocked = breakBeam.isBlocked();
    }

    /**
     * 炮台一个发射周期结束（扳机释放并关闭）时调用：清空球道计数。
     * 语义与 {@link #reset()} 完全相同，提供此别名是为了让“炮台 → 球道”的接口
     * 在调用处自解释，而不是让炮台代码直接操作计数器内部状态。
     */
    public void onShotComplete() {
        reset();
    }
}
