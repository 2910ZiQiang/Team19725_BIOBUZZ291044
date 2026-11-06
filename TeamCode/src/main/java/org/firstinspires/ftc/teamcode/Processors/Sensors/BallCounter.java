package org.firstinspires.ftc.teamcode.Processors.Sensors;

/**
 * 小球计数器：基于对射光电门（BreakBeamSensor）的上升沿检测（边缘检测）
 * 统计经过的小球个数，计数方向由外部每帧传入的 0/1 信息决定。
 *
 * 两路 0/1 信号的区别（不要混淆）：
 *   1. 外部模式信息（update 的 externalState 参数）：
 *      0 = 进入模式（intake），1 = 出球模式（output）。它只决定"上升沿来了
 *      计数 +1 还是 -1"，不表示球有没有挡住光电门。
 *   2. 光电门信号（BreakBeamSensor 的 isBlocked()）：
 *      本项目光电门约定：0 = 遮挡，1 = 未遮挡（对应 BreakBeamSensor 的
 *      blockedOnState = false）。由数字电平经 EMA 滤波 + 阈值判定得到，
 *      用于检测上升沿（"球挡光"这一事件）。
 *
 * 计数规则：
 *   进入模式（外部信息 = 0，intake）：光电门每检测到一个上升沿，count +1
 *   出球模式（外部信息 = 1，output）：光电门每检测到一个上升沿，count -1
 *   count 不会减到负数（球已打空时再挡光，计数停在 0）
 *
 *   注意：发射（shooting）时需要清空球道内球的计数，该操作由上层程序负责，
 *   不在本类实现（上层可调用 reset() 清零）。
 *
 * 为什么不会重复计数：
 *   若小球正好停在光电门中不动，遮挡信号持续为 1（1, 1, 1, ...k,
 *   上升沿只在 0 → 1 的那一帧出现一次，之后不再触发，
 *   因此停留多久都只计 1 次。小球离开后信号回到 0，
 *   下一个小球挡光时才会产生新的上升沿。
 *
 * 去抖说明：
 *   光电门数字量的去抖已由 BreakBeamSensor 内部的 EMA 滤波 + 阈值判定完成
 *   （threshold / alpha / 极性可在 FTC Dashboard 调参），因此本类不再重复
 *   做去抖，只负责"上升沿检测 + 方向计数"。
 *
 * 用法（OpMode 主循环中，每帧调用 update(externalState)）：
 *   注意：本项目光电门 0 = 遮挡，创建 BreakBeamSensor 时需令 blockedOnState = false。
 *   BreakBeamSensor beam = new BreakBeamSensor(hardwareMap, "breakBeam");
 *   beam.setBlockedOnState(false);                 // 0 = 遮挡
 *   BallCounter counter = new BallCounter(beam);
 *   while (opModeIsActive()) {
 *       int externalState = ...;                   // 0 = 进入(intake), 1 = 出球(output)
 *       counter.update(externalState);             // 采样 + 边缘检测计数
 *       telemetry.addData("balls", counter.getCount());  // 实时计数
 *   }
 */
public class BallCounter {

    /** 外部信息：进入模式（intake，球挡光计数 +1） */
    public static final int MODE_INTAKE = 0;
    /** 外部信息：出球模式（output，球挡光计数 -1） */
    public static final int MODE_OUTPUT = 1;

    private final BreakBeamSensor breakBeam;

    /** 上一帧的遮挡状态，用于与当前帧比较以检测跳变 */
    private boolean lastBlocked;

    /** 累计计数的小球个数（始终 ≥ 0） */
    private int count;

    /**
     * @param breakBeam 已创建好的对射光电门（本类内部负责调用其 update()）
     */
    public BallCounter(BreakBeamSensor breakBeam) {
        this.breakBeam = breakBeam;
        this.lastBlocked = false;
        this.count = 0;
    }

    /**
     * 采样一次并做上升沿检测计数。
     * 应在 OpMode 主循环中每帧调用，且每帧传入当前外部模式信息。
     *
     * @param externalState 外部信息：0 = 进入模式（intake），1 = 出球模式（output）
     * @return 本帧是否有球挡光（上升沿；无论计数 +1 还是 -1）
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
            } else {
                count++;
            }
        }

        lastBlocked = blocked;
        return risingEdge;
    }

    /**
     * 实时获取当前累计计数的小球个数（始终 ≥ 0）
     */
    public int getCount() {
        return count;
    }

    /**
     * 清零计数并复位边缘检测状态（不影响光电门的 threshold/alpha 配置）。
     * <b>不读取硬件</b>：以最近一帧 {@link BreakBeamSensor#isBlocked()} 的结果作为
     * 边缘检测基准，下一帧 update() 若物理状态未变就不会产生虚假上升沿。
     */
    public void reset() {
        count = 0;
        breakBeam.reset();
        lastBlocked = breakBeam.isBlocked();
    }
}
