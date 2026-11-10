package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 球道计数器：读取对射光电门，按上升沿记录球道内的小球个数。
 *
 * <p><b>职责边界（只做计数）</b>
 * <ul>
 *   <li>读取光电门：光电门挂在本类之下，由本类创建并每帧调用其 update()，
 *       电压采集、滤波与遮挡判定全部由 {@link BreakBeamSensor} 完成；</li>
 *   <li>计数：每出现一个上升沿计数 +1；</li>
 *   <li>清零：{@link #reset()} 把计数归零并复位边缘检测基准。</li>
 * </ul>
 * 本类<b>不做</b>满球判定、不控制电机、不读手柄、也不判断计数方向：
 * 容量判断（BallMax）在 {@code Sweeper} 中，发射完毕与操作手手动清空
 * 由 {@code Turret} / {@code OpMode} 调用 {@code RobotPosition.resetBallCount()} 完成。
 *
 * <p><b>为什么不会重复计数</b>：小球停在光电门中时遮挡持续为 true，
 * 上升沿只在 false → true 的那一帧出现一次，之后不再触发；
 * 小球离开后信号回到 false，下一个小球挡光时才会产生新的上升沿。
 *
 * <p><b>非阻塞</b>：{@link #update()} 每帧 O(1)，无循环与等待。
 */
public class BallCounter {

    private final BreakBeamSensor breakBeam;

    /**
     * 是否已建立边缘检测基准。
     * 构造阶段<b>不读硬件</b>（初始化不属于更新链），基准由第一次 {@link #update()} 建立：
     * 该帧只记录当前遮挡状态、不计数，从而避免上电时已停在光路中的球被误计为新球。
     */
    private boolean hasBaseline;

    /** 上一帧的遮挡状态，用于与当前帧比较以检测上升沿 */
    private boolean lastBlocked;

    /** 球道内累计的小球个数（始终 ≥ 0，只能被 {@link #reset()} 清零） */
    private int count;

    /**
     * 按设备名创建光电门（读取全部在本类内部完成，调用方无需接触传感器）。
     *
     * @param hardwareMap 硬件映射
     * @param deviceName  光电门模拟口在机器人配置中的名称
     */
    public BallCounter(HardwareMap hardwareMap, String deviceName) {
        this(new BreakBeamSensor(hardwareMap, deviceName));
    }

    /**
     * 使用已创建好的光电门，供需要读取原始电压 / 调整阈值的测试程序使用。
     *
     * @param breakBeam 已创建的对射光电门（本类内部负责调用其 update()）
     */
    public BallCounter(BreakBeamSensor breakBeam) {
        this.breakBeam = breakBeam;
        this.count = 0;
        // 初始化阶段只保存下级引用，不采样光电门；
        // 光电门 → BallCounter 的更新只发生在 update() 链内
        this.lastBlocked = false;
        this.hasBaseline = false;
    }

    /**
     * 每帧调用一次：采样光电门，出现上升沿则计数 +1。
     * 无参数：计数只增不减，归零统一走 {@link #reset()}。
     *
     * <p>第一帧只建立边缘检测基准（不计数），之后每帧按 false → true 的上升沿 +1。
     *
     * @return 本帧是否出现上升沿（有新球挡光进入）
     */
    public boolean update() {
        boolean blocked = breakBeam.update();
        if (!hasBaseline) {
            hasBaseline = true;
            lastBlocked = blocked;
            return false;
        }
        boolean risingEdge = blocked && !lastBlocked;
        if (risingEdge) {
            count++;
        }
        lastBlocked = blocked;
        return risingEdge;
    }

    /** @return 当前累计的小球个数（始终 ≥ 0） */
    public int getCount() {
        return count;
    }

    /**
     * 计数清零并复位边缘检测基准（阈值 / alpha 配置不受影响）。
     * <b>不读取硬件</b>：以最近一帧 {@link BreakBeamSensor#isBlocked()} 的结果作为
     * 边缘检测基准，下一帧 update() 若物理状态未变就不会产生虚假上升沿。
     */
    public void reset() {
        count = 0;
        breakBeam.reset();
        lastBlocked = breakBeam.isBlocked();
        hasBaseline = true;
    }
}
