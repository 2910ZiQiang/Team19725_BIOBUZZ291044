package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 球道计数器：读取对射光电门，按上升沿记录球道内的小球个数。
 *
 * <p><b>职责边界（只做计数）</b>
 * <ul>
 *   <li>读取光电门：模拟量的电压采集、滤波与遮挡判定全部由
 *       {@link BreakBeamSensor} 完成，由本类负责创建与每帧调用；</li>
 *   <li>计数：每个上升沿按方向 ±1，计数不会为负；</li>
 *   <li>清零：{@link #reset()} 把计数归零并复位边缘检测基准。</li>
 * </ul>
 * 本类<b>不做</b>满球判定、不控制电机、不读手柄：
 * 容量判断（BallMax）在 {@code Sweeper} 中，发射完毕与操作手手动清空
 * 由 {@code Turret} / {@code OpMode} 调用 {@code RobotPosition.resetBallCount()} 完成。
 *
 * <p><b>两路 0/1 信号不要混淆</b>：{@code externalState} 只表示计数方向
 * （0 = 进入模式 +1，1 = 出球模式 −1），与光电门是否被遮挡无关；
 * 遮挡与否由 BreakBeamSensor 判定（未遮挡 = 高电压、遮挡 = 低电压）。
 *
 * <p><b>为什么不会重复计数</b>：小球停在光电门中时遮挡持续为 true，
 * 上升沿只在 false → true 的那一帧出现一次，之后不再触发；
 * 小球离开后信号回到 false，下一个小球挡光时才会产生新的上升沿。
 *
 * <p><b>非阻塞</b>：{@link #update} 每帧 O(1)，无循环与等待。
 */
public class BallCounter {

    /** 外部信息：进入模式（球挡光计数 +1） */
    public static final int MODE_INTAKE = 0;
    /** 外部信息：出球模式（球挡光计数 -1） */
    public static final int MODE_OUTPUT = 1;

    private final BreakBeamSensor breakBeam;

    /** 上一帧的遮挡状态，用于与当前帧比较以检测上升沿 */
    private boolean lastBlocked;

    /** 球道内累计的小球个数（始终 ≥ 0） */
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
        // 以构造时刻的光电门状态作为边缘检测基准（与 reset() 一致）：
        // 上电时已经停放在光路中的球不会被误计为新球
        this.lastBlocked = breakBeam.update();
    }

    /**
     * 每帧调用一次：采样光电门并更新计数。
     *
     * @param externalState 外部信息：0 = 进入模式（+1），1 = 出球模式（-1）
     * @return 本帧是否有球挡光（上升沿；无论计数 +1 还是 -1）
     */
    public boolean update(int externalState) {
        if (externalState != MODE_INTAKE && externalState != MODE_OUTPUT) {
            throw new IllegalArgumentException(
                    "externalState must be 0 (intake) or 1 (output), got: " + externalState);
        }
        boolean blocked = breakBeam.update();
        boolean risingEdge = blocked && !lastBlocked;
        if (risingEdge) {
            if (externalState == MODE_OUTPUT) {
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
    }
}
