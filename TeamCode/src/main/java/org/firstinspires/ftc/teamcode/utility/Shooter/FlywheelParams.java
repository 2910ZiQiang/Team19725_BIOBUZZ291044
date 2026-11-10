package org.firstinspires.ftc.teamcode.utility.Shooter;

import com.acmerobotics.dashboard.config.Config;

/**
 * 飞轮速度环的<b>参数集</b>（纯数据类，无硬件依赖）。
 *
 * <p><b>所有飞轮控制参数只在本文件里出现，且大球 / 小球分开写</b>：
 * <ul>
 *   <li>{@link #BIG} —— 大球飞轮，取值见文件末尾「大球飞轮参数」块</li>
 *   <li>{@link #SMALL} —— 小球飞轮，取值见文件末尾「小球飞轮参数」块</li>
 * </ul>
 * 中间是字段声明（只有含义，没有数值），<b>要改参数就只改末尾那两个块</b>。
 *
 * <p>两个实例由 {@code Controllers.Turret.Shooter} 在构造两个发射器时分别传入
 * （{@code Shooter.BIG_PARAMS} / {@code Shooter.SMALL_PARAMS}），并由
 * {@link #applyTo(FlywheelCalculator)} 每帧灌进计算器，
 * 因此在 FTC Dashboard 上改值也是下一帧即生效（两个飞轮互不影响）。
 *
 * <p><b>为什么参数不放 {@link FlywheelCalculator} 里</b>：计算器是「一套算法」，参数是「一台机器」。
 * 分开后计算器可以换一套参数就复算另一台飞轮的历史日志，两个飞轮也不必各写一份控制代码。
 *
 * <p><b>当前控制律</b>（第 4 代，2026-10-22 起，无 boost）：
 * <pre>
 *   e_raw  = ω* - ω
 *   s      = clamp(V_nom / V_busFilt, sMin, sMax)        // 母线电压归一化，只作用于前馈
 *   ff     = s·(kS·sgn(ω*) + kff·ω*)
 *   ωf     = EMA(ω, τ(|e_raw|))                          // 误差越大 τ 越小
 *   duty   = sat( ff + kp·(ω*-ωf) + ki·∫(ω*-ωf) , 0, 1 )
 * </pre>
 */
@Config
public class FlywheelParams {

    // ==================================================================================
    //  字段声明（只有含义，数值一律写在文件末尾的两套参数块里）
    // ==================================================================================

    // ---------------- 前馈：稳态模型 d_ss(ω) = kS + kff·ω ----------------

    /** 静摩擦前馈 (占空比)：稳态模型的截距，按 {@code kS·sgn(ω*)} 施加（ω*=0 时为 0） */
    public double kS;

    /** 稳态前馈增益 (占空比)/(tick/s)：稳态模型的斜率 */
    public double kff;

    // ---------------- 反馈 PI ----------------

    /**
     * 比例增益 (占空比)/(tick/s)。
     *
     * <p>量化台阶自检：{@code kp × quantStep} 是「一格编码器测量噪声被放大成多少占空比」，
     * 即量化极限环的幅值下限。工程判据：≤0.05 合格，≤0.02 好，&gt;0.1 会跑出教科书极限环。
     * 遥测里有一行 {@code kp*step} 自动告警。
     */
    public double kp;

    /** 积分增益 (占空比)/(tick/s·s) */
    public double ki;

    /** I 项上限 (占空比)：对 {@code ki·∫e} 限幅，同时回算积分量（抗饱和） */
    public double maxI;

    // ---------------- 输出限幅 ----------------

    /** 输出下限 (占空比) */
    public double dutyMin;

    /** 输出上限 (占空比) */
    public double dutyMax;

    // ---------------- 量化与稳态判据 ----------------

    /** 速度量化台阶 (tick/s)：编码器测速最小分辨率（用于误差带下限、kp 告警、滤波拐点） */
    public double quantStep;

    /**
     * 稳态误差带比例：{@code eBand = max(settleFrac·|ω*|, 2·quantStep)}。
     *
     * <p>eBand 同时是到位判据和积分门限（{@code |e| < eBand} 才积分）。SimpleShooter 分支
     * 把积分门限换成固定 iZone=120 tick/s，被否决：低目标下等于把准入放宽 3 倍
     * （0→600 过冲 +4.5→+11.0 tick/s）。门限必须随目标缩放。
     */
    public double settleFrac;

    /** 连续落在误差带内多少帧算「到位」（10 帧 ≈ 0.1 s） */
    public int settleFrames;

    // ---------------- 速度滤波（误差调度 EMA） ----------------

    /** 稳态时间常数 (s)：越小噪声越小、相位滞后越大 */
    public double tauVMax;

    /** 大误差时间常数 (s)：约 1 个控制周期，几乎不滤波，保证加速/退饱和不滞后 */
    public double tauVMin;

    /** 调度下拐点（倍 eBand）：{@code |e| ≤ eLoFrac·eBand} 用 tauVMax */
    public double eLoFrac;

    /** 调度上拐点（倍 eBand）：{@code |e| ≥ eHiFrac·eBand} 用 tauVMin */
    public double eHiFrac;

    // ---------------- 积分冻结 ----------------
    //
    // 第 4 代原有 dropEnterFrac（进掉速态的门限，倍 eBand）+ inDrop 滞回，已整体删除
    // （2026-11-06，见 FlywheelCalculator 类注释「取舍记录」第 4 条）：实测 inDrop 占比 6.3%，
    // 模型里删掉后跌深/回带时间/过冲几乎不变。删掉后带外只剩「冻结积分」这一个动作。

    /** 带外冻结积分的最长时长 (s)，超时强制清零（防误差长期带外导致积分永久失效） */
    public double iFreezeTimeoutSec;

    // ---------------- 母线电压归一化 ----------------

    /** 是否启用母线电压归一化（只乘前馈）。默认关：先按 T1~T4 协议验证再打开 */
    public boolean useVoltageNorm;

    /** 额定母线电压 (V)：前馈按该电压标定 */
    public double vBusNominal;

    /** 归一化系数下限（欠压时前馈最多放大到 1/sMin 倍） */
    public double vScaleMin;

    /** 归一化系数上限（过压时前馈最多缩小到 1/sMax 倍） */
    public double vScaleMax;

    /**
     * 母线电压低通时间常数 (s)。必须远大于单发发射造成的电压跌落时长（≈0.4 s），
     * 否则会把「单发掉压」当成「电池没电」，在恢复期末把前馈放大到 &gt;1，人造一次饱和。
     *
     * <p>原「掉速期间冻结 + 0.6 s hold-off + 2.0 s 冻结超时」三个参数已随状态机删除；
     * 冻结窗口本身保留，但改成无状态的「{@code |e| ≥ eBand} 时不吸收读数」（见
     * {@code FlywheelCalculator#updateBusVoltage}）。所以现在两级防护是：
     * 这一项负责把 0.4 s 跌落压到 33%，误差门负责在大误差窗口内完全不吸收。
     * 不要调小，更不要退化成原始读数 {@code s=12/V_raw}（那会让 s 立刻跳 1.071）。
     */
    public double tauBus;

    // ---------------- 装配与范围 ----------------

    /** 电机方向：由飞轮接线极性决定（构造时下发一次），与整定参数无关 */
    public boolean motorReversed;

    /** 该飞轮最大转速 (tick/s)，目标转速会被钳制到 [0, speedMax] */
    public int speedMax;

    // ==================================================================================
    //  大球飞轮参数（BIG）—— 2026-09-04 实机整定，8 工作点回归 R²=0.9949
    //  2026-10 按实测日志复核 kp 与速度滤波（kS/kff/ki 未动），见 §2.15
    // ==================================================================================

    /**
     * 大球飞轮参数。改这里即改大球；Dashboard 上显示为「FlywheelParams.BIG」分组。
     */
    public static final FlywheelParams BIG = new FlywheelParams();

    static {
        // ---- 前馈（Tune6.5 Feedforward：多点稳态回归 d_ss = kS + kff·ω）----
        BIG.kS = 0.02945;
        BIG.kff = 3.8307e-4;

        // ---- PI（Tune6.1 StepResponse；kp 另需满足 kp×20 ≤ 0.05）----
        // 2026-10 按 09-02 实测日志重定：单步增益 kp·Kd（Kd≈2740 tick/s 每单位 duty）
        // 从 6.0 降到 2.7 → 稳态极限环由 ±2 个量化台阶收到 ±1 个，见 doc/现在的控制系统.md §2.15。
        // 2026-10-27 实机复测（kp=1.9095e-3，7 发）：回带中位 0.31 s、duty_sd 0.013~0.017，
        // 比 kp=1.0e-3 的回带 0.91 s 明显改善，且仍在判据①②内 → 采用 1.9095e-3。
        BIG.kp = 1.9095e-3;
        BIG.ki = 2.59224e-3;
        BIG.maxI = 0.3;

        // ---- 输出限幅 ----
        BIG.dutyMin = 0.0;
        BIG.dutyMax = 1.0;

        // ---- 量化与稳态判据 ----
        BIG.quantStep = 20.0;      // 实测编码器测速台阶
        BIG.settleFrac = 0.02;     // ±2%
        BIG.settleFrames = 10;     // 10 帧 ≈ 0.1 s

        // ---- 速度滤波（Tune6.6 VelocityFilter 反解 tauVMax）----
        // 2026-10 重定：实测 |e| 有 14% 的时间越过 0.90·eBand（36 tick/s），而此时
        // τ 被调度到 tauVMin=8 ms——比 tauVMax 更"不滤波"，正是稳态梳齿的来源。
        // 把两个拐点抬到 1 / 3 个量化台阶之上（20 / 60 tick/s），τ 才停在慢档。
        BIG.tauVMax = 0.055;
        BIG.tauVMin = 0.012;
        BIG.eLoFrac = 0.50;        // 0.50·eBand = 20 tick/s = 1 个台阶
        BIG.eHiFrac = 1.50;        // 1.50·eBand = 60 tick/s = 3 个台阶

        // ---- 积分冻结（掉速态已删除：见 FlywheelCalculator 类注释「取舍记录」第 4 条）----
        // SimpleShooter 删掉的是整个冻结逻辑、只留固定 iZone=120 tick/s 门限——被否决：
        // 门限不随目标缩放，低目标下过冲翻倍。这里保留 1.5 s 超时兜底。
        BIG.iFreezeTimeoutSec = 1.5;

        // ---- 母线电压归一化（Tune6.7 BusVoltageFilter 定 tauBus）----
        // 2026-10-27 起默认开启：09-02 两份实测（大轮 20:12 vscale≡1、小轮 20:16 vscale≈0.96）
        // 对比显示开启后前馈随母线跌落自动补偿，判据⑥待 VoltageSweep 收尾。
        // 掉速状态机已删，冻结窗口改由「|e| ≥ eBand 时不吸收读数」承担；这里靠
        // tauBus=1.0 s 低通吸收单发跌压（见字段注释）。
        BIG.useVoltageNorm = true;
        BIG.vBusNominal = 12.0;
        BIG.vScaleMin = 0.85;
        BIG.vScaleMax = 1.25;
        BIG.tauBus = 1.0;

        // ---- 装配与范围 ----
        BIG.motorReversed = true;
        BIG.speedMax = 3000;
    }

    // ==================================================================================
    //  小球飞轮参数（SMALL）—— 2026-10-27 实机整定完成
    // ==================================================================================

    /**
     * 小球飞轮参数。改这里即改小球；Dashboard 上显示为「FlywheelParams.SMALL」分组。
     *
     * <p>2026-10-27 按 09-02 20:16 的实测日志定稿（0→1900→1700，8 发）：
     * 稳态 1700 误差 +0.10 tick/s、duty 0.6534±0.0206、积分中位 −1.6（判据③通过），
     * 回带中位 0.19 s、最大 0.32 s（判据④ 0.43 s），`vscale≈0.96`（归一化已开）。
     * 其静态模型 `duty/s = 0.0295 + 3.83e-4·ω` 与大球几乎一致，故 kS/kff 沿用大球量级。
     */
    public static final FlywheelParams SMALL = new FlywheelParams();

    static {
        // ---- 前馈（Tune6.5：多点稳态回归 d_ss = kS + kff·ω，实测 kff=3.8346e-4）----
        SMALL.kS = 0.02945;
        SMALL.kff = 3.8346e-4;

        // ---- PI（Tune6.1；kp×20 = 0.0348 ≤ 0.05）----
        SMALL.kp = 1.739e-3;
        SMALL.ki = 2.50771e-3;
        SMALL.maxI = 0.3;

        // ---- 输出限幅（一般与大球相同）----
        SMALL.dutyMin = 0.0;
        SMALL.dutyMax = 1.0;

        // ---- 量化与稳态判据：台阶由编码器/齿比决定，换了机构要重测 ----
        SMALL.quantStep = 20.0;
        SMALL.settleFrac = 0.02;
        SMALL.settleFrames = 10;

        // ---- 速度滤波：沿用大球重定值（小球实测稳态纹波与大球同量级，暂不单独整定）----
        SMALL.tauVMax = 0.055;
        SMALL.tauVMin = 0.012;
        SMALL.eLoFrac = 0.50;
        SMALL.eHiFrac = 1.50;

        // ---- 积分冻结（掉速态已删除，同大球）----
        SMALL.iFreezeTimeoutSec = 1.5;

        // ---- 母线电压归一化（2026-10-27 实测已开：vscale≈0.96，前馈补齐母线跌落）----
        SMALL.useVoltageNorm = true;
        SMALL.vBusNominal = 12.0;
        SMALL.vScaleMin = 0.85;
        SMALL.vScaleMax = 1.25;
        SMALL.tauBus = 1.0;

        // ---- 装配与范围：方向与最大转速按小球机构填 ----
        SMALL.motorReversed = true;
        SMALL.speedMax = 3000;
    }

    // ==================================================================================
    //  装配 / 复制
    // ==================================================================================

    /** 供 Dashboard 反射与离线脚本使用的空构造（字段无工程默认值，请用 {@link #copyOf} 取默认） */
    public FlywheelParams() {
    }

    /**
     * @param src 源参数集（为 null 时返回空实例）
     * @return 一份独立副本（字段与 src 相同，但不共享引用）
     */
    public static FlywheelParams copyOf(FlywheelParams src) {
        FlywheelParams copy = new FlywheelParams();
        copy.copyFrom(src);
        return copy;
    }

    /**
     * 把本参数集灌进计算器（每帧调用，Dashboard 改动即时生效）。
     *
     * @param calc 目标计算器；为 null 时直接返回
     */
    public void applyTo(FlywheelCalculator calc) {
        if (calc == null) {
            return;
        }
        calc.kS = kS;
        calc.kff = kff;
        calc.kp = kp;
        calc.ki = ki;
        calc.maxI = maxI;
        calc.dutyMin = dutyMin;
        calc.dutyMax = dutyMax;
        calc.quantStep = quantStep;
        calc.settleFrac = settleFrac;
        calc.settleFrames = settleFrames;
        calc.tauVMax = tauVMax;
        calc.tauVMin = tauVMin;
        calc.eLoFrac = eLoFrac;
        calc.eHiFrac = eHiFrac;
        calc.iFreezeTimeoutSec = iFreezeTimeoutSec;
        calc.useVoltageNorm = useVoltageNorm;
        calc.vBusNominal = vBusNominal;
        calc.vScaleMin = vScaleMin;
        calc.vScaleMax = vScaleMax;
        calc.tauBus = tauBus;
    }

    /**
     * 用另一套参数覆盖本参数集（整定辅助：把标定结果在两个飞轮之间互相复制）。
     *
     * @param other 源参数集；为 null 或 this 时直接返回
     */
    public void copyFrom(FlywheelParams other) {
        if (other == null || other == this) {
            return;
        }
        kS = other.kS;
        kff = other.kff;
        kp = other.kp;
        ki = other.ki;
        maxI = other.maxI;
        dutyMin = other.dutyMin;
        dutyMax = other.dutyMax;
        quantStep = other.quantStep;
        settleFrac = other.settleFrac;
        settleFrames = other.settleFrames;
        tauVMax = other.tauVMax;
        tauVMin = other.tauVMin;
        eLoFrac = other.eLoFrac;
        eHiFrac = other.eHiFrac;
        iFreezeTimeoutSec = other.iFreezeTimeoutSec;
        useVoltageNorm = other.useVoltageNorm;
        vBusNominal = other.vBusNominal;
        vScaleMin = other.vScaleMin;
        vScaleMax = other.vScaleMax;
        tauBus = other.tauBus;
        motorReversed = other.motorReversed;
        speedMax = other.speedMax;
    }
}
