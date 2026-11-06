package org.firstinspires.ftc.teamcode.utility.Shooter;

/**
 * 飞轮（发射机构）速度环计算器 —— <b>纯计算类</b>，不持有任何硬件引用。
 *
 * <p>本类只做一件事：{@code (目标转速 ω*, 当前转速 ω, 母线电压 V_bus, dt) → 占空比 duty}。
 * 电机、遥测、硬件初始化全部留在调用方（{@code Controllers.Shooter.PrototypeShooter}、
 * {@code Controllers.Turret.Shooter}），因此本类可离线复算（给一串采样即可回放整条控制轨迹），
 * 也便于单独做数值验证。
 *
 * <p><b>控制律（第 4 代，2026-10-22）</b>：
 * <pre>
 *   e_raw  = ω* - ω
 *   e_f    = ω* - ωf,   ωf = EMA(ω, τ(|e_raw|))          // 误差调度速度滤波
 *   s      = clamp(V_nom / V_busFilt, sMin, sMax)          // 母线电压归一化（只乘前馈）
 *   ff     = s·(kS·sgn(ω*) + kff·ω*)
 *   duty   = sat( ff + kp·e_f + ki·∫e_f , dutyMin, dutyMax )
 * </pre>
 * 与第 3 代（含 {@code boost} 掉速补偿）的差别：
 * <ul>
 *   <li><b>删除 boost</b>：实测发射时刻输出早已饱和（kp·e 一项就够把输出顶到 1.0），
 *       boost 的 0.4 在饱和段贡献恒为 0；它唯一真实的作用是把退饱和点从
 *       {@code e_desat=(1-ff)/kp}（102~165 tick/s）推迟到 {@code e≤0}，即抹掉整段线性收敛，
 *       结构性造成 +50~90 tick/s 过冲。掉速期间「多给一点」在物理上根本不可能——
 *       占空比已到 1，电机端电压就是母线电压。</li>
 *   <li><b>积分冻结</b>：掉速（误差出带）期间积分既不累加也不清零，而是<b>冻结</b>；
 *       误差回到带内才继续累加。这样恢复过程不攒积分、不产生过冲，靠原有稳态积分削掉残差。</li>
 *   <li><b>速度滤波</b>：稳态用 τ=t{@link #tauVMax} 压掉 20 tick/s 的量化噪声，
 *       大误差时 τ 收敛到 {@link #tauVMin}（≈1 帧），避免加速/退饱和滞后。</li>
 *   <li><b>母线电压归一化</b>：只对前馈做 {@code s} 缩放（不乘反馈），
 *       让「同一目标转速在不同电量下要的占空比」保持一致，积分不必长期负重。
 *       电压必须经过时间常数 ≈1 s 的低通：否则单发发射造成的 0.4 s 跌落会被当成电池没电，
 *       在恢复期末把前馈放大到 &gt;1，人造出一次饱和。</li>
 * </ul>
 *
 * <p><b>2026-11-06 取舍记录</b>（针对分支 {@code 简化的飞轮控制/SimpleShooter}，提交 {@code c8c0735}）：
 * 该提交把本类改写成「原始误差 P/I + 原始母线电压归一化 + 绝对积分门限 iZone=120 tick/s」，
 * 净删 133 行（占本文件 25%）。评审结论：<b>只有「删掉 inDrop 掉速状态机」被采纳</b>；
 * 速度 EMA、母线电压低通、相对积分门限三项全部退回。判据来自
 * {@code tools/tuning/analyze_generations.py} 的机器模型（A=3830 1/s、τ_plant=0.68 s、
 * 量化 20 tick/s、控制周期 20 ms）与实测转速序列回放：
 * <ol>
 *   <li><b>速度 EMA 不能删</b>：编码器只有 20 tick/s 分辨率，kp×quantStep=0.038 会原封不动
 *       印到 duty 上。同一段实测转速回放，稳态 duty 标准差 0.0053→0.0210（1750 目标）、
 *       0.0076→0.0197（1600）、0.0059→0.0207（2050），峰峰值 0.027→0.115（G4→SimpleShooter）。
 *       抖动会经电机/皮带变成可听的嗡嗡声与额外铜损，不是「看不见的细节」。</li>
 *   <li><b>电压低通不能删</b>：单发发射把母线从 12.4 V 拉到 11.2 V 约 0.4 s。没有低通时
 *       {@code s=12/V_raw} 立刻跳 1.000→1.071，前馈整段放大约 7%，再叠加电池 RC 回弹
 *       （+0.3 V、τ≈0.15 s）会在恢复期末人造过冲：模型里 +15.8→+24.1 tick/s。
 *       反之保留 τ=1.0 s 的 EMA，同样 0.4 s 跌落只走完 33%（12.40→12.00 V，s 变化 &lt;0.3%），
 *       所以真正起作用的是低通本身；再叠一层「掉速期间不吸收读数」的暂停（本类用
 *       {@code |e| ≥ eBand} 表达，见 {@link #updateBusVoltage}），恢复期过冲还能再砍一半。
 *       <b>删低通才是问题</b>。</li>
 *   <li><b>积分门限必须是相对量</b>：{@code eBand=max(0.02·|ω*|, 2·quantStep)}，1700 目标下 40 tick/s，
 *       600 目标下仍是 40（量化下限）。换成固定 120 等于低目标时把积分准入放宽 3 倍：
 *       模型里 0→600 过冲 +4.5→+11.0、0→1000 +1.8→+10.5 tick/s。
 *       门限随目标缩放正是「低目标也能追得稳」的原因，不能为少一个参数拍平成常数。</li>
 *   <li><b>掉速状态机可以删，但它那层电压冻结要留下</b>：模型里 depth 260 相同、回带 0.260 s
 *       相同、过冲 +10.2→+11.2，实测日志中 inDrop 时间占比仅 6.3%。它的积分冻结一半是死代码
 *       （进入条件是 |e|&gt;3·eBand，而积分门本来就是 |e|&lt;eBand，两者互斥），唯一实质作用是
 *       给电压采样一个冻结窗口。这不需要状态机：<b>{@code |e| ≥ eBand} 就等于「不在稳态」</b>，
 *       直接用它暂停电压 EMA 即可（见 {@link #updateBusVoltage}）。该写法与原状态机等效：
 *       掉速+跌压同发时过冲 +5.1/+6.6（2 V 深跌压 +6.6）对旧版 +5.4/+7.1；而只删不补是
 *       +12.0/+17.0，门限取 3·eBand 则只到 +10.0/+10.5（放得太早）。所以本类接受「无掉速态」，
 *       但<b>不</b>接受同一提交里其余三项删除。</li>
 * </ol>
 * <p><b>残余风险</b>：冻结窗口没有 hold-off，速度一回到到位带就恢复跟踪；若此刻电池 RC 回弹
 * 正处高点，s 会被压低约 1.5%（模型值），可接受。另外「持续 &gt;2 s 的深跌压」与深掉速同时
 * 出现时，误差门在带内不起作用，EMA 会正常跟下去——那已经是电池真没电的工况，本来就该跟。
 * 不要为了这种场景把滞回/超时/重播种三套时序搬回来。
 *
 * <p><b>量纲</b>：所有增益单位均为「占空比」，所以本类不直接读写硬件，调用方负责把
 * {@code DcMotorEx#getVelocity()} 的 tick/s 与 {@code VoltageSensor#getVoltage()} 的 V 传进来。
 *
 * <p><b>参数从哪来</b>：一套机器一套参数，见 {@link FlywheelParams}（实机有两套：大球 / 小球）。
 * 调用方每帧把 {@code FlywheelParams#applyTo(FlywheelCalculator)} 调一遍即可，Dashboard 改动实时生效。
 */
public class FlywheelCalculator {

    // ==================== 可调参数（由 FlywheelParams#applyTo 每帧同步） ====================
    //
    // 这里<b>不放工程默认值</b>：一套机器一套参数，唯一出处是 utility/Shooter/FlywheelParams.java
    // 里的 BIG / SMALL 两个块。调用方必须先 applyTo 再 calculate（Controllers 里都是这么做的），
    // 否则所有增益为 0、输出恒为 dutyMin。

    /** 静摩擦前馈 (占空比)，按 {@code kS·sgn(ω*)} 施加 */
    public double kS;
    /** 稳态前馈增益 (占空比)/(tick/s) */
    public double kff;
    /** 比例增益 (占空比)/(tick/s) */
    public double kp;
    /** 积分增益 (占空比)/(tick/s·s) */
    public double ki;
    /** I 项限幅 (占空比) */
    public double maxI;

    /** 输出下限 (占空比) */
    public double dutyMin;
    /** 输出上限 (占空比) */
    public double dutyMax;

    /** 速度量化台阶 (tick/s) */
    public double quantStep;
    /**
     * 稳态误差带比例：{@code eBand = max(settleFrac·|ω*|, 2·quantStep)}。
     *
     * <p>eBand 同时是<b>到位判据</b>与<b>积分门限</b>（{@code |e| < eBand} 才累加积分）。
     * 门限必须随目标缩放：曾被改成固定 iZone=120 tick/s 后被否决，因为低目标下等于
     * 把积分准入放宽 3 倍（0→600 过冲 +4.5→+11.0 tick/s）。
     */
    public double settleFrac;
    /** 连续落在误差带内多少帧算到位 */
    public int settleFrames;

    /**
     * 稳态速度滤波时间常数 (s)。
     *
     * <p>稳态用它压掉 20 tick/s 的量化噪声。SimpleShooter 分支曾删掉整个速度滤波、
     * 直接用原始误差做 P/I，结果 kp×quantStep=0.038 原样出现在 duty 上：
     * 实测回放稳态 duty 标准差 0.0053→0.0210（1750）、0.0076→0.0197（1600），峰峰 0.027→0.115。
     * 需要「更跟手」请调小 {@link #tauVMax}，不要删滤波。
     */
    public double tauVMax;
    /** 大误差速度滤波时间常数 (s) */
    public double tauVMin;
    /** 误差调度下拐点（倍 eBand） */
    public double eLoFrac;
    /** 误差调度上拐点（倍 eBand） */
    public double eHiFrac;

    // 注：第 4 代原有 dropEnterFrac（|e| > dropEnterFrac·eBand 进入掉速态）已随 inDrop 状态机
    // 一起删除（2026-11-06，见类注释「取舍记录」第 4 条）。不要用固定误差门限把它换个名字加回来。
    /** 带外冻结积分的最长时长 (s)：超时清零，防「误差长期卡在带外 → 积分永久失效」 */
    public double iFreezeTimeoutSec;

    /** 是否启用母线电压归一化（只乘前馈） */
    public boolean useVoltageNorm;
    /** 额定母线电压 (V) */
    public double vBusNominal;
    /** 归一化系数下限 */
    public double vScaleMin;
    /** 归一化系数上限 */
    public double vScaleMax;
    /** 母线电压低通时间常数 (s)：与「带外暂停吸收」一起构成电压瞬态防护，不要调小（见 updateBusVoltage） */
    public double tauBus;

    // ==================== 运行时状态 ====================

    /** 最近一次输出的占空比 */
    private double duty = 0;

    /** 最近一次控制量各分量（供遥测：静摩擦 / 前馈 / 比例 / 积分） */
    private double lastKSTerm = 0, lastFFTerm = 0, lastPTerm = 0, lastITerm = 0;

    /** 积分累计量 ∫e_f·dt (tick/s·s) */
    private double integral = 0;

    /** 滤波后的转速 (tick/s) */
    private double velocityFilt = 0;

    /** 最近一次原始误差 / 滤波后误差 (tick/s) */
    private double errorRaw = 0, errorFilt = 0;

    /** 当前稳态误差带 (tick/s) */
    private double eBand = 0;

    /** 当前实际使用的速度滤波时间常数 (s) */
    private double tauVUsed = 0;

    /** 母线电压滤波值 (V)，初值 NaN 表示尚未采样 */
    private double vBusFilt = Double.NaN;

    /** 当前电压归一化系数 */
    private double vScale = 1.0;

    /** 积分是否处于活跃（累加）状态 */
    private boolean integralActive = false;

    /** 积分冻结累计时长 (s) */
    private double iFrozenT = 0;

    /** 稳态判据：连续在带内的帧数 */
    private int settleCnt = 0;

    /** 上一次的目标转速，用于检测目标改变 */
    private double lastTargetVel = Double.NaN;

    /** 是否已完成首帧初始化（首帧直接用实测值播种，避免滤波器从 0 爬升） */
    private boolean initialized = false;

    /** dt 异常（首帧、长时间停顿、时钟回退）时使用的标称循环周期 (s) */
    public static final double DEFAULT_DT = 0.02;

    public FlywheelCalculator() {
        lastTime = now();
    }

    /** 上一次 calculate 的时间戳 (s)，仅 2 参数/3 参数入口使用 */
    private double lastTime;

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    // ==================== 计算 ====================

    /**
     * 单步计算（内部自行推进 dt，用单调时钟）。
     *
     * <p>每帧只可调用一次（内部计时器、滤波器与积分状态会推进）。
     *
     * @param targetVel  目标转速 (tick/s)，0 表示停转
     * @param currentVel 当前转速 (tick/s)
     * @param vBus       母线电压 (V)；≤1 或 NaN 视为无有效读数（沿用上次滤波值）
     * @return 占空比，已限幅到 [dutyMin, dutyMax]
     */
    public double calculate(double targetVel, double currentVel, double vBus) {
        double currentTime = now();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        return calculate(targetVel, currentVel, vBus, dt);
    }

    /**
     * 单步计算（显式 dt，含母线电压）—— 实机主入口 / 离线复算入口。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @param vBus       母线电压 (V)；≤1 或 NaN 视为无有效读数
     * @param dt         距上一步的时间 (s)；≤0 或 &gt;1 时用 {@link #DEFAULT_DT} 兜底
     * @return 占空比，已限幅到 [dutyMin, dutyMax]
     */
    public double calculate(double targetVel, double currentVel, double vBus, double dt) {
        if (dt <= 0 || dt > 1.0) {
            dt = DEFAULT_DT;
        }

        // ---- 首帧：直接用实测值播种滤波器，避免从 0 爬升造成假的「大误差」 ----
        if (!initialized) {
            velocityFilt = currentVel;
            vBusFilt = validVoltage(vBus) ? vBus : vBusNominal;
            lastTargetVel = targetVel;
            initialized = true;
        }

        // ---- 目标改变：清掉瞬态状态（不清积分——积分由停机/超时负责） ----
        if (targetVel != lastTargetVel) {
            lastTargetVel = targetVel;
            iFrozenT = 0.0;
            settleCnt = 0;
        }

        // ---- 误差与稳态误差带 ----
        errorRaw = targetVel - currentVel;
        eBand = bandFor(targetVel);

        // ---- 不维护掉速态 ----
        // 第 4 代在此做 inDrop 滞回（进 3·eBand、出 eBand），用来冻结积分与母线电压采样。
        // 该状态机已删除（2026-11-06，见类注释「取舍记录」第 4 条）：实测 inDrop 时间占比 6.3%，
        // 模型里删掉后跌深/回带时间/过冲基本不变。它对电压采样的冻结作用保留了下来，但不需要
        // 状态——updateBusVoltage 直接用 |e| ≥ eBand 判断「不在稳态」，效果与原状态机等效。
        // 这里只剩「带外冻结积分 + iFreezeTimeoutSec 兜底」，没有任何额外状态。

        // ---- 速度滤波：τ 由原始误差调度（不能用滤波后的误差，否则会自反馈滞回） ----
        tauVUsed = tauForError(Math.abs(errorRaw));
        double alpha = dt / (tauVUsed + dt);
        velocityFilt += alpha * (currentVel - velocityFilt);
        errorFilt = targetVel - velocityFilt;

        // ---- 母线电压低通（EMA；带外暂停吸收，见 updateBusVoltage）+ 归一化系数 ----
        updateBusVoltage(vBus, dt);
        vScale = 1.0;
        if (useVoltageNorm && validVoltage(vBusFilt)) {
            vScale = vBusNominal / vBusFilt;
            if (vScale < vScaleMin) {
                vScale = vScaleMin;
            } else if (vScale > vScaleMax) {
                vScale = vScaleMax;
            }
        }

        // ---- 前馈（静摩擦 + 稳态）：只有这两项乘电压归一化系数 ----
        // sgn(ω*)：ω*=0 时为 0，避免停转指令下静摩擦前馈把飞轮推转
        double ffSign = targetVel > 0 ? 1.0 : (targetVel < 0 ? -1.0 : 0.0);
        lastKSTerm = vScale * kS * ffSign;
        lastFFTerm = vScale * kff * targetVel;

        // ---- 比例 ----
        lastPTerm = kp * errorFilt;

        // ---- 积分：只在稳态误差带内累加；停机清零；带外冻结（超时强制清零） ----
        // 门限必须是相对量 eBand。SimpleShooter 换成固定 iZone=120 tick/s：1700 目标下两者等效
        // （eBand=40 的 3 倍），但 600 目标下等于把积分准入从 40 放宽到 120（3 倍），
        // 模型里 0→600 过冲 +4.5→+11.0、0→1000 +1.8→+10.5 tick/s。不要这么改。
        integralActive = targetVel != 0 && Math.abs(errorRaw) < eBand;
        if (integralActive) {
            integral += errorFilt * dt;
            iFrozenT = 0.0;
        } else if (targetVel == 0) {
            integral = 0.0;
            iFrozenT = 0.0;
        } else {
            iFrozenT += dt;
            if (iFreezeTimeoutSec > 0 && iFrozenT >= iFreezeTimeoutSec) {
                integral = 0.0; // 冻结超时：防「误差长期卡在带外 → 积分永久失效」
                iFrozenT = 0.0;
            }
        }
        double iTerm = ki * integral;
        // 抗饱和：对积分项回算限幅
        if (iTerm > maxI) {
            iTerm = maxI;
            if (ki != 0) {
                integral = maxI / ki;
            }
        } else if (iTerm < -maxI) {
            iTerm = -maxI;
            if (ki != 0) {
                integral = -maxI / ki;
            }
        }
        lastITerm = iTerm;

        // ---- 合成输出 ----
        double out = lastKSTerm + lastFFTerm + lastPTerm + lastITerm;
        if (out > dutyMax) {
            out = dutyMax;
        }
        if (out < dutyMin) {
            out = dutyMin;
        }
        duty = out;

        // ---- 到位判据：连续 settleFrames 帧误差在带内 ----
        boolean nearBand = targetVel != 0 && Math.abs(errorRaw) <= eBand;
        if (nearBand) {
            if (settleCnt < settleFrames) {
                settleCnt++;
            }
        } else {
            settleCnt = 0;
        }

        return out;
    }

    /**
     * 单步计算（内部推进 dt，<b>不含</b>母线电压）—— 兼容旧调用方式，实机请用带 vBus 的重载。
     *
     * <p>注意：Java 无法用「第三参数的含义」区分重载，因此<b>没有</b>
     * {@code calculate(t, ω, dt)} 这个三参数无电压版本。离线回放无电压列的旧日志请用
     * {@code calculate(t, ω, Double.NaN, dt)}（等价于电压归一化关闭）。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @return 占空比
     */
    public double calculate(double targetVel, double currentVel) {
        return calculate(targetVel, currentVel, Double.NaN);
    }

    // ==================== 内部工具 ====================

    /**
     * 稳态误差带：{@code max(settleFrac·|ω*|, 2·quantStep)}。
     *
     * <p>下限 2 个量化台阶是物理下限——再小就没有意义，编码器给不出那样的分辨率；
     * 上限保证高转速下相对误差不会被压到 0（否则永远「不到位」）。
     *
     * @param targetVel 目标转速 (tick/s)
     * @return 误差带 (tick/s)
     */
    private double bandFor(double targetVel) {
        double floor = 2.0 * quantStep;
        double relative = settleFrac * Math.abs(targetVel);
        return relative > floor ? relative : floor;
    }

    /**
     * 误差调度的速度滤波时间常数：{@code |e| ≤ eLoFrac·eBand} 用 tauVMax，
     * {@code |e| ≥ eHiFrac·eBand} 用 tauVMin，中间线性过渡。
     *
     * <p>拐点必须落在 eBand 附近：残余过冲 ≈ 饱和加速度 × τ(eBand)，
     * 若 eBand 处 τ 还停在 30 ms，1700 目标下会留下 1146×0.03 ≈ 34 tick/s 的过冲。
     *
     * @param absError 原始误差绝对值 (tick/s)
     * @return τ (s)
     */
    private double tauForError(double absError) {
        double lo = eLoFrac * eBand;
        double hi = eHiFrac * eBand;
        double blend;
        if (hi <= lo) {
            blend = 1.0;
        } else {
            blend = (absError - lo) / (hi - lo);
            if (blend < 0) {
                blend = 0;
            } else if (blend > 1) {
                blend = 1;
            }
        }
        return tauVMax + (tauVMin - tauVMax) * blend;
    }

    /** @return 电压读数是否有效 */
    private static boolean validVoltage(double v) {
        return !Double.isNaN(v) && v > 1.0;
    }

    /**
     * 母线电压低通：纯 EMA，τ={@link #tauBus}；读数无效时保留上一次的滤波值。
     *
     * <p><b>不在稳态时暂停更新</b>（{@code |e| ≥ eBand}）：发射瞬间速度被打掉几百 tick/s，
     * 远超到位带，所以整个发射窗口内 {@code vBusFilt} 保持「发射前」的值；速度回到带内后
     * 才继续跟踪。这是旧 inDrop 状态机唯一有实效的一半（另一半积分冻结与 |e|&lt;eBand 的
     * 积分门互斥，是死代码），去掉状态机后用误差门直接表达，不需要滞回、hold-off、超时重播种。
     *
     * <p>为什么需要这个窗口：单发把母线从 12.4 V 拉到 10.4 V 约 0.4 s，而 τ=1.0 s 的 EMA
     * 只走完 33%。不暂停的话，那 1/3 的跌落会被一直记着到恢复期末——此时速度已经追平、
     * P 项≈0，duty≈ff+i，多出来的 s 就是纯粹的助推。掉速+跌压同发场景（1900 目标、冲量 270、
     * 12.4→10.4 V/0.4 s）实测：暂停 +5.1 tick/s，不暂停 +12.0；2 V 深跌压 +6.6 对 +17.0。
     * 注意门限不能用 3·eBand（那等于旧状态机的进入条件，放得太早，只到 +10.0/+10.5）。
     *
     * <p><b>不要</b>退化成 {@code s = 12/V_raw}（被否决的 SimpleShooter 写法）：同样的 0.4 s
     * 跌落让 s 立刻跳 1.000→1.071，恢复期末过冲 +15.8→+24.1。
     *
     * @param vBus 原始电压读数 (V)
     * @param dt   步长 (s)
     */
    private void updateBusVoltage(double vBus, double dt) {
        if (!validVoltage(vBus)) {
            return;
        }
        if (!validVoltage(vBusFilt)) {
            vBusFilt = vBus; // 首次有效读数直接播种，避免从 0 爬升
            return;
        }
        // 不在到位带内 = 正在被扰动（发射、堵转、急变速），此拍不吸收电压读数。
        // 目标为 0（停机滑行）时不判：那会儿误差天然很大，但电压读数仍然是可信的。
        if (lastTargetVel != 0.0 && Math.abs(errorRaw) >= eBand) {
            return;
        }
        double alpha = dt / (tauBus + dt);
        if (alpha > 1) {
            alpha = 1;
        }
        vBusFilt += alpha * (vBus - vBusFilt);
    }

    // ==================== 状态重置 ====================

    /** 重置全部瞬态状态（从开环切回闭环前调用），假定当前转速为 0 */
    public void reset() {
        reset(0.0);
    }

    /**
     * 重置全部瞬态状态，并用给定转速播种速度滤波器（避免切回闭环的瞬间产生假的误差冲击）。
     *
     * @param currentVelocity 当前实测转速 (tick/s)
     */
    public void reset(double currentVelocity) {
        integral = 0.0;
        velocityFilt = currentVelocity;
        errorRaw = 0.0;
        errorFilt = 0.0;
        eBand = 0.0;
        tauVUsed = tauVMax;
        duty = 0.0;
        lastKSTerm = 0.0;
        lastFFTerm = 0.0;
        lastPTerm = 0.0;
        lastITerm = 0.0;
        integralActive = false;
        iFrozenT = 0.0;
        settleCnt = 0;
        lastTargetVel = Double.NaN;
        lastTime = now();
    }

    // ==================== 读取 ====================

    /** @return 最近一次输出的占空比 */
    public double getDuty() {
        return duty;
    }

    /** @return 内部积分累计量 ∫e_f·dt (tick/s·s) */
    public double getIntegral() {
        return integral;
    }

    /** @return 最近一次的静摩擦前馈分量（占空比，已乘电压归一化系数） */
    public double getKSTerm() {
        return lastKSTerm;
    }

    /** @return 最近一次的前馈分量 kff·ω*（占空比，已乘电压归一化系数） */
    public double getFfTerm() {
        return lastFFTerm;
    }

    /** @return 最近一次的比例分量 kp·e_f（占空比） */
    public double getPTerm() {
        return lastPTerm;
    }

    /** @return 最近一次的积分分量（占空比，已限幅） */
    public double getITerm() {
        return lastITerm;
    }

    /** @return 滤波后的转速 (tick/s) */
    public double getVelocityFiltered() {
        return velocityFilt;
    }

    /** @return 最近一次的原始误差 ω*−ω (tick/s) */
    public double getErrorRaw() {
        return errorRaw;
    }

    /** @return 最近一次的滤波后误差 ω*−ωf (tick/s) */
    public double getErrorFiltered() {
        return errorFilt;
    }

    /** @return 当前稳态误差带 (tick/s) */
    public double getErrorBand() {
        return eBand;
    }

    /** @return 当前使用的速度滤波时间常数 (s) */
    public double getTauV() {
        return tauVUsed;
    }

    /** @return 母线电压滤波值 (V)；未采样时为 NaN */
    public double getBusVoltageFiltered() {
        return vBusFilt;
    }

    /** @return 当前电压归一化系数 s（未启用恒为 1） */
    public double getVScale() {
        return vScale;
    }

    // 注：isVoltageFrozen() / isInDrop() 随状态机一并删除（2026-11-06）。
    // 调用方遥测列 inDrop / indrop / drop_g4 已同步移除。若 CSV 里还想要这一列，
    // 只能自己按 |e| 判个门限——那就是把已论证过要删的状态机重新加回来，别这么做。

    /** @return 积分当前是否在累加（false = 冻结或停机） */
    public boolean isIntegralActive() {
        return integralActive;
    }

    /** @return 是否已到位（连续 {@link #settleFrames} 帧误差在带内） */
    public boolean isSettled() {
        return settleCnt >= settleFrames;
    }

    /**
     * @return 量化极限环幅值下限 kp × 一个量化台阶（占空比）。
     *         工程上应 ≤0.05；超过 0.1 会跑出明显的量化极限环，须降 kp 或加滤波。
     */
    public double getKpQuantization() {
        return kp * quantStep;
    }
}
