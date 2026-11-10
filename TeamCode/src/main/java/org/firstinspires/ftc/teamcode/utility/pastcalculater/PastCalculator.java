package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelCalculator;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams;

/**
 * 多代飞轮计算器的公共骨架（**精简版**，只用于对比实验与离线复算，不参与正式控制）。
 *
 * <p>统一入口 {@link #calculate(double, double, double, double)}：
 * 子类只实现 {@link #compute}，基类负责 dt 兜底、误差计算、输出限幅、
 * **统一的稳态判据**与分量快照。统一判据很重要——否则各代“settled”定义不同，图就没法并排比。
 *
 * <p>统一判据（与第 4 代一致）：
 * {@code |ω*−ω| ≤ max(settleFrac·|ω*|, 2·quantStep)} 连续 {@code settleFrames} 帧。
 */
public abstract class PastCalculator {

    protected static final double DEFAULT_DT = 0.02;

    // ---- 每帧写出的分量快照（getter 见下） ----
    protected double duty;
    protected double kSTerm;
    protected double ffTerm;
    protected double pTerm;
    protected double iTerm;
    protected double boostTerm;
    protected double integral;

    protected double target;
    protected double error;
    protected double errorFiltered;
    protected double velocityFiltered;

    protected boolean inDrop;
    protected boolean iActive;

    private int settleCount;

    /** 有符号平方根式饱和辅助：clamp(v, lo, hi)。 */
    protected static double clamp(double v, double lo, double hi) {
        return v > hi ? hi : (v < lo ? lo : v);
    }

    /** 显示名，进 CSV 表头（如 "duty_g1"）。 */
    public abstract String name();

    /** 清空全部内部状态，并以 {@code currentVelocity} 播种（掉速检测用）。 */
    public abstract void reset(double currentVelocity);

    /**
     * 单步计算。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @param vBus       母线电压 (V)，未知传 {@link Double#NaN}
     * @param dt         距上一步的时间 (s)；≤0 或 &gt;1 用 {@value #DEFAULT_DT} 兜底
     * @return 占空比，已限幅
     */
    public final double calculate(double targetVel, double currentVel, double vBus, double dt) {
        if (dt <= 0 || dt > 1.0) {
            dt = DEFAULT_DT;
        }
        target = targetVel;
        error = targetVel - currentVel;
        errorFiltered = error;
        velocityFiltered = currentVel;

        compute(targetVel, currentVel, vBus, dt);

        duty = clamp(duty, -1.0, 1.0); // 各代自己的 dutyMin/dutyMax 已在 compute 内限过

        // ---- 统一稳态判据 ----
        if (targetVel == 0) {
            settleCount = 0;
        } else if (Math.abs(error) <= errorBand()) {
            if (settleCount < settleFrames()) {
                settleCount++;
            }
        } else {
            settleCount = 0;
        }
        return duty;
    }

    /** 本代的控制律主体：读 {@link #target}/{@link #error}，写 {@link #duty} 与各分量。 */
    protected abstract void compute(double targetVel, double currentVel, double vBus, double dt);

    /** 稳态误差带；默认沿用第 4 代的定义。 */
    protected double errorBand() {
        return Math.max(0.02 * Math.abs(target), 2.0 * quantStep());
    }

    /** 稳态判据需要连续满足的帧数。 */
    protected int settleFrames() {
        return 10;
    }

    /** 编码器量化台阶 (tick/s)，子类按自己参数覆盖。 */
    protected double quantStep() {
        return 20.0;
    }

    /** 清空公共状态（子类 reset 里调用）。 */
    protected void clearCommonState() {
        duty = 0;
        kSTerm = 0;
        ffTerm = 0;
        pTerm = 0;
        iTerm = 0;
        boostTerm = 0;
        integral = 0;
        target = 0;
        error = 0;
        errorFiltered = 0;
        velocityFiltered = 0;
        inDrop = false;
        iActive = false;
        settleCount = 0;
    }

    // ---------------------------------------------------------------- getter

    public double getDuty() {
        return duty;
    }

    public double getKSTerm() {
        return kSTerm;
    }

    public double getFfTerm() {
        return ffTerm;
    }

    public double getPTerm() {
        return pTerm;
    }

    public double getITerm() {
        return iTerm;
    }

    public double getBoostTerm() {
        return boostTerm;
    }

    public double getIntegral() {
        return integral;
    }

    public double getError() {
        return error;
    }

    public double getErrorFiltered() {
        return errorFiltered;
    }

    public double getVelocityFiltered() {
        return velocityFiltered;
    }

    public double getErrorBand() {
        return errorBand();
    }

    public boolean isInDrop() {
        return inDrop;
    }

    public boolean isIntegralActive() {
        return iActive;
    }

    public boolean isBoostActive() {
        return boostTerm > 0;
    }

    public boolean isSettled() {
        return settleCount >= settleFrames();
    }

    /** 一行分量快照，便于遥测/日志。 */
    public String snapshot() {
        return String.format(java.util.Locale.US,
                "d=%.4f ff=%.4f p=%.4f i=%.4f boost=%.4f e=%.1f%s%s",
                duty, kSTerm + ffTerm, pTerm, iTerm, boostTerm, error,
                inDrop ? " DROP" : "", iActive ? " I" : "");
    }

    // ================================================================ 第 1/2 代

    /**
     * 第 1/2 代：{@code PIDSVAController} 的精简版。
     * <pre>
     *   d = kP·e + kI·∫e + kD·de/dt + kS·sgn(ω*) + kV·ω* + kA·d(ω*)/dt
     *   |e| &lt; iZone 才累加积分，否则清零
     * </pre>
     */
    public static final class Gen1 extends PastCalculator {
        private double lastError;
        private double lastTarget;
        private boolean first = true;

        @Override
        public String name() {
            return "Gen1";
        }

        @Override
        public void reset(double currentVelocity) {
            clearCommonState();
            lastError = 0;
            lastTarget = 0;
            first = true;
        }

        @Override
        protected void compute(double targetVel, double currentVel, double vBus, double dt) {
            double dTarget = first ? 0 : (targetVel - lastTarget) / dt;
            double dError = first ? 0 : (error - lastError) / dt;
            lastTarget = targetVel;
            lastError = error;
            first = false;

            // ---- 积分：仅 |e| < iZone 且 target != 0 ----
            iActive = targetVel != 0 && Math.abs(error) < Gen1Params.iZone;
            if (iActive) {
                integral += error * dt;
            } else {
                integral = 0;
            }
            iTerm = clamp(Gen1Params.kI * integral, -Gen1Params.maxI, Gen1Params.maxI);
            if (Gen1Params.kI != 0) {
                integral = iTerm / Gen1Params.kI; // 回算，防饱和
            }

            // ---- 前馈 ----
            double sgn = targetVel > 0 ? 1 : (targetVel < 0 ? -1 : 0);
            kSTerm = Gen1Params.kS * sgn;
            ffTerm = Gen1Params.kV * targetVel + Gen1Params.kA * dTarget;

            pTerm = Gen1Params.kP * error;
            inDrop = false;

            duty = kSTerm + ffTerm + pTerm + iTerm + Gen1Params.kD * dError;
            duty = clamp(duty, Gen1Params.dutyMin, Gen1Params.dutyMax);
        }

        @Override
        protected double quantStep() {
            return Gen1Params.quantStep;
        }

        @Override
        protected double errorBand() {
            return Math.max(Gen1Params.settleFrac * Math.abs(target), 2.0 * Gen1Params.quantStep);
        }

        @Override
        protected int settleFrames() {
            return Gen1Params.settleFrames;
        }
    }

    // ================================================================ 第 3 代

    /**
     * 第 3 代（boost 版）：与被第 4 代删除的实现**逐行等价**的精简版。
     * <pre>
     *   d = kS·sgn + kff·ω* + kp·e + ki·∫e + boost
     *   boost = boostActive ? clamp(kBoost·e, 0, boostMax) : 0
     *   掉速「武装」：|e| &lt; dropThreshold 过才算进入稳态，之后
     *                e &gt; dropThreshold 或 dω/dt &lt; −dropRateThreshold 触发
     * </pre>
     */
    public static final class Gen3 extends PastCalculator {
        private double previousVelocity;
        private double lastTargetVel;
        private boolean dropArmed;
        private boolean boostActive;
        private double boostTimer;

        @Override
        public String name() {
            return "Gen3";
        }

        @Override
        public void reset(double currentVelocity) {
            clearCommonState();
            previousVelocity = currentVelocity;
            lastTargetVel = 0;
            dropArmed = false;
            boostActive = false;
            boostTimer = 0;
        }

        @Override
        protected void compute(double targetVel, double currentVel, double vBus, double dt) {
            double dVelocity = (currentVel - previousVelocity) / dt;
            previousVelocity = currentVel;

            // ---- 武装：只有进入稳态后才允许触发 ----
            if (targetVel != lastTargetVel) {
                dropArmed = false;
                lastTargetVel = targetVel;
            }
            if (Math.abs(error) < Gen3Params.dropThreshold) {
                dropArmed = true;
            }

            // ---- 掉速检测与 boost ----
            inDrop = dropArmed && targetVel != 0
                    && (error > Gen3Params.dropThreshold
                    || dVelocity < -Gen3Params.dropRateThreshold);
            if (inDrop) {
                boostActive = true;
                boostTimer = 0;
                integral = 0; // 触发补偿时清零，避免叠加
            }
            if (boostActive) {
                boostTimer += dt;
                if (boostTimer >= Gen3Params.boostDurationSec || error <= 0) {
                    boostActive = false;
                }
            }

            // ---- 积分：iZone 内才积，带外清零 ----
            iActive = targetVel != 0 && Math.abs(error) < Gen3Params.iZone;
            if (iActive) {
                integral += error * dt;
            } else {
                integral = 0;
            }
            double i = Gen3Params.ki * integral;
            if (i > Gen3Params.maxI) {
                i = Gen3Params.maxI;
                if (Gen3Params.ki != 0) {
                    integral = Gen3Params.maxI / Gen3Params.ki;
                }
            } else if (i < -Gen3Params.maxI) {
                i = -Gen3Params.maxI;
                if (Gen3Params.ki != 0) {
                    integral = -Gen3Params.maxI / Gen3Params.ki;
                }
            }
            iTerm = i;

            double boost = boostActive
                    ? Math.max(0.0, Math.min(Gen3Params.kBoost * error, Gen3Params.boostMax))
                    : 0.0;
            boostTerm = boost;

            double sgn = targetVel > 0 ? 1 : (targetVel < 0 ? -1 : 0);
            kSTerm = Gen3Params.kS * sgn;
            ffTerm = Gen3Params.kff * targetVel;
            pTerm = Gen3Params.kp * error;

            duty = kSTerm + ffTerm + pTerm + iTerm + boostTerm;
            duty = clamp(duty, Gen3Params.dutyMin, Gen3Params.dutyMax);
        }

        @Override
        protected double quantStep() {
            return Gen3Params.quantStep;
        }

        @Override
        protected double errorBand() {
            return Math.max(Gen3Params.settleFrac * Math.abs(target), 2.0 * Gen3Params.quantStep);
        }

        @Override
        protected int settleFrames() {
            return Gen3Params.settleFrames;
        }
    }

    // ================================================================ 第 4 代

    /**
     * 第 4 代（现行）：直接包一层 {@link FlywheelCalculator}，
     * 参数来自 {@link Gen4Params}（Dashboard 可改），每帧灌一次。
     */
    public static final class Gen4 extends PastCalculator {
        private final FlywheelCalculator calc = new FlywheelCalculator();
        private final FlywheelParams params = FlywheelParams.copyOf(FlywheelParams.BIG);

        public Gen4() {
            Gen4Params.applyTo(params);
            params.applyTo(calc);
            calc.reset(0);
        }

        @Override
        public String name() {
            return "Gen4";
        }

        /** 电压缩放系数 {@code s = clamp(V_nom/V_busFilt, vScaleMin, vScaleMax)}（第 4 代独有）。 */
        public double getVScale() {
            return calc.getVScale();
        }

        @Override
        public void reset(double currentVelocity) {
            clearCommonState();
            Gen4Params.applyTo(params);
            params.applyTo(calc);
            calc.reset(currentVelocity);
        }

        @Override
        protected void compute(double targetVel, double currentVel, double vBus, double dt) {
            Gen4Params.applyTo(params); // Dashboard 改动即时生效
            params.applyTo(calc);
            duty = calc.calculate(targetVel, currentVel, vBus, dt);

            kSTerm = calc.getKSTerm();
            ffTerm = calc.getFfTerm();
            pTerm = calc.getPTerm();
            iTerm = calc.getITerm();
            integral = calc.getIntegral();
            error = calc.getErrorRaw();
            errorFiltered = calc.getErrorFiltered();
            velocityFiltered = calc.getVelocityFiltered();
            inDrop = false; // 第 4 代已删掉速状态机（2026-11-06）：base.inDrop 只由 Gen1/Gen3 使用
            iActive = calc.isIntegralActive();
        }

        @Override
        protected double errorBand() {
            double band = calc.getErrorBand();
            return band > 0 ? band : super.errorBand();
        }

        @Override
        protected int settleFrames() {
            return Gen4Params.settleFrames;
        }
    }

    // ================================================================ 工厂

    /** 按代际创建计算器（已 reset 到 0 转速状态）。 */
    public static PastCalculator create(PastGen gen) {
        PastCalculator c;
        switch (gen) {
            case GEN1_PIDSVA:
                c = new Gen1();
                break;
            case GEN3_BOOST:
                c = new Gen3();
                break;
            case GEN4_CURRENT:
            default:
                c = new Gen4();
                break;
        }
        c.reset(0);
        return c;
    }
}
