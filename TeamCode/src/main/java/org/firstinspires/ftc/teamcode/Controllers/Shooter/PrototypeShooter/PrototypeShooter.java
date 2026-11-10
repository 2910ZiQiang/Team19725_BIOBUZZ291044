package org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelCalculator;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams;

/**
 * 原型发射机构（飞轮）控制器 —— 速度闭环，直接输出占空比。
 *
 * <p><b>职责划分</b>：本类只负责「电机 + 遥测 + 参数面板」，控制律本身在
 * {@link FlywheelCalculator}（{@code utility/Shooter}，纯计算、无硬件依赖），
 * 参数在 {@link FlywheelParams}（一台机器一套，实机两套：{@code FlywheelParams.BIG} /
 * {@code FlywheelParams.SMALL}）。本类每帧 {@code params.applyTo(calculator)}，
 * 因此 Dashboard 上改参数依旧实时生效。
 *
 * <p><b>控制律（第 4 代，2026-10-22）</b>：
 * <pre>
 *   duty = s·(kS·sgn(ω*) + kff·ω*) + kp·(ω*-ωf) + ki·∫(ω*-ωf)
 *   s    = clamp(V_nom / V_busFilt, 0.85, 1.25)   （只乘前馈；未启用时为 1）
 *   ωf   = EMA(ω, τ(|ω*-ω|))                      （稳态 30 ms，大误差 8 ms）
 * </pre>
 * 掉速期间积分<b>冻结</b>（既不加也不清），误差回到稳态带内才继续累加；
 * 第 3 代的 boost 掉速补偿已删除（实测其饱和段贡献恒为 0，唯一作用是抹掉退饱和段、
 * 结构性造成 +50~90 tick/s 过冲）。
 *
 * <p>电机置于 {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}，{@code setPower} 直接控制占空比。
 *
 * <p><b>遥测约定</b>：标准遥测固定 5 行（目标 / 当前 / 占空比 / 电流 / 电压），
 * 其余「需要研究的量」分行列在 {@link #setTelemetry()} 的诊断段里，
 * 默认折叠（{@link #telemetryDetailed}=false），复盘时在 Dashboard 勾开。
 * 需要研究的量清单（与类同名的 Guide 里也有）：
 * <ol>
 *   <li>{@code error raw / filtered}：原始与滤波后误差，判断滤波是否造成滞后</li>
 *   <li>{@code velocity filtered}：滤波后转速，与原始转速之差就是滤波吃掉的量</li>
 *   <li>{@code tauV}：当前实际滤波时间常数，验证误差调度是否按设计生效</li>
 *   <li>{@code eBand}：当前稳态误差带，到位与积分门限的共同基准</li>
 *   <li>{@code vScale / vBusFilt}：电压归一化系数与滤波电压，判断电量估计是否跟手</li>
 *   <li>{@code kS / FF / P / I term}：控制量分解，判断积分是否长期负重</li>
 *   <li>{@code kp×step}：量化极限环幅值下限，&gt;0.05 应立即告警</li>
 *   <li>{@code integralActive / settled}：两个状态位（掉速态已随状态机删除）</li>
 * </ol>
 */
@Config
public class PrototypeShooter {

    /** 默认电机名 */
    private static final String DEFAULT_MOTOR_NAME = "shooter";

    public DcMotorEx motor;

    private final Telemetry telemetry;

    /** 母线电压传感器（硬件配置里的全部电压传感器，取正读数最小值，与官方 ConceptTelemetry 一致） */
    private final Iterable<VoltageSensor> voltageSensors;

    /** 该飞轮的完整参数集 */
    private final FlywheelParams params;

    /** 速度环计算器（纯计算，本类每帧把参数灌进去） */
    private final FlywheelCalculator calculator = new FlywheelCalculator();

    /** 遥测前缀（大球/小球两个实例必须可区分） */
    private final String name;

    // ==================== 可调面板 (FTC Dashboard 实时生效) ====================

    /** 目标转速 (tick/s)，0 表示停转 */
    public static int targetVelocity = 0;

    /** 是否在标准 5 行之外展开诊断行（控制量分解 / 滤波器状态 / 量化告警） */
    public static boolean telemetryDetailed = false;

    /**
     * 默认构造：电机名 {@code shooter}，参数用大球那一套（{@link FlywheelParams#BIG}）。
     *
     * @param hardwareMap 硬件映射
     * @param telemetry   遥测
     */
    public PrototypeShooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, DEFAULT_MOTOR_NAME, FlywheelParams.BIG);
    }

    /**
     * @param hardwareMap 硬件映射
     * @param telemetry   遥测
     * @param motorName   飞轮电机设备名
     * @param params      该飞轮的完整参数集
     */
    public PrototypeShooter(HardwareMap hardwareMap, Telemetry telemetry, String motorName, FlywheelParams params) {
        this.telemetry = telemetry;
        this.params = (params != null) ? params : FlywheelParams.BIG;
        this.name = motorName;
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.voltageSensors = hardwareMap.voltageSensor;

        motor.setDirection(this.params.motorReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // 软件闭环：关闭固件速度环，让 setPower 直接控制占空比
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 飞轮惯性大，停转后让其自然滑行
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.params.applyTo(calculator);
        calculator.reset();
    }

    // ==================== 控制 ====================

    /**
     * 设置目标转速。
     *
     * @param velocity 目标转速 (tick/s)，0 表示停转
     */
    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
    }

    /** 停转（目标转速置 0，下一帧 update 会收油门并清积分） */
    public void stop() {
        targetVelocity = 0;
    }

    /**
     * 速度环单步计算：输入目标与当前转速，输出占空比（内部推进 dt）。
     *
     * <p>每帧只可调用一次（内部计时器、滤波器与积分状态会推进）。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @return 占空比，已限幅
     */
    public double calculate(double targetVel, double currentVel) {
        params.applyTo(calculator);
        return calculator.calculate(targetVel, currentVel, getBusVoltage());
    }

    /** 每帧调用一次：计算占空比并下发给电机 */
    public void update() {
        motor.setPower(calculate(targetVelocity, motor.getVelocity()));
    }

    /**
     * 直接开环设定功率，绕过闭环（供 kS/kff 的稳态回归辨识使用）。
     * 调用后下一帧 {@link #update()} 会重新接管输出；切回闭环前请先 {@link #resetController()}。
     *
     * @param power 电机功率 [-1, 1]
     */
    public void setOpenLoopPower(double power) {
        motor.setPower(power);
    }

    /** 重置闭环控制器的瞬态状态（从开环切回闭环前调用） */
    public void resetController() {
        calculator.reset(motor.getVelocity());
    }

    // ==================== 读取 ====================

    /** @return 该飞轮的参数集（整定/标定程序可直接改字段） */
    public FlywheelParams getParams() {
        return params;
    }

    /** @return 速度环计算器（Tester 读取中间量用） */
    public FlywheelCalculator getCalculator() {
        return calculator;
    }

    /** @return 当前转速 (tick/s) */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** @return 最近一次下发的占空比 */
    public double getDuty() {
        return calculator.getDuty();
    }

    /** @return 目标转速 (tick/s) */
    public int getTargetVelocity() {
        return targetVelocity;
    }

    /** @return 速度误差 (tick/s) */
    public double getError() {
        return targetVelocity - getVelocity();
    }

    /** @return 是否已到位（连续若干帧误差在稳态带内） */
    public boolean isSettled() {
        return calculator.isSettled();
    }

    /** @return 电机电流 (A) */
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * 读取母线电压 (V)：遍历全部电压传感器，取<b>所有正读数中的最小值</b>，
     * 与官方示例 {@code ConceptTelemetry#getBatteryVoltage()} 一致。
     *
     * @return 母线电压 (V)；无有效读数时返回 {@link Double#NaN}
     */
    public double getBusVoltage() {
        double result = -1;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = (result < 0) ? voltage : Math.min(result, voltage);
            }
        }
        return result < 0 ? Double.NaN : result;
    }

    // ==================== 遥测 ====================

    /**
     * 标准飞轮遥测：<b>固定 5 行</b>（目标转速 / 当前转速 / 占空比 / 电流 / 电压）。
     *
     * <p>需要研究的量按 {@link #telemetryDetailed} 展开成分行诊断段（见类注释的清单）。
     * 调用方需自行 {@code telemetry.update()}。
     */
    public void setTelemetry() {
        telemetry.addData(name + " target (tick/s)", targetVelocity);
        telemetry.addData(name + " velocity (tick/s)", "%.0f", getVelocity());
        telemetry.addData(name + " power (duty)", "%.4f", getDuty());
        telemetry.addData(name + " current (A)", "%.2f", getCurrent());
        telemetry.addData(name + " voltage (V)", "%.2f", getBusVoltage());

        if (telemetryDetailed) {
            addDiagnosticTelemetry();
        }
    }

    /**
     * 详细遥测：标准 5 行 + 诊断段。供整定类程序直接调用（不依赖静态开关）。
     *
     * <p>调用方需自行 {@code telemetry.update()}。
     */
    public void setTelemetryDetailed() {
        setTelemetryCore();
        addDiagnosticTelemetry();
    }

    /** 5 行核心遥测，供 {@link #setTelemetryDetailed()} 复用 */
    private void setTelemetryCore() {
        telemetry.addData(name + " target (tick/s)", targetVelocity);
        telemetry.addData(name + " velocity (tick/s)", "%.0f", getVelocity());
        telemetry.addData(name + " power (duty)", "%.4f", getDuty());
        telemetry.addData(name + " current (A)", "%.2f", getCurrent());
        telemetry.addData(name + " voltage (V)", "%.2f", getBusVoltage());
    }

    /**
     * 诊断段：需要研究的量逐行列出。
     *
     * <p>顺序按「看这个量是为了回答什么问题」排列：误差 → 滤波 → 积分/到位状态 →
     * 电压归一化 → 控制量分解 → 量化告警。
     */
    private void addDiagnosticTelemetry() {
        telemetry.addLine("-- " + name + " research --");
        // 1. 误差：原始 vs 滤波
        telemetry.addData(name + " error raw (tick/s)", "%.0f", calculator.getErrorRaw());
        telemetry.addData(name + " error filt (tick/s)", "%.0f", calculator.getErrorFiltered());
        // 2. 滤波：滤波后转速与当前实际 τ
        telemetry.addData(name + " velocity filt (tick/s)", "%.0f", calculator.getVelocityFiltered());
        telemetry.addData(name + " tauV (ms)", "%.1f", calculator.getTauV() * 1000.0);
        // 3. 状态位：积分活跃 / 到位（掉速态已删除，见 FlywheelCalculator 类注释）
        telemetry.addData(name + " eBand (tick/s)", "%.0f", calculator.getErrorBand());
        telemetry.addData(name + " integral active", calculator.isIntegralActive());
        telemetry.addData(name + " settled", calculator.isSettled());
        // 4. 电压归一化
        telemetry.addData(name + " vBus filt (V)", "%.2f", calculator.getBusVoltageFiltered());
        telemetry.addData(name + " vScale", "%.3f", calculator.getVScale());
        // 5. 控制量分解
        telemetry.addData(name + " kS term", "%.4f", calculator.getKSTerm());
        telemetry.addData(name + " FF term", "%.4f", calculator.getFfTerm());
        telemetry.addData(name + " P term", "%.4f", calculator.getPTerm());
        telemetry.addData(name + " I term", "%.4f", calculator.getITerm());
        telemetry.addData(name + " I integral", "%.2f", calculator.getIntegral());
        // 6. 量化告警
        double kpQuant = calculator.getKpQuantization();
        String flag = (kpQuant > 0.05) ? "  <-- HIGH" : "";
        telemetry.addData(name + " kp*step", "%.4f%s", kpQuant, flag);
    }
}
