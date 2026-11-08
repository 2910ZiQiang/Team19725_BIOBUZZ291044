package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelCalculator;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 单个发射器控制器：飞轮速度闭环 + 俯仰舵机 + 扳机舵机。
 *
 * <p><b>两个发射器共用一套控制律、各用一套参数</b>：实机是一个大球飞轮 + 一个小球飞轮，
 * 二者惯量、齿比、静摩擦都不同。所以飞轮参数不是静态裸字段，而是
 * {@link Params#flywheel}（类型 {@link FlywheelParams}）——大球持
 * {@code FlywheelParams.BIG}、小球持 {@code FlywheelParams.SMALL}，
 * 两套参数在 Dashboard 上独立可见、独立整定。
 *
 * <p><b>控制律与 {@code Controllers.Shooter.PrototypeShooter} 完全一致</b>（第 4 代，2026-10-22）：
 * <pre>
 *   duty = s·(kS·sgn(ω*) + kff·ω*) + kp·(ω*-ωf) + ki·∫(ω*-ωf)
 * </pre>
 * 计算逻辑全部在 {@link FlywheelCalculator}（{@code utility/Shooter}）里，本类每帧
 * {@code params.flywheel.applyTo(calculator)} 后调用一次 {@code calculate(...)}，
 * 因此台架上用 PrototypeShooter 标出来的参数可以原样搬到这里，两份代码不会漂移。
 *
 * <p><b>单位约定</b>：
 * <ul>
 *   <li>飞轮速度：{@link DcMotorEx#getVelocity()} 的默认单位 tick/s</li>
 *   <li>俯仰与扳机：舵机位置 [0, 1]（舵机空间，不一定是真实发射角度）</li>
 * </ul>
 *
 * <p><b>就绪判据</b>（{@link #update(int, double, boolean)} 的返回值）要求
 * 飞轮转速与俯仰舵机同时到位。俯仰舵机无位置反馈，因此以「目标变更后经过
 * {@link Params#pitchSettleTime} 秒」作为其到位的近似判据；飞轮侧用「误差在稳态带内」
 * 判定（{@link FlywheelCalculator#isSettled()}，连续 10 帧 ≈ 0.1 s）。
 */
@Config
public class Shooter {

    // ==================== 超参数 ====================

    /**
     * 单个离散仰角档位：舵机位置、适用射程区间、射程 → 飞轮转速的线性回归模型。
     *
     * <p>仰角只允许取有限个离散档位，每档对应固定的舵机位置；
     * 射程落在 {@code [minRange, maxRange]} 内时，用 {@code speed = speedSlope·range + speedIntercept}
     * 计算所需飞轮转速。所有数值均需实机标定。
     */
    public static class PitchModel {
        /** 俯仰舵机位置 */
        public double pitchPos;
        /** 适用射程下限 (英寸) */
        public double minRange;
        /** 适用射程上限 (英寸) */
        public double maxRange;
        /** 线性模型斜率 (tick/s)/in */
        public double speedSlope;
        /** 线性模型截距 (tick/s) */
        public double speedIntercept;

        /** 供 Forge / Dashboard 反射使用的空构造 */
        public PitchModel() {
        }

        public PitchModel(double pitchPos, double minRange, double maxRange,
                          double speedSlope, double speedIntercept) {
            this.pitchPos = pitchPos;
            this.minRange = minRange;
            this.maxRange = maxRange;
            this.speedSlope = speedSlope;
            this.speedIntercept = speedIntercept;
        }

        /** @return 该档位的射程区间是否包含 {@code range} */
        public boolean covers(double range) {
            return range >= minRange && range <= maxRange;
        }

        /** @return 该档位在射程 {@code range} 下所需的飞轮转速 (tick/s) */
        public int speedFor(double range) {
            return (int) Math.round(speedSlope * range + speedIntercept);
        }
    }

    /** 自动瞄准的单发射器解算结果 */
    public static class ShotSolution {
        /** 所需平转角（rad，0 = 车头，逆时针为正） */
        public final double yaw;
        /** 射程（英寸） */
        public final double range;
        /** 选中的俯仰舵机位置 */
        public final double pitch;
        /** 对应飞轮转速 (tick/s，已按 speedMax 钳制) */
        public final int speed;

        public ShotSolution(double yaw, double range, double pitch, int speed) {
            this.yaw = yaw;
            this.range = range;
            this.pitch = pitch;
            this.speed = speed;
        }
    }

    /**
     * 单个发射器的超参数。大球与小球各持一份实例，故各字段为实例字段。
     */
    public static class Params {
        /**
         * 飞轮速度环的完整参数集（前馈 / PI / 两个滤波器 / 电压归一化 / 量化判据）。
         * 大球传 {@link FlywheelParams#BIG}，小球传 {@link FlywheelParams#SMALL}；
         * <b>所有飞轮控制数值只在 {@link FlywheelParams} 里维护，本类不重复定义</b>。
         */
        public FlywheelParams flywheel = FlywheelParams.copyOf(FlywheelParams.BIG);

        // ---- 机械范围 ----
        /** 俯仰舵机最小位置 */
        public double pitchMin = 0.0;
        /** 俯仰舵机最大位置 */
        public double pitchMax = 1.0;

        // ---- 自动瞄准：预载与离散仰角档位 ----

        /** 该发射器预载最大飞轮转速 (tick/s)；预载解算超过该值时改用该值 + 最高仰角档 */
        public int preSpeedMax = 2000;

        /**
         * 转速目标死区 (tick/s)：新解算值与当前目标之差小于该值时不改变目标。
         *
         * <p>自动瞄准下射程由位姿每帧重算（{@code speed = slope·range + intercept}），
         * 位姿噪声会让目标转速逐帧跳动；而 {@link FlywheelCalculator} 只要目标变化 1 tick/s
         * 就会把稳态计数清零，导致 {@link #isReady()} 永远不成立、扳机永不释放。
         * 取一个量化台阶（约 20 tick/s，对应射程约 0.6 in）作为死区即可消除该抖动。
         */
        public int speedDeadband = 20;

        /**
         * 仰角档位切换滞回裕量 (英寸)：射程超出当前档位区间但未超过该裕量时保持当前档位。
         *
         * <p>没有滞回时，射程在档位边界附近抖动会让俯仰目标逐帧跳变，不断重置
         * {@link #pitchSettleTime} 的计时起点，使 {@link #isReady()} 永远不成立、扳机无法释放。
         */
        public double pitchSwitchMargin = 5.0;

        /**
         * 离散仰角档位表，<b>必须按 {@link PitchModel#pitchPos} 升序排列</b>。
         * 每档给出适用射程区间与「射程 → 转速」线性模型；空表时自动瞄准退化为
         * {@code pitchMax} + 转速 0。todo: 实机标定后填入。
         */
        public PitchModel[] pitchModels = new PitchModel[0];

        // ---- 扳机 ----
        /** 扳机释放（发射）时的舵机位置 */
        public double triggerOnPos = 1.0;
        /** 扳机关闭时的舵机位置 */
        public double triggerOffPos = 0.0;

        // ---- 就绪判据 ----
        /** 俯仰舵机动作时间 (秒)：目标变更后经过该时间即视为到位（转速判据在 {@link FlywheelCalculator#isSettled()}） */
        public double pitchSettleTime = 0.25;

        /** 供 Forge / Dashboard 反射使用的空构造 */
        public Params() {
        }

        /**
         * @param flywheel 该发射器的飞轮参数集
         */
        public Params(FlywheelParams flywheel) {
            this.flywheel = flywheel;
        }
    }

    /** 大球发射器超参数：飞轮参数指向 {@link FlywheelParams#BIG} */
    public static Params BIG_PARAMS = new Params(FlywheelParams.BIG);

    /** 小球发射器超参数：飞轮参数指向 {@link FlywheelParams#SMALL} */
    public static Params SMALL_PARAMS = new Params(FlywheelParams.SMALL);

    static {
        // todo: 实机标定。以下仅为占位档位，用于验证解算流程与结构，数值必须实机重新测定。
        //       档位按 pitchPos 升序；每档给出射程区间与该区间内「射程 → 转速」线性模型。
        BIG_PARAMS.pitchModels = new PitchModel[]{
                new PitchModel(0.30, 30.0, 70.0, 30.0, 900.0),
                new PitchModel(0.50, 60.0, 100.0, 34.0, 1000.0),
                new PitchModel(0.70, 90.0, 150.0, 38.0, 1100.0)
        };
        SMALL_PARAMS.pitchModels = new PitchModel[]{
                new PitchModel(0.30, 30.0, 70.0, 30.0, 900.0),
                new PitchModel(0.50, 60.0, 100.0, 34.0, 1000.0),
                new PitchModel(0.70, 90.0, 150.0, 38.0, 1100.0)
        };
    }

    // ==================== 是否展开诊断遥测 ====================

    /** 是否在 6 行常规遥测之后展开飞轮控制量分解（整定/复盘时勾开） */
    public static boolean telemetryDetailed = false;

    // ==================== 运行时状态 ====================

    public final DcMotorEx motor;
    public final Servo pitchServo;
    public final Servo triggerServo;

    private final Params params;
    private final Telemetry telemetry;
    /** 飞轮电机设备名，同时用作遥测条目的前缀（大球/小球两个实例的 caption 必须可区分） */
    private final String name;
    /** 母线电压传感器（取全部正读数中的最小值） */
    private final Iterable<VoltageSensor> voltageSensors;
    /** 飞轮速度环计算器（与大球/小球各一份，避免状态串台） */
    private final FlywheelCalculator calculator = new FlywheelCalculator();

    /** 最近一次下发的目标转速 (tick/s，钳制后) */
    private int targetSpeed = 0;
    /** 最近一次下发的俯仰舵机位置（钳制后） */
    private double targetPitch = 0;
    /** 最近一次下发的扳机状态：true = 释放（发射） */
    private boolean triggerReleased = false;

    /** 最近一次自动瞄准选中的仰角档位；用于 {@link #selectPitchModel(double)} 的切换滞回 */
    private PitchModel selectedModel;
    /** 上一次的俯仰目标，用于检测目标变更 */
    private double lastPitchCommand = Double.NaN;
    /** 俯仰目标最近一次变更的时刻 (秒) */
    private double pitchCommandTime = 0;

    /** 最近一次下发的占空比（供遥测） */
    private double power = 0;

    /** 最近一次 update() 采到的飞轮转速 (tick/s)，供读取与遥测复用 */
    private double cachedVelocity = 0;
    /** 最近一次 update() 采到的电机电流 (A)，供读取与遥测复用 */
    private double cachedCurrent = 0;
    /** 最近一次 update() 采到的母线电压 (V)，无有效读数时为 NaN */
    private double cachedBusVoltage = Double.NaN;

    /**
     * @param hardwareMap      硬件映射
     * @param telemetry        遥测
     * @param motorName        飞轮电机设备名
     * @param pitchServoName   俯仰舵机设备名
     * @param triggerServoName 扳机舵机设备名
     * @param params           该发射器的超参数（大球传 {@link #BIG_PARAMS}，小球传 {@link #SMALL_PARAMS}）
     */
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, String motorName,
                   String pitchServoName, String triggerServoName, Params params) {
        this.telemetry = telemetry;
        this.params = (params != null) ? params : BIG_PARAMS;
        this.name = motorName;
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.pitchServo = hardwareMap.get(Servo.class, pitchServoName);
        this.triggerServo = hardwareMap.get(Servo.class, triggerServoName);
        this.voltageSensors = hardwareMap.voltageSensor;

        FlywheelParams fw = this.params.flywheel;
        motor.setDirection((fw != null && fw.motorReversed)
                ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // 软件速度闭环：关闭固件速度环，让 setPower 直接控制功率
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 飞轮惯性大，停转后让其自然滑行
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (fw != null) {
            fw.applyTo(calculator);
        }
        calculator.reset();

        pitchCommandTime = now();
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    /**
     * 每帧调用一次：把目标钳制到可用范围后下发舵机，并对飞轮执行速度闭环。
     *
     * @param speed   目标飞轮转速 (tick/s)，钳制到 [0, {@link FlywheelParams#speedMax}]；
     *                与当前目标之差小于 {@link Params#speedDeadband} 时保持原目标不变
     * @param pitch   目标俯仰舵机位置，钳制到 [{@link Params#pitchMin}, {@link Params#pitchMax}]
     * @param release 扳机是否释放（true = 释放/发射）
     * @return 飞轮转速与俯仰机构是否均已到位（即当前是否具备发射条件）
     */
    public boolean update(int speed, double pitch, boolean release) {
        FlywheelParams fw = (params.flywheel != null) ? params.flywheel : FlywheelParams.BIG;
        double currentTime = now();

        // 钳制到可用范围
        int requestedSpeed = Math.max(0, Math.min(fw.speedMax, speed));
        // 死区：小改动不刷新目标，避免稳态判据被位姿噪声反复清零
        if (requestedSpeed == 0 || targetSpeed == 0
                || Math.abs(requestedSpeed - targetSpeed) >= params.speedDeadband) {
            targetSpeed = requestedSpeed;
        }
        targetPitch = Math.max(params.pitchMin, Math.min(params.pitchMax, pitch));

        // 舵机下发
        pitchServo.setPosition(targetPitch);
        setTrigger(release);

        // 俯仰目标变更时重新计时，用于近似判断舵机是否到位
        if (Double.isNaN(lastPitchCommand) || Math.abs(targetPitch - lastPitchCommand) > 1e-6) {
            lastPitchCommand = targetPitch;
            pitchCommandTime = currentTime;
        }

        // 飞轮速度闭环：参数每帧同步，Dashboard 改动即时生效
        fw.applyTo(calculator);
        // 本帧唯一的硬件采样点：转速 / 电流 / 母线电压全部缓存，getter 与遥测只读缓存
        cachedVelocity = motor.getVelocity();
        cachedCurrent = motor.getCurrent(CurrentUnit.AMPS);
        cachedBusVoltage = scanBusVoltage();
        power = calculator.calculate(targetSpeed, cachedVelocity, cachedBusVoltage);
        motor.setPower(power);

        return isReady();
    }

    /**
     * 自动瞄准解算：由车辆位姿、球门位姿与发射器相对位置，解出所需平转角、射程，
     * 并按离散仰角档位表选择俯仰舵机位置与飞轮转速。
     *
     * <p>计算步骤：
     * <ol>
     *   <li>发射器地面投影 {@code p_s = p + R(θ)·relPos}；</li>
     *   <li>方位角 {@code β = atan2(y_g - y_s, x_g - x_s)}，所需平转角 {@code yaw = wrap(β - θ)}；</li>
     *   <li>射程 {@code d = ||p_g - p_s||}；</li>
     *   <li>在档位表中取覆盖 {@code d} 且转速最小的一档（无覆盖时取区间最近的一档）；</li>
     *   <li>转速按 {@code speedMax} 钳制。</li>
     * </ol>
     *
     * @param robotPose 车辆位姿（场坐标系，x 车头方向基准）
     * @param goalPose  球门位置（仅 x/y 有效）
     * @param relPos    发射器旋转中心地面投影在车体相对坐标系中的坐标（x 车头、y 车体左侧）
     * @return 该发射器的解算结果
     */
    public ShotSolution solve(Pose2d robotPose, Pose2d goalPose, Vector2D relPos) {
        double theta = robotPose.heading.toDouble();
        Vector2D rel = (relPos != null) ? relPos : Vector2D.ZERO;
        Vector2D rotated = Vector2D.rotate(rel, theta);
        double shooterX = robotPose.position.x + rotated.getX();
        double shooterY = robotPose.position.y + rotated.getY();

        double dx = goalPose.position.x - shooterX;
        double dy = goalPose.position.y - shooterY;
        double range = Math.hypot(dx, dy);
        double yaw = Vector2D.normalizeAngle(Math.atan2(dy, dx) - theta);

        PitchModel model = selectPitchModel(range);
        if (model == null) {
            // 档位表为空：退化为最高仰角 + 转速 0，保持机构安全
            return new ShotSolution(yaw, range, params.pitchMax, 0);
        }
        return new ShotSolution(yaw, range, model.pitchPos, clampSpeed(model.speedFor(range)));
    }

    /**
     * 选择仰角档位：优先取覆盖该射程且所需转速最小的一档；
     * 若无档位覆盖，取射程区间距离最近的一档（速度随后由 {@link #clampSpeed(int)} 钳制）。
     *
     * <p><b>切换滞回</b>：一旦选中某档，只要射程仍在该档区间外 {@link Params#pitchSwitchMargin}
     * 以内就保持不动。否则射程在档位边界附近抖动时，俯仰目标与转速目标会逐帧跳变，
     * 不断重置 {@link #pitchSettleTime} 的计时与 {@link FlywheelCalculator} 的稳态计数，
     * 导致 {@link #isReady()} 永远不成立、扳机无法释放。
     */
    private PitchModel selectPitchModel(double range) {
        PitchModel[] models = params.pitchModels;
        if (models == null) {
            selectedModel = null;
            return null;
        }

        // 1) 覆盖该射程且所需转速最小的一档
        PitchModel best = null;
        int bestSpeed = Integer.MAX_VALUE;
        for (PitchModel model : models) {
            if (model == null || !model.covers(range)) {
                continue;
            }
            int speed = model.speedFor(range);
            if (speed < bestSpeed) {
                bestSpeed = speed;
                best = model;
            }
        }

        // 2) 滞回：当前档位即使已被更优档取代，只要射程仍在它区间外的滞回裕量内就保持
        double margin = params.pitchSwitchMargin;
        if (selectedModel != null && selectedModel != best
                && range >= selectedModel.minRange - margin
                && range <= selectedModel.maxRange + margin) {
            best = selectedModel;
        }

        // 3) 无档位覆盖（且已超出滞回裕量）：取射程区间距离最近的一档
        if (best == null) {
            double bestDistance = Double.MAX_VALUE;
            for (PitchModel model : models) {
                if (model == null) {
                    continue;
                }
                double distance = (range < model.minRange) ? (model.minRange - range)
                        : (range > model.maxRange) ? (range - model.maxRange) : 0;
                if (distance < bestDistance) {
                    bestDistance = distance;
                    best = model;
                }
            }
        }
        selectedModel = best;
        return best;
    }

    /** 把转速钳制到 [0, speedMax] */
    private int clampSpeed(int speed) {
        int max = (params.flywheel != null) ? params.flywheel.speedMax : Integer.MAX_VALUE;
        return Math.max(0, Math.min(max, speed));
    }

    /**
     * @return 离散档位中的最低仰角（档位表为空时返回 {@link Params#pitchMin}）
     */
    public double getMinPitch() {
        PitchModel[] models = params.pitchModels;
        if (models != null && models.length > 0 && models[0] != null) {
            return models[0].pitchPos;
        }
        return params.pitchMin;
    }

    /**
     * @return 离散档位中的最高仰角（档位表为空时返回 {@link Params#pitchMax}）
     */
    public double getMaxPitch() {
        PitchModel[] models = params.pitchModels;
        if (models != null && models.length > 0 && models[models.length - 1] != null) {
            return models[models.length - 1].pitchPos;
        }
        return params.pitchMax;
    }

    /**
     * 仅更新扳机状态与舵机位置，不重算速度环。
     * 供自动瞄准在「本帧解算就绪」后单独扣扳机，避免用上一帧状态提前发射。
     *
     * @param release 扳机是否释放（true = 释放/发射）
     */
    public void setTrigger(boolean release) {
        triggerReleased = release;
        triggerServo.setPosition(release ? params.triggerOnPos : params.triggerOffPos);
    }

    /**
     * 停止发射：飞轮目标归零、扳机复位。
     * 俯仰舵机保持当前位置不变（不改动 {@code targetPitch}）。
     */
    public void stop() {
        targetSpeed = 0;
        triggerReleased = false;
        triggerServo.setPosition(params.triggerOffPos);
        motor.setPower(0);
        power = 0;
        calculator.reset(cachedVelocity);
    }

    // ==================== 读取 ====================

    /**
     * @return 飞轮转速与俯仰机构是否均已到位。飞轮需进入稳态带（{@link FlywheelCalculator#isSettled()}）
     *         且目标非零，俯仰舵机以动作时间近似判断。
     */
    public boolean isReady() {
        boolean speedReady = targetSpeed > 0 && calculator.isSettled();
        boolean pitchReady = now() - pitchCommandTime >= params.pitchSettleTime;
        return speedReady && pitchReady;
    }

    /** @return 该发射器的超参数（整定程序可改 {@code params.flywheel} 的字段） */
    public Params getParams() {
        return params;
    }

    /** @return 飞轮速度环计算器 */
    public FlywheelCalculator getCalculator() {
        return calculator;
    }

    /** @return 飞轮电机设备名 */
    public String getName() {
        return name;
    }

    /** @return 最近一次 update() 采到的飞轮转速 (tick/s) */
    public double getVelocity() {
        return cachedVelocity;
    }

    /** @return 最近一次下发的目标转速 (tick/s) */
    public int getTargetSpeed() {
        return targetSpeed;
    }

    /** @return 最近一次下发的俯仰舵机位置 */
    public double getTargetPitch() {
        return targetPitch;
    }

    /** @return 最近一次下发的扳机状态 */
    public boolean isTriggerReleased() {
        return triggerReleased;
    }

    /** @return 最近一次下发的占空比 */
    public double getPower() {
        return power;
    }

    /** @return 最近一次 update() 采到的电机电流 (A) */
    public double getCurrent() {
        return cachedCurrent;
    }

    /**
     * @return 最近一次 update() 采到的母线电压 (V)；无有效读数时 NaN
     */
    public double getBusVoltage() {
        return cachedBusVoltage;
    }

    /**
     * 遍历全部电压传感器采样母线电压：取所有正读数中的最小值。
     * 只在 {@link #update()} 内调用，其他接口只读 {@link #cachedBusVoltage}。
     */
    private double scanBusVoltage() {
        double result = -1;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = (result < 0) ? voltage : Math.min(result, voltage);
            }
        }
        return result < 0 ? Double.NaN : result;
    }

    /**
     * 输出遥测（调用方需自行 telemetry.update()）。
     * caption 以设备名为前缀，避免大球/小球两个实例的条目互相覆盖
     * （FTC Dashboard 以 caption 为键，重名时后者覆盖前者）。
     */
    public void setTelemetry() {
        telemetry.addData(name + " speed target (tick/s)", targetSpeed);
        telemetry.addData(name + " speed current (tick/s)", "%.0f", cachedVelocity);
        telemetry.addData(name + " power", "%.4f", power);
        telemetry.addData(name + " pitch", "%.3f", targetPitch);
        telemetry.addData(name + " trigger released", triggerReleased);
        telemetry.addData(name + " ready", isReady());

        if (telemetryDetailed) {
            telemetry.addLine("-- " + name + " research --");
            telemetry.addData(name + " error raw (tick/s)", "%.0f", calculator.getErrorRaw());
            telemetry.addData(name + " error filt (tick/s)", "%.0f", calculator.getErrorFiltered());
            telemetry.addData(name + " tauV (ms)", "%.1f", calculator.getTauV() * 1000.0);
            telemetry.addData(name + " eBand (tick/s)", "%.0f", calculator.getErrorBand());
            telemetry.addData(name + " settled", calculator.isSettled());
            telemetry.addData(name + " vScale", "%.3f", calculator.getVScale());
            telemetry.addData(name + " vBus filt (V)", "%.2f", calculator.getBusVoltageFiltered());
            telemetry.addData(name + " kS term", "%.4f", calculator.getKSTerm());
            telemetry.addData(name + " FF term", "%.4f", calculator.getFfTerm());
            telemetry.addData(name + " P term", "%.4f", calculator.getPTerm());
            telemetry.addData(name + " I term", "%.4f", calculator.getITerm());
            telemetry.addData(name + " kp*step", "%.4f", calculator.getKpQuantization());
            telemetry.addData(name + " current (A)", "%.2f", getCurrent());
        }
    }
}
