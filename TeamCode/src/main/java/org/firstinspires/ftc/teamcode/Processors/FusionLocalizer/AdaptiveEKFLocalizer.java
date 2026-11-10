package org.firstinspires.ftc.teamcode.Processors.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.utility.filter.EKF.EKF;

/**
 * 融合定位器 —— 里程计 + Limelight MegaTag1(视觉) + EKF 融合 + Hub IMU 自适应 Q。
 *
 * <p>支持两种里程计模式：
 * <ul>
 *   <li><b>D2</b> (默认): {@link PinpointLocalizer}，标准 2D 里程计，无斜坡补偿</li>
 *   <li><b>D3</b>: {@link PinpointD3Localizer}，3D 斜坡补偿里程计</li>
 * </ul>
 *
 * <p>每帧调用 {@link #update()} 即可完成：
 * <ol>
 *   <li>里程计速度预测 ({@link EKF#predict})</li>
 *   <li>Limelight 视觉更新 (条件触发, {@link EKF#update})</li>
 *   <li>自适应调整 Q (IMU 角速度/角加速度) 和 R (视觉 stdDev)</li>
 * </ol>
 *
 * <p><b>自适应 Q 策略</b> (D2 与 D3 不同)：
 * <ul>
 *   <li><b>D2</b>: roll/pitch 角加速度 → 冲击检测；yaw 角加速度 → 旋转冲击</li>
 *   <li><b>D3</b>: roll/pitch 角速度 → 坡度变化检测；yaw 角加速度 → 旋转冲击</li>
 *   <li>不确定性方向与旋转轴垂直: pitch 绕 body Y → X 方向腾空, roll 绕 body X → Y 方向腾空</li>
 * </ul>
 *
 * <p><b>R 自适应</b>: MT1 各方向 stdDev → 各方向独立 R 矩阵
 *
 * <p><b>视觉门控</b>: 仅当 {@link MT1Localizer#isValid()} 与 {@link MT1Localizer#isHiveEstimated()}
 * 同时为 true 时才进入 R 自适应与马氏距离门控。后者为 false 说明该帧 HIVE 倾角无解或被拒
 * (无标签 / 无解 / 取根超 CellUpAngle / 姿态交叉校验不通过 / 枢轴高度未标定),
 * 此时 {@code MT1Localizer#getPose()} 回退为未经修正的受污染位姿, 必须整帧丢弃。
 */

//todo：向 RobotPosition 透传 MT1Localizer 的 hive 倾角/状态观测（拒绝无效视觉更新已实现）
@Config
public class AdaptiveEKFLocalizer implements Localizer {

    private final EKF ekf;
    /** 里程计定位器 (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    private final Localizer odom;
    private final MT1Localizer mt1;
    private final IMU hubImu;
    private final boolean useD3;

    // ---- 时间基准 ----
    private double lastTimestamp = 0;

    /** 最近一次里程计速度缓存 */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    /**
     * 本帧采到的 IMU 姿态角：下传给 D3 里程计（斜坡补偿 pith/roll），
     * 上供 RobotPosition 的 getYaw / getYawPitchRollAngles 读取，保证每帧只读一次 IMU。
     */
    private YawPitchRollAngles imuAngles =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
    /** 本帧采到的 IMU 角速度 (rad/s)，供 Q 自适应使用 */
    private AngularVelocity imuAngularVelocity;

    // ---- Q 自适应: IMU 检测 ----
    /** 上一帧 roll 角速度 (rad/s) — 绕 body X 轴 (左右) */
    private double lastRollRate = 0;
    /** 上一帧 pitch 角速度 (rad/s) — 绕 body Y 轴 (前后) */
    private double lastPitchRate = 0;
    /** 上一帧 yaw 角速度 (rad/s) — 绕 body Z 轴 (垂直) */
    private double lastYawRate = 0;
    /** 是否已采样首帧角速度 (防止首帧 (rate - 0) / dt 误触发 Q 提升) */
    private boolean ratesInitialized = false;

    /** Q 基值 (in²/s) */
    public static double qBase = 0.002;
    /** Q 倍增因子 (运行时状态, 每实例独立; 不可为 static, 否则多实例互相污染) */
    private double qBoostX = 1.0;
    private double qBoostY = 1.0;
    private double qBoostTheta = 1.0;

    // ---- D2: 角加速度阈值 (冲击检测) ----
    public static double ANGULAR_ACCEL_THRESHOLD = 5.0;  // rad/s² (pitch/roll 角加速度阈值)
    public static double JERK_THRESHOLD = 4.0;           // rad/s² (yaw 角速度 jerk 阈值)

    // ---- D3: 角速度阈值 (坡度变化检测) ----
    public static double ANGULAR_VEL_THRESHOLD = 1.0;    // rad/s (pitch/roll 角速度阈值，坡度变化)
    public static double VEL_BOOST_MAX = 10.0;            // 角速度最大 Q 倍增因子 (坡度变化，较高)
    public static double ACCEL_BOOST_MAX = 4.0;         // 角加速度最大 Q 倍增因子 (冲击，较低)

    public static double Q_BOOST_MAX = 10.0;
    public static double Q_DECAY = 0.85;

    // ---- R 自适应: MT1 stdDev ----
    /** 单位转换: 1 m = 39.3701 in */
    public static double M_TO_INCH = 39.37007874;

    /** stdDev 阈值 (英寸) — 对应原 0.05m / 0.15m */
    public static double STD_LOW_INCH = 2.0;
    public static double STD_HIGH_INCH = 6.0;

    /** stdDev 阈值 (弧度) — 角度分量专用，对应 ≈2° / ≈10° */
    public static double STD_LOW_ANGLE = 0.035;   // rad (≈2°)
    public static double STD_HIGH_ANGLE = 0.175;  // rad (≈10°)

    public static double R_MAX_SCALE = 20.0;

    // ---- 视觉距离 / 标签缩放与门控 (9/2 改进) ----
    /** 参考距离 (米)，超过该距离视觉位置 R 随距离二次放大 */
    public static double DIST_REF_M = 1.0;
    /** 参考标签数，标签数低于该值视觉位置 R 放大 */
    public static double TAG_REF = 2.0;
    /** 标签数过少时 R 的最大放大倍数 */
    public static double TAG_SCALE_MAX = 4.0;
    /** 马氏距离门控阈值 (无量纲) */
    public static double GATE_THRESHOLD = 4.0;

    /** R 基值缓存 (调试展示, 每实例独立) */
    private double rBase = 0.01;

    // ==================== 构造 ====================

    /**
     * D2 模式构造 (标准 2D 里程计), 默认红方 (Limelight pipeline 0)。
     *
     * @param hardwareMap    硬件映射
     * @param limelight      已启动的 Limelight3A 实例
     * @param imuDeviceName  IMU 设备名 (如 "imu")
     * @param initialPose    初始位姿 (x, y, heading)
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, false, TeamColor.RED);
    }

    /**
     * D2 模式简化构造: 使用默认 IMU 名称 "imu", 初始位姿 (0,0,0), 默认红方。
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, "imu", new Pose2d(0, 0, 0), false, TeamColor.RED);
    }

    /**
     * D3 模式构造 (3D 斜坡补偿里程计), 默认红方 (Limelight pipeline 0)。
     *
     * @param hardwareMap    硬件映射
     * @param limelight      已启动的 Limelight3A 实例
     * @param imuDeviceName  IMU 设备名 (如 "imu")，供里程计和 adaptQ 共用
     * @param initialPose    初始位姿 (x, y, heading)
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, useD3, TeamColor.RED);
    }

    /**
     * 完整构造。
     *
     * @param hardwareMap    硬件映射
     * @param limelight      已启动的 Limelight3A 实例
     * @param imuDeviceName  IMU 设备名 (如 "imu")，供里程计和 adaptQ 共用
     * @param initialPose    初始位姿 (x, y, heading)
     * @param useD3          true 使用 D3 斜坡补偿里程计, false 使用标准 2D 里程计
     * @param teamColor      队伍颜色: 红方加载 Limelight pipeline 0, 蓝方加载 pipeline 1
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3,
                           TeamColor teamColor) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, useD3, teamColor, false);
    }

    /**
     * 完整构造，可开启视觉时间戳回滚重放。
     *
     * @param hardwareMap    硬件映射
     * @param limelight      已启动的 Limelight3A 实例
     * @param imuDeviceName  IMU 设备名 (如 "imu")，供里程计和 adaptQ 共用
     * @param initialPose    初始位姿 (x, y, heading)
     * @param useD3          true 使用 D3 斜坡补偿里程计, false 使用标准 2D 里程计
     * @param teamColor      队伍颜色: 红方加载 Limelight pipeline 0, 蓝方加载 pipeline 1
     * @param allowReplay    是否启用视觉时间戳回滚重放 (默认 false)
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3,
                           TeamColor teamColor, boolean allowReplay) {
        this.ekf = new EKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble(), allowReplay);
        if (useD3) {
            this.odom = new PinpointD3Localizer(hardwareMap, 0.001999, imuDeviceName, initialPose);
        } else {
            this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        }
        this.mt1 = new MT1Localizer(limelight, teamColor);
        this.hubImu = hardwareMap.get(IMU.class, imuDeviceName);
        this.useD3 = useD3;
        this.lastTimestamp = getNow();
    }

    // ==================== 核心循环 ====================

    /**
     * 每帧调用一次，完成：
     * <ol>
     *   <li>里程计更新 → 获取速度</li>
     *   <li>IMU 角速度/角加速度检测 → 自适应 Q (D2/D3 策略不同)</li>
     *   <li>EKF 预测</li>
     *   <li>Limelight 更新 → 自适应 R (3x3 矩阵) + EKF 更新</li>
     * </ol>
     *
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        // ---- 1. IMU 采样：本帧唯一一次。姿态角下传里程计、上供 RobotPosition，角速度供 Q 自适应 ----
        imuAngles = hubImu.getRobotYawPitchRollAngles();
        imuAngularVelocity = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);

        // ---- 2. 里程计速度 (D3 复用本帧 IMU 姿态，不再单独读 IMU) ----
        lastVel = useD3
                ? ((PinpointD3Localizer) odom).update(imuAngles)
                : odom.update();

        // ---- 3. IMU 角速度 → 自适应 Q 矩阵 (D2/D3 策略不同) ----
        ekf.setQ(adaptQ(dt));

        // ---- 4. EKF 预测 ----
        ekf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 5. MT1 视觉 → 自适应 R 矩阵 + 门控 + EKF 更新 ----
        mt1.update();
        // isHiveEstimated() 为 false 表示该帧 HIVE 解算失败或被拒 (无标签 / 无解 / 取根超 CellUpAngle /
        // 姿态交叉校验不通过 / 枢轴高度未标定), 此时 mt1.getPose() 是未经修正的受污染位姿, 直接丢弃
        if (mt1.isValid() && mt1.isHiveEstimated()) {
            ekf.setR(adaptR());
            Pose2d visionPose = mt1.getPose();              // (英寸, 英寸, 弧度)
            if (ekf.gateVision(
                    visionPose.position.x,
                    visionPose.position.y,
                    visionPose.heading.toDouble(),
                    GATE_THRESHOLD
            )) {
                ekf.update(
                        visionPose.position.x,                  // 英寸
                        visionPose.position.y,                  // 英寸
                        visionPose.heading.toDouble(),          // 弧度
                        // 回滚重放需与 predict 快照同基准 (System.nanoTime()); 未启用时沿用 Limelight 硬件时间戳
                        ekf.isReplayEnabled() ? mt1.getTimestampNanoBase() : mt1.getTimestamp()
                );
            }
        }

        return lastVel;
    }

    // ==================== 自适应 Q (SimpleMatrix 输出) ====================

    /**
     * 基于 IMU 角速度/角加速度构建 3x3 对角 Q 矩阵。
     *
     * <p><b>D2 模式</b> (角加速度)：
     * <ul>
     *   <li><b>x, y</b>: roll/pitch 角加速度 → 旋转到场坐标系 → 交叉映射 (pitch→X, roll→Y)</li>
     *   <li><b>θ</b>: yaw 角加速度 → 检测旋转碰撞</li>
     * </ul>
     *
     * <p><b>D3 模式</b> (角速度)：
     * <ul>
     *   <li><b>x, y</b>: roll/pitch 角速度 → 旋转到场坐标系 → 交叉映射 (pitch→X, roll→Y)</li>
     *   <li><b>θ</b>: yaw 角加速度 → 检测旋转冲击</li>
     * </ul>
     *
     * @param dt 帧间隔 (秒)
     * @return 3x3 对角过程噪声协方差矩阵 Q
     */
    private SimpleMatrix adaptQ(double dt) {
        double safeDt = Math.max(dt, 1e-6);

        // 使用补偿后航向 (含 setPose 偏移) 而非 pinpoint 原始航向:
        // 旋转 pitch/roll 角速度到场地坐标系需要机器人在场地中的真实朝向,
        // 补偿后航向 = txWorldPinpoint.heading + pinpointTheta, 代表了 EKF 跟踪的位姿朝向
        double theta = getPose().heading.toDouble();
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);

        if (hubImu != null) {
            AngularVelocity angVel = imuAngularVelocity;  // 由 update() 本帧采样，避免在 adaptQ 内再读硬件
            if (angVel != null) {
                // IMU 体坐标系 (X=右, Y=前) → RR 体坐标系 (X=前, Y=左):
                //   xRotationRate (绕 Robot X/右) → pitch (绕 RR Y/左, 前后倾斜)
                //   yRotationRate (绕 Robot Y/前) → roll  (绕 RR X/前, 左右倾斜)
                double pitchRate = angVel.xRotationRate;
                double rollRate  = angVel.yRotationRate;
                double yawRate   = angVel.zRotationRate;  // 仅用于角速度变化率检测, 非绝对值

                if (!ratesInitialized) {
                    // 首帧仅采样上次角速度, 避免 (rate - 0) / dt 产生虚假的大角加速度并误触发 Q 提升
                    lastPitchRate = pitchRate;
                    lastRollRate  = rollRate;
                    lastYawRate   = yawRate;
                    ratesInitialized = true;
                } else {
                // ---- pitch/roll 角加速度 (冲击) ----
                double pitchAccel = (pitchRate - lastPitchRate) / safeDt;
                double rollAccel  = (rollRate  - lastRollRate)  / safeDt;

                // 体坐标系 → 场坐标系旋转 (Rz(θ))
                // angular velocity vector in body: [rollRate, pitchRate]^T
                double fieldX = rollAccel * cosT - pitchAccel * sinT;
                double fieldY = rollAccel * sinT + pitchAccel * cosT;

                if (useD3) {
                    // ========== D3: 角速度 (坡度变化检测) ==========
                    // 旋转到场地坐标系，然后交叉映射:
                    // pitch 绕 body Y → X 方向轮子腾空 → X 不确定度 (取 fieldY 分量)
                    // roll  绕 body X → Y 方向轮子腾空 → Y 不确定度 (取 fieldX 分量)
                    double fieldVelX = rollRate * cosT - pitchRate * sinT;
                    double fieldVelY = rollRate * sinT + pitchRate * cosT;

                    qBoostX = updateBoost(qBoostX, Math.abs(fieldVelY), ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX);
                    qBoostY = updateBoost(qBoostY, Math.abs(fieldVelX), ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX);

                    // yaw: 角加速度 (旋转冲击)
                    double yawAccel = Math.abs((yawRate - lastYawRate) / safeDt);
                    qBoostTheta = updateBoost(qBoostTheta, yawAccel, JERK_THRESHOLD);

                } else {
                    // ========== D2: 角加速度 (冲击检测) ==========
                    // 交叉映射: pitch 绕 body Y → X 方向轮子腾空 → X 不确定度 (取 fieldY 分量)
                    //           roll  绕 body X → Y 方向轮子腾空 → Y 不确定度 (取 fieldX 分量)

                    qBoostX = updateBoost(qBoostX, Math.abs(fieldY), ANGULAR_ACCEL_THRESHOLD);
                    qBoostY = updateBoost(qBoostY, Math.abs(fieldX), ANGULAR_ACCEL_THRESHOLD);

                    // yaw 角加速度
                    double yawAccel = Math.abs((yawRate - lastYawRate) / safeDt);
                    qBoostTheta = updateBoost(qBoostTheta, yawAccel, JERK_THRESHOLD);
                }

                lastPitchRate = pitchRate;
                lastRollRate  = rollRate;
                lastYawRate   = yawRate;
                }
            }
        }

        // 构建 3x3 对角 Q 矩阵
        SimpleMatrix Q = new SimpleMatrix(3, 3);
        Q.set(0, 0, qBase * qBoostX);
        Q.set(1, 1, qBase * qBoostY);
        Q.set(2, 2, qBase * qBoostTheta);
        return Q;
    }

    // ==================== 自适应 R (SimpleMatrix 输出) ====================

    /**
     * 基于 MT1 各方向 stdDev 构建 3x3 对角 R 矩阵。
     *
     * <p>每个方向独立映射 (std → R):
     * <pre>
     *   std &lt; STD_LOW (0.05)   →  R = 0.01
     *   std &gt; STD_HIGH (0.15)  →  R = 0.01 × R_MAX_SCALE
     *   中间 → 线性插值
     * </pre>
     *
     * @return 3x3 对角观测噪声协方差矩阵 R
     */
    private SimpleMatrix adaptR() {
        double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (米, 度)

        // 距离与标签数缩放: 远离 tag / 标签过少时视觉位置误差大, 放大位置 R
        double distF = computeDistFactor();
        double tagF = computeTagFactor();

        // 位置: stdDev 从米 → 英寸，与 EKF 状态单位一致
        double rX = mapStdToR(stdDevs[0] * M_TO_INCH) * distF * tagF;
        double rY = mapStdToR(stdDevs[1] * M_TO_INCH) * distF * tagF;
        // 角度: yaw 从度 → 弧度，使用角度专用阈值
        double rTheta = mapStdToRAngle(Math.toRadians(stdDevs[5]));

        // 缓存 rBase 供调试展示
        rBase = rTheta;

        SimpleMatrix R = new SimpleMatrix(3, 3);
        R.set(0, 0, rX);
        R.set(1, 1, rY);
        R.set(2, 2, rTheta);
        return R;
    }

    // ==================== 工具函数 ====================

    /**
     * 单个方向的 Q 倍增因子更新 (带自定义 maxBoost)。
     *
     * @param current   当前 boost 值
     * @param magnitude 信号幅值
     * @param threshold 触发阈值
     * @param maxBoost  最大 boost 上限
     * @return 更新后的 boost 值
     */
    private double updateBoost(double current, double magnitude, double threshold, double maxBoost) {
        if (magnitude > threshold) {
            return Math.min(maxBoost, current * (1.0 + magnitude / threshold));
        } else {
            return Math.max(1.0, current * Q_DECAY);
        }
    }

    /**
     * 单个方向的 Q 倍增因子更新 (D2 兼容，使用默认 Q_BOOST_MAX)。
     */
    private double updateBoost(double current, double magnitude, double threshold) {
        return updateBoost(current, magnitude, threshold, Q_BOOST_MAX);
    }

    /**
     * 单方向 std → R 映射 (三段式: 信任 / 线性插值 / 怀疑)。
     * 使用位置阈值 (STD_LOW_INCH / STD_HIGH_INCH)。
     *
     * @param std 标准偏差 (英寸)
     */
    private double mapStdToR(double std) {
        return mapStdToR(std, STD_LOW_INCH, STD_HIGH_INCH);
    }

    /**
     * 角度方向 std → R 映射，使用角度专用阈值。
     *
     * @param std 角度标准偏差 (弧度)
     */
    private double mapStdToRAngle(double std) {
        return mapStdToR(std, STD_LOW_ANGLE, STD_HIGH_ANGLE);
    }

    /**
     * 通用 std → R 映射 (三段式: 信任 / 线性插值 / 怀疑)。
     */
    private double mapStdToR(double std, double low, double high) {
        if (std <= low) {
            return 0.01;
        }
        // 线性增长, 超过 high 时继续线性放大, 不再截断上界 (9/2 改进)
        double t = (std - low) / (high - low);
        return 0.01 * (1.0 + t * (R_MAX_SCALE - 1.0));
    }

    /**
     * 视觉位置观测噪声的距离缩放因子（远离 tag 时二次放大）。
     */
    private double computeDistFactor() {
        double dist = mt1.getAvgDist();  // 米
        if (Double.isNaN(dist) || Double.isInfinite(dist) || dist <= DIST_REF_M) {
            return 1.0;
        }
        double ratio = dist / DIST_REF_M;
        return ratio * ratio;
    }

    /**
     * 视觉位置观测噪声的标签数缩放因子（标签过少时放大）。
     */
    private double computeTagFactor() {
        int tags = mt1.getTagCount();
        if (tags >= TAG_REF) {
            return 1.0;
        }
        if (tags <= 0) {
            return TAG_SCALE_MAX;
        }
        double ratio = TAG_REF / tags;
        return Math.min(TAG_SCALE_MAX, ratio);
    }

    // ==================== Localizer 接口 ====================

    /** 设置定位器位姿。 */
    @Override
    public void setPose(Pose2d pose) {
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
        lastTimestamp = getNow();
        qBoostX = 1.0;
        qBoostY = 1.0;
        qBoostTheta = 1.0;
        lastPitchRate = 0;
        lastRollRate = 0;
        lastYawRate = 0;
        ratesInitialized = false;
    }

    // ==================== 输出 ====================

    /** @return 融合后的位姿 {x, y, heading} (英寸, 英寸, 弧度) */
    @Override
    public Pose2d getPose() {
        double[] pose = ekf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    /** @return 原始 EKF 实例 */
    public EKF getEKF() { return ekf; }

    /** @return MT1 视觉定位器 */
    public MT1Localizer getMT1() { return mt1; }

    /** @return 里程计定位器 (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    public Localizer getOdom() { return odom; }

    /**
     * @return 本帧 {@link #update()} 采到的 IMU 姿态角 (yaw/pitch/roll)。
     *         融合链路中 IMU 每帧只在此处读一次，RobotPosition 通过本接口取用，
     *         不再自行读取硬件。
     */
    public YawPitchRollAngles getImuAngles() { return imuAngles; }

    /** @return 是否为 D3 模式 */
    public boolean isD3() { return useD3; }

    /** @return x 方向 Q 倍增因子 */
    public double getQBoostX() { return qBoostX; }
    /** @return y 方向 Q 倍增因子 */
    public double getQBoostY() { return qBoostY; }
    /** @return θ 方向 Q 倍增因子 */
    public double getQBoostTheta() { return qBoostTheta; }

    // ==================== 重置 ====================

    /** 重置定位到指定位姿。 */
    public void reset(Pose2d pose) {
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
        lastTimestamp = getNow();
        qBoostX = 1.0;
        qBoostY = 1.0;
        qBoostTheta = 1.0;
        lastPitchRate = 0;
        lastRollRate = 0;
        lastYawRate = 0;
        ratesInitialized = false;
    }

    // ==================== 内部工具 ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}