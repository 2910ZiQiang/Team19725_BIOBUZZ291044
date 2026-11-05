package org.firstinspires.ftc.teamcode.Processors.RobotPosition;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BallCounter;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.utility.filter.EMA;
//负责除视觉以外的所有状态估计
@Config
public class RobotPosition {
    static MecanumDrive drive;
    HardwareMap hardwareMap;

    /** 融合定位器：Pinpoint 速度 + Limelight 视觉 + 自适应 EKF（权威位姿来源） */
    private AdaptiveEKFLocalizer fusionLocalizer;

    /** 球道计数器：BreakBeam → BallCounter，本类持有并向 OpMode / Sweeper / Turret 透传 */
    /** 球道光电门在 Robot Configuration 中的模拟口设备名 */
    private static final String BALL_BEAM_DEVICE = "breakBeam";

    private BallCounter ballCounter;

    /**
     * HIVE 状态跟踪值：MT1 有观测时更新为观测值，无观测 / 解算被拒时保持上一状态。
     * 初值取 MIDDLE（HIVE 水平），跟踪值不会为 {@link MT1Localizer.HiveState#UNKNOWN}。
     */
    private MT1Localizer.HiveState hiveState = MT1Localizer.HiveState.MIDDLE;

    /** IMU，复用 Road Runner 已初始化的实例（设备名 "imu"），仅用于 resetYaw() */
    private IMU imu;

    public Pose2d currentPose;
    public PoseVelocity2d currentVelocity2d;

    private static RobotPosition instance;

    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }

    /**
     * @return 已初始化的实例；未初始化时返回 null。
     *         供可视化等「可选」功能安全访问，避免 {@link #getInstance()} 抛异常。
     */
    public static RobotPosition getInstanceOrNull(){
        return instance;
    }
    private RobotPosition(){
    }

;
    /**
     * 按红方初始化（Limelight pipeline 0）。
     */
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose) {
        return RobotPositioninit(hardwareMap, initpose, TeamColor.RED);
    }

    /**
     * @param hardwareMap 硬件映射
     * @param initpose    初始位姿
     * @param teamColor   队伍颜色: 红方加载 Limelight pipeline 0, 蓝方加载 pipeline 1
     */
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose,
                                                  TeamColor teamColor) {

        instance=new RobotPosition();
        instance.hardwareMap = hardwareMap;

        // ---- 球道计数：光电门 → BallCounter（BreakBeam 是 RobotPosition 的下属）----
        instance.initBallCounter(hardwareMap);

        instance.currentPose = initpose != null ? initpose : new Pose2d(0,0,0);

        // ---- 先构造融合定位器：Pinpoint 速度预测 + Limelight 视觉更新 + 自适应 Q/R ----
        // 队伍颜色决定 Limelight pipeline（红 0 / 蓝 1），由 MT1Localizer 按颜色切换。
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        instance.fusionLocalizer = new AdaptiveEKFLocalizer(
                hardwareMap, limelight, "imu", instance.currentPose, false, teamColor);

        // ---- 把融合定位器注入 RoadRunner，作为 MecanumDrive 唯一的定位器 ----
        // 位姿与速度都直接来自 EKF，不再需要外部 setPose 回写；
        // 定位器每帧仅由 RobotPosition.update() 调用一次 drive.updatePoseEstimate() 推进，
        // 轨迹 / 转向 Action 内部改为复用该帧结果。
        instance.drive = new MecanumDrive(hardwareMap, instance.fusionLocalizer);
        // IMU 复用 Road Runner 已初始化的实例
        instance.imu = instance.drive.lazyImu.get();
        return instance;
    }

    /**
     * 重置机器人位姿。
     * MecanumDrive 的 localizer 就是融合定位器本身，因此一次 setPose
     * 即同时复位 EKF 状态与内部 Pinpoint 里程计。
     *
     * @param pose 目标位姿（真实位姿）
     */
    public void ResetPoseTo(Pose2d pose) {
        drive.localizer.setPose(pose);
        currentPose = pose;
    }

    /**
     * 创建球道计数器。光电门是可选硬件：未配置时 ballCounter = null，
     * 计数接口返回 0/false，避免仅做定位/底盘调试的 OpMode 因缺少光电门而无法初始化。
     */
    private void initBallCounter(HardwareMap hardwareMap) {
        // 光电门是可选硬件：调试用 OpMode 可能没配，get() 会抛异常。
        // 创建失败就把 ballCounter 留成 null，计数接口自然返回 0 / false。
        try {
            ballCounter = new BallCounter(hardwareMap, BALL_BEAM_DEVICE,
                    HypParams.BALL_CAPACITY);
        } catch (RuntimeException e) {
            ballCounter = null;
        }
    }

    // 每帧调用：推进融合定位器并返回当前位姿
    public Pose2d update() {
        // 唯一推进点：每帧只调用一次，避免同一帧的视觉观测被重复计入 EKF
        // （MecanumDrive 的轨迹 / 转向 Action 内部已改为复用本帧结果）
        currentVelocity2d = drive.updatePoseEstimate();

        // 球道计数：每帧采样一次光电门并做上升沿计数（PPT 顺序：EKF → BreakBeam → BallCounter）
        if (ballCounter != null) {
            ballCounter.update();
        }

        // HIVE 状态跟踪：MT1 成功解出倾角观测时更新，否则保持上一状态
        MT1Localizer mt1 = fusionLocalizer.getMT1();
        if (mt1.isHiveEstimated()) {
            hiveState = mt1.getHiveState();
        }

        currentPose = drive.localizer.getPose();
        return currentPose;
    }


    public Pose2d getPose2d(){        return currentPose;    }

    public double getX(){     return currentPose.position.x;    }
    public double getY(){   return currentPose.position.y;    }
    public double getTheta(){ return currentPose.heading.toDouble();    }
    /** @return 场坐标系 X 轴线速度 (英寸/秒)。输入为体坐标系速度，按 R(θ) 旋转到场坐标系。 */
    public double getVx(){
        double vxBody = currentVelocity2d.linearVel.x;
        double vyBody = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return vxBody * Math.cos(theta) - vyBody * Math.sin(theta);
    }
    /** @return 场坐标系 Y 轴线速度 (英寸/秒)。输入为体坐标系速度，按 R(θ) 旋转到场坐标系。 */
    public double getVy(){
        double vxBody = currentVelocity2d.linearVel.x;
        double vyBody = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return vxBody * Math.sin(theta) + vyBody * Math.cos(theta);
    }
    public MecanumDrive getDrive(){return drive;}
    /** @return 自适应 EKF 融合定位器（可读取 MT1 HIVE 倾角/状态观测与 Q、R 调试量） */
    public AdaptiveEKFLocalizer getFusionLocalizer(){return fusionLocalizer;}

    /**
     * @return 当前跟踪的 HIVE 状态 (MIDDLE / AUDIENCE_UP / AUDIENCE_DOWN)。
     *         在 {@link #update()} 中于 MT1 成功解出倾角观测时更新为观测值，
     *         无观测 / 解算被拒时保持上一状态，永不为 UNKNOWN。
     */
    public MT1Localizer.HiveState getHiveState(){return hiveState;}

    // ---- 球道计数（BallCounter 透传） ----

    /** @return 球道内当前球数（无光电门时为 0） */
    public int getBallCount() {
        return ballCounter != null ? ballCounter.getCount() : 0;
    }

    /** @return 球道容量（无光电门时回退到 HypParams.BALL_CAPACITY） */
    public int getBallCapacity() {
        return ballCounter != null ? ballCounter.getCapacity() : HypParams.BALL_CAPACITY;
    }

    /** @return 球道是否已满；Sweeper 据此停止吸球（无光电门时恒为 false） */
    public boolean isBallLaneFull() {
        return ballCounter != null && ballCounter.isFull();
    }

    /** @return 还能再装几个球（无光电门时为容量） */
    public int getBallsRemaining() {
        return ballCounter != null ? ballCounter.getRemaining() : HypParams.BALL_CAPACITY;
    }

    /** 清空球道计数：操作手手动清空、炮台发射完毕时调用 */
    public void resetBallCount() {
        if (ballCounter != null) {
            ballCounter.reset();
        }
    }

    /** @return 是否已成功初始化球道光电门 */
    public boolean isBallCounterAvailable() {
        return ballCounter != null;
    }

    public double getOmega(){return currentVelocity2d.angVel;}

    // ---- IMU 功能（原 IMUSensor.java 合并于此） ----

    /**
     * 最近一帧的 IMU 的 yaw/pitch/roll 角。
     * 由 {@code AdaptiveEKFLocalizer.update()} 每帧采样一次后上传，此处不读硬件。
     */
    public YawPitchRollAngles getYawPitchRollAngles() {
        return fusionLocalizer.getImuAngles();
    }

    /**
     * 获取指定单位的 yaw（航向角），取自 {@code AdaptiveEKFLocalizer} 最近一帧的 IMU 采样。
     * @param angleUnit 角度单位
     */
    public double getYaw(AngleUnit angleUnit) {
        return fusionLocalizer.getImuAngles().getYaw(angleUnit);
    }

    /**
     * 重置 IMU yaw 为 0。
     * 注意：Road Runner 定位依赖 IMU yaw 计算航向增量，运行中调用会破坏位姿估计，仅应在初始化/标定时使用。
     */
    public void resetYaw() {
        imu.resetYaw();
    }

}