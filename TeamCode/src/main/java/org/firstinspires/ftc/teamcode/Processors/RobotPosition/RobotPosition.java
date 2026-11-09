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

    /**
     * HIVE 状态跟踪值：MT1 有观测时更新为观测值，无观测 / 解算被拒时保持上一状态。
     * 初值取 MIDDLE（HIVE 水平），跟踪值不会为 {@link MT1Localizer.HiveState#UNKNOWN}。
     */
    private MT1Localizer.HiveState hiveState = MT1Localizer.HiveState.MIDDLE;

    /** IMU，复用 Road Runner 已初始化的实例（设备名 "imu"） */
    private IMU imu;

    /** 最近一次 update() 采到的 IMU 姿态角，供 getYaw / getYawPitchRollAngles 读取 */
    private YawPitchRollAngles cachedImuAngles =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    /** 球道计数器（对射光电门上升沿计数，reset 时清零），对外只做透传 */
    private BallCounter ballCounter;

    /** 球道光电门在机器人配置中的设备名 */
    private static final String BREAK_BEAM_NAME = "breakBeam";

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

        // ---- 球道计数器：光电门由 BallCounter 自行创建与读取，本类只持有并透传 ----
        // 满球停吸由 Sweeper 依据 getBallCount() 执行。
        // 配置中缺少该模拟口时降级为不计数（ballCounter = null），
        // 避免与球道无关的 OpMode（含自动）在 Init 阶段直接失败
        try {
            instance.ballCounter = new BallCounter(hardwareMap, BREAK_BEAM_NAME);
        } catch (RuntimeException e) {
            instance.ballCounter = null;
        }
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

    // 每帧调用：推进融合定位器并返回当前位姿
    public Pose2d update() {
        // 唯一推进点：每帧只调用一次，避免同一帧的视觉观测被重复计入 EKF
        // （MecanumDrive 的轨迹 / 转向 Action 内部已改为复用本帧结果）
        currentVelocity2d = drive.updatePoseEstimate();

        // HIVE 状态跟踪：MT1 成功解出倾角观测时更新，否则保持上一状态
        MT1Localizer mt1 = fusionLocalizer.getMT1();
        if (mt1.isHiveEstimated()) {
            hiveState = mt1.getHiveState();
        }

        currentPose = drive.localizer.getPose();

        // IMU 姿态在本帧采样一次并缓存，getYaw / getYawPitchRollAngles 只读缓存
        cachedImuAngles = imu.getRobotYawPitchRollAngles();

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
    public double getOmega(){return currentVelocity2d.angVel;}

    // ---- 球道计数透传（BallCounter：上升沿计数，reset 时清零） ----

    /**
     * 每帧透传一次球道计数（上升沿 ±1）。
     * 满球判定由 Sweeper 负责，发射完毕与手动清空通过 {@link #resetBallCount()}。
     *
     * @param externalState 0 = 进入模式（球挡光计数 +1），1 = 出球 / 发射模式（-1）
     */
    public void updateBallCount(int externalState) {
        if (ballCounter == null) {
            return;
        }
        ballCounter.update(externalState);
    }

    /** @return 球道内累计小球个数（上升沿计数，resetBallCount 后归零；无光电门时恒为 0） */
    public int getBallCount() {
        return ballCounter == null ? 0 : ballCounter.getCount();
    }

    /** 清零球道计数并复位边缘检测基准（透传 {@link BallCounter#reset()}） */
    public void resetBallCount() {
        if (ballCounter == null) {
            return;
        }
        ballCounter.reset();
    }

    // ---- IMU 功能（原 IMUSensor.java 合并于此） ----

    /**
     * 最近一次 update() 采到的 IMU 的 yaw/pitch/roll 角。
     */
    public YawPitchRollAngles getYawPitchRollAngles() {
        return cachedImuAngles;
    }

    /**
     * 获取指定单位的 yaw（航向角），取自最近一次 update() 的 IMU 采样。
     * @param angleUnit 角度单位
     */
    public double getYaw(AngleUnit angleUnit) {
        return cachedImuAngles.getYaw(angleUnit);
    }

    /**
     * 重置 IMU yaw 为 0。
     * 注意：Road Runner 定位依赖 IMU yaw 计算航向增量，运行中调用会破坏位姿估计，仅应在初始化/标定时使用。
     */
    public void resetYaw() {
        imu.resetYaw();
    }

}