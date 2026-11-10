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

    /** IMU，复用 Road Runner 已初始化的实例（设备名 "imu"），仅用于 resetYaw() */
    private IMU imu;

    /** 队伍颜色（init 时保存，用于按颜色选择球门） */
    private TeamColor teamColor = TeamColor.RED;

    /**
     * 当前瞄准的球门是否位于 AudienceDown 侧。
     * 非 MIDDLE 观测期间由 {@link #update()} 按 MT1 的 HIVE 状态自动推导；
     * 无有效观测（本帧无读数或 MIDDLE）时保持上一帧选择，并允许
     * {@link #setManualGoalDown(boolean)} 手动改写。<b>全程只有一条约定（抬升侧 = 打同侧）</b>：
     * HIVE 在 AUDIENCE_UP 侧 → 打 AudienceUp 门；AUDIENCE_DOWN 侧 → 打 AudienceDown 门；
     * 从未观测到抬升侧且未手动设置时默认 AudienceUp。
     */
    private boolean goalDown = false;

    /**
     * 最近一次有效的抬升侧（{@code AUDIENCE_UP} / {@code AUDIENCE_DOWN}）。
     * 由 MT1 的非 MIDDLE 观测或操作手手动设置（{@link #setManualGoalDown(boolean)}）更新。
     * {@code null} 表示尚未观测到有效抬升侧，此时退化为默认的 AudienceUp 门。
     */
    private MT1Localizer.HiveState lastLiftedSide = null;

    /** 当前球门的缓存值：在 init 与每帧 update 中刷新，getGoalPose 只读该缓存 */
    private Pose2d cachedGoalPose = HypParams.RedAudienceUp;

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

        // ---- 球门选择：保存队伍颜色并刷新默认球门（AudienceUp） ----
        instance.teamColor = (teamColor != null) ? teamColor : TeamColor.RED;
        instance.goalDown = false;
        instance.cachedGoalPose = instance.goalPose(false);

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
        boolean freshObservation = mt1.isHiveEstimated();
        if (freshObservation) {
            hiveState = mt1.getHiveState();
        }

        // 自动估计瞄准球门：全程只有一条约定 —— 抬升侧 = 打同侧
        //   AUDIENCE_UP   → 瞄准 AudienceUp 侧球门
        //   AUDIENCE_DOWN → 瞄准 AudienceDown 侧球门
        //   MIDDLE / 本帧无观测 → 保持上一帧选择（含操作手手动设置）
        // 只有本帧真正拿到非 MIDDLE 观测时才覆盖；这样操作手输入在 MT1 无读数或
        // MIDDLE 时不会每帧被上一帧的残留状态冲掉。
        // 早期实现让 MIDDLE 取「最近一次抬升侧的相反侧」，与上面两个分支的符号约定相反：
        // HIVE 只要抖出一帧 MIDDLE，目标就翻到对侧，炮台随即 180° 回转、档位与转速目标全变。
        if (freshObservation
                && (hiveState == MT1Localizer.HiveState.AUDIENCE_UP
                || hiveState == MT1Localizer.HiveState.AUDIENCE_DOWN)) {
            lastLiftedSide = hiveState;
            goalDown = (hiveState == MT1Localizer.HiveState.AUDIENCE_DOWN);
        } else if (lastLiftedSide == null) {
            goalDown = false;   // 尚未观测到有效抬升侧：默认 AudienceUp
        }
        // MIDDLE 且有历史：goalDown 保持不变（滞回，与"抬升侧=同侧"同一约定）
        cachedGoalPose = goalPose(goalDown);

        currentPose = drive.localizer.getPose();
        // 球道计数：本类每帧「更新」其下级 BallCounter（见球道计数架构图），
        // 光电门采样只在这里发生一次，getBallCount() 只读缓存值
        if (ballCounter != null) {
            ballCounter.update();
        }

        return currentPose;
    }


    public Pose2d getPose2d(){        return currentPose;    }

    public double getX(){     return currentPose.position.x;    }
    public double getY(){   return currentPose.position.y;    }
    public double getTheta(){ return currentPose.heading.toDouble();    }

    // ==================== 球门估计（由 HIVE 状态自动推导，供炮台自动瞄准读取） ====================

    /** @return 当前是否瞄准 AudienceDown 侧球门（由 HIVE 状态自动推导） */
    public boolean isGoalDown() {
        return goalDown;
    }

    /** @return 当前球门点（仅 x/y 有效，heading 无意义） */
    public Pose2d getGoalPose() {
        return cachedGoalPose;
    }

    /**
     * 操作手手动指定当前瞄准球门（TeleOp 中由副控 LT/RT 映射而来）。
     *
     * <p>优先级：MT1 本帧的<b>非 MIDDLE</b> 观测最高，{@link #update()} 会覆盖本选择；
     * 无有效观测时本选择生效，并作为"最近一次抬升侧"保持，直到出现新的非 MIDDLE 观测。
     * 因此应在同一帧的 {@link #update()} <b>之前</b>调用。
     *
     * @param down true = AudienceDown 侧球门，false = AudienceUp 侧球门
     */
    public void setManualGoalDown(boolean down) {
        goalDown = down;
        lastLiftedSide = down ? MT1Localizer.HiveState.AUDIENCE_DOWN
                : MT1Localizer.HiveState.AUDIENCE_UP;
        cachedGoalPose = goalPose(down);
    }

    /**
     * @param down true = AudienceDown 侧，false = AudienceUp 侧
     * @return 按队伍颜色选取的球门点
     */
    private Pose2d goalPose(boolean down) {
        if (teamColor == TeamColor.BLUE) {
            return down ? HypParams.BlueAudienceDown : HypParams.BlueAudienceUp;
        }
        return down ? HypParams.RedAudienceDown : HypParams.RedAudienceUp;
    }
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

    /** @return 球道内累计小球个数（上升沿计数，resetBallCount 后重置为光路中的球数 0/1；无光电门时恒为 0） */
    public int getBallCount() {
        return ballCounter == null ? 0 : ballCounter.getCount();
    }

    /** 重置球道计数并复位边缘检测基准（光路中仍有球时计为 1，透传 {@link BallCounter#reset()}） */
    public void resetBallCount() {
        if (ballCounter == null) {
            return;
        }
        ballCounter.reset();
    }

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