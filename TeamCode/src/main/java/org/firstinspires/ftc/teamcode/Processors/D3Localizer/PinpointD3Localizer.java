package org.firstinspires.ftc.teamcode.Processors.D3Localizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;

import java.util.Objects;

/**
 * 3D 斜坡补偿定位器 —— Pinpoint(里程计) + Hub IMU(pitch/roll)。
 *
 * <p>与 {@link org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer PinpointLocalizer} 的区别：
 * <ul>
 *   <li>使用 Hub IMU 获取 pitch / roll 角度</li>
 *   <li>将 Pinpoint 场地坐标系速度转换到体坐标系后，再进行斜坡补偿投影到水平面</li>
 *   <li>对补偿后的速度积分得到位姿，适应坡度变化</li>
 * </ul>
 *
 * <p>数学推导参考 {@code Processors/D3Localizer/Theory.md}。
 *
 * <p>设计要点：
 * <ul>
 *   <li>航向角 (yaw) 使用 Pinpoint 内置 IMU 的融合值，与里程计数据同步</li>
 *   <li>俯仰角 (pitch) 和横滚角 (roll) 使用 Hub IMU，独立于 Pinpoint</li>
 *   <li>Pinpoint 的 getVelX/getVelY 返回场地坐标系速度，需先转换到体坐标系再斜坡补偿</li>
 *   <li>不直接使用 Pinpoint 内部维护的位置（斜坡面位置），而是对校正后的速度逐帧积分</li>
 * </ul>
 */
public class PinpointD3Localizer implements Localizer {

    /**
     * Pinpoint 定位器的参数配置类
     */
    public static class Params {
        public double parYTicks = 2460;   // 平行编码器的 y 位置（tick 单位）
        public double perpXTicks = -1970; // 垂直编码器的 x 位置（tick 单位）
    }

    /**
     * 全局参数实例，可通过 FTC Dashboard 实时调整
     */
    public static Params PARAMS = new Params();

    private final GoBildaPinpointDriver pinpoint;
    private final IMU hubImu;

    /**
     * 初始平行编码器方向和初始垂直编码器方向
     */
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    /**
     * 世界坐标系到 Pinpoint 坐标系的变换
     */
    private Pose2d txWorldPinpoint;

    /**
     * Pinpoint 坐标系到机器人坐标系的变换（斜坡面位姿）
     */
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    /** 当前位姿估计 (x, y 英寸; theta 弧度) — 斜坡补偿后的水平面位姿 */
    private Pose2d pose;

    /** 上一帧时间戳 (秒)，用于积分 */
    private double lastTimestamp = -1;

    /**
     * @param hardwareMap   硬件映射
     * @param inPerTick     每个编码器 tick 对应的英寸数
     * @param imuDeviceName Hub IMU 在硬件配置中的名称
     * @param initialPose   初始位姿
     */
    public PinpointD3Localizer(HardwareMap hardwareMap, double inPerTick,
                               String imuDeviceName, Pose2d initialPose) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        hubImu = hardwareMap.get(IMU.class, imuDeviceName);

        double mmPerTick = inPerTick * 25.4;
        pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        // TODO: 如果需要，反转编码器方向
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        pinpoint.setEncoderDirections(initialParDirection, initialPerpDirection);
        pinpoint.resetPosAndIMU();

        txWorldPinpoint = initialPose;
        this.pose = initialPose;
    }

    /**
     * 简化构造：使用默认 IMU 名称 "imu"，初始位姿 (0, 0, 0)。
     */
    public PinpointD3Localizer(HardwareMap hardwareMap, double inPerTick, String imuDeviceName) {
        this(hardwareMap, inPerTick, imuDeviceName, new Pose2d(0, 0, 0));
    }

    // ==================== Localizer 接口 ====================

    @Override
    public void setPose(Pose2d pose) {
        // 计算偏移量: T_WP = pose_desired ∘ T_PR⁻¹
        // txPinpointRobot 是 Pinpoint 当前斜坡面位姿 (每帧 update 中更新),
        // 用于计算世界坐标系到 Pinpoint 原点的偏移, 与 PinpointLocalizer 同构
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
        this.pose = pose;
        lastTimestamp = -1;  // 重置时间戳，避免下一次 update() 使用过大的 dt 导致积分跳变
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    /**
     * 每帧调用一次，完成：
     * <ol>
     *   <li>读取 Pinpoint 场地坐标系速度</li>
     *   <li>场地坐标系 → 体坐标系 (逆旋转 R_z(-ψ))</li>
     *   <li>读取 Hub IMU 的 pitch / roll（由上层传入，见 {@link #update(YawPitchRollAngles)}）</li>
     *   <li>斜坡补偿：将体坐标系速度投影到水平面</li>
     *   <li>水平面体坐标系 → 场地坐标系 (用于位姿积分)</li>
     *   <li>欧拉积分更新位姿</li>
     * </ol>
     *
     * @return 体坐标系下的水平速度 (补偿后) 和角速度，供 EKF predict 使用
     */
    @Override
    public PoseVelocity2d update() {
        // 独立使用（上层未传入 IMU 采样）时自行读一次
        return update(hubImu.getRobotYawPitchRollAngles());
    }

    /**
     * 每帧调用一次，使用上层已采样的 IMU 姿态角，避免同一帧重复读 IMU。
     *
     * @param angles 上层（{@code AdaptiveEKFLocalizer}）本帧采到的 yaw/pitch/roll，可为 null
     * @return 体坐标系下的水平速度 (补偿后) 和角速度，供 EKF predict 使用
     */
    public PoseVelocity2d update(YawPitchRollAngles angles) {
        double now = getNow();

        pinpoint.update();
        if (Objects.requireNonNull(pinpoint.getDeviceStatus()) != GoBildaPinpointDriver.DeviceStatus.READY) {
            lastTimestamp = now;  // 即使未就绪也更新时间戳，避免恢复后积分跳变
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        // ---- 读取 Pinpoint 原始位姿（斜坡面） ----
        txPinpointRobot = new Pose2d(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)
        );

        // ---- 1. 获取 Pinpoint 场地坐标系速度 ----
        // Pinpoint 的 getVelX/getVelY 返回场地坐标系 (field frame) 速度
        double vxField = pinpoint.getVelX(DistanceUnit.INCH);
        double vyField = pinpoint.getVelY(DistanceUnit.INCH);

        // ---- 2. 获取航向角 theta 和 Hub IMU 的 pitch / roll (弧度) ----
        // 补偿后航向 = txWorldPinpoint.heading + txPinpointRobot.heading
        // 等效于 pinpointTheta + thetaOffset，但使用矩阵乘法语义
        double pinpointTheta = txPinpointRobot.heading.toDouble();
        double theta = txWorldPinpoint.heading.toDouble() + pinpointTheta;
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);

        double pitch = 0, roll = 0;
        if (angles != null) {
            pitch = angles.getPitch(AngleUnit.RADIANS);
            roll  = angles.getRoll(AngleUnit.RADIANS);
        }

        // ---- 3. 场地坐标系 → 体坐标系 (逆旋转 R_z(-θ)) ----
        // 斜坡补偿公式 (Theory.md §5.2) 要求输入为体坐标系速度
        // 注意：旋转使用 Pinpoint 原始航向 theta，因为速度旋转是纯几何变换
        double cosPinpoint = Math.cos(pinpointTheta);
        double sinPinpoint = Math.sin(pinpointTheta);
        double vxBody =  vxField * cosPinpoint + vyField * sinPinpoint;
        double vyBody = -vxField * sinPinpoint + vyField * cosPinpoint;

        // ---- 4. 斜坡补偿：体坐标系速度投影到水平面 ----
        // 公式来源: Theory.md §5.2
        //   v_x^{horiz} = v_x·cos(θ) - v_y·sin(θ)·sin(φ)
        //   v_y^{horiz} = v_y·cos(φ)
        double cosPitch = Math.cos(pitch);
        double sinPitch = Math.sin(pitch);
        double sinRoll  = Math.sin(roll);
        double cosRoll  = Math.cos(roll);

        double vxHoriz = vxBody * cosPitch - vyBody * sinPitch * sinRoll;
        double vyHoriz = vyBody * cosRoll;

        // ---- 5. 水平面体坐标系 → 场地坐标系 (用于位姿积分) ----
        // 公式来源: Theory.md §5.3
        //   v_x^{field} = v_x^{horiz}·cos(θ) - v_y^{horiz}·sin(θ)
        //   v_y^{field} = v_x^{horiz}·sin(θ) + v_y^{horiz}·cos(θ)
        // 旋转使用 theta（含偏移），确保积分结果与 setPose 设置的 theta 一致
        double vxFieldCompensated = vxHoriz * cosT - vyHoriz * sinT;
        double vyFieldCompensated = vxHoriz * sinT + vyHoriz * cosT;

        // ---- 6. 欧拉积分更新位姿 ----
        if (lastTimestamp > 0) {
            double dt = now - lastTimestamp;
            if (dt > 0) {
                pose = new Pose2d(
                        pose.position.x + vxFieldCompensated * dt,
                        pose.position.y + vyFieldCompensated * dt,
                        theta
                );
            }
        }
        lastTimestamp = now;

        // ---- 7. 返回体坐标系水平速度 (供 EKF predict 使用) ----
        double angVel = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        return new PoseVelocity2d(new Vector2d(vxHoriz, vyHoriz), angVel);
    }

    // ==================== 公开访问器 ====================

    /** @return Pinpoint 驱动实例 */
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    /** @return Hub IMU 实例 */
    public IMU getHubImu() {
        return hubImu;
    }

    // ==================== 内部工具 ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}