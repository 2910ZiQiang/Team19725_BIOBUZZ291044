package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * 炮台瞄准的纯几何计算（无状态、无硬件、无 Controllers 依赖）。
 *
 * <p>坐标系约定：场坐标系 x 为车体朝 0 弧度时的前进方向、y 为车体左侧；
 * 车体相对坐标系 x 车头、y 车体左侧；heading 逆时针为正。
 *
 * <p>本类只做"位置 → 方位/距离"的换算，不涉及档位选择、滞回、扳机等策略，
 * 因此可被自动瞄准与手动瞄准两条路径共用，避免同一公式多份拷贝。
 */
public final class AimGeometry {

    private AimGeometry() {
    }

    /**
     * 发射器旋转中心地面投影的场坐标。
     *
     * @param pose   车辆位姿
     * @param relPos 旋转中心在车体相对坐标系中的坐标；null 视为原点
     */
    public static Vector2D shooterGroundPos(Pose2d pose, Vector2D relPos) {
        Vector2D rotated = Vector2D.rotate(relPos != null ? relPos : Vector2D.ZERO,
                pose.heading.toDouble());
        return new Vector2D(pose.position.x + rotated.getX(),
                pose.position.y + rotated.getY());
    }

    /** @return 发射器到球门（仅 x/y 有效）的平面水平距离 */
    public static double rangeTo(Vector2D shooterPos, Pose2d goalPose) {
        return Math.hypot(goalPose.position.x - shooterPos.getX(),
                goalPose.position.y - shooterPos.getY());
    }

    /** @return 球门在车体相对坐标系中的方位角，归一化到 (-π, π] */
    public static double bodyYaw(Pose2d pose, Vector2D shooterPos, Pose2d goalPose) {
        return Vector2D.normalizeAngle(
                Math.atan2(goalPose.position.y - shooterPos.getY(),
                        goalPose.position.x - shooterPos.getX())
                        - pose.heading.toDouble());
    }

    /** 单个发射器中心指向球门的车体相对方位角（等价于 {@link #bodyYaw} 的便捷形式） */
    public static double bodyYawToGoal(Pose2d pose, Pose2d goalPose, Vector2D relPos) {
        return bodyYaw(pose, shooterGroundPos(pose, relPos), goalPose);
    }

    /**
     * 两个方位角的劣角平分线。
     *
     * <p>夹角超过 {@code maxSpread} 时单位向量和退化（{@code atan2(0,0)=0} 会把平分线
     * 错当成车头方向），返回 NaN 表示不可解。
     */
    public static double bisectorYaw(double bigYaw, double smallYaw, double maxSpread) {
        if (Math.abs(Vector2D.normalizeAngle(bigYaw - smallYaw)) > maxSpread) {
            return Double.NaN;
        }
        return Math.atan2(Math.sin(bigYaw) + Math.sin(smallYaw),
                Math.cos(bigYaw) + Math.cos(smallYaw));
    }
}
