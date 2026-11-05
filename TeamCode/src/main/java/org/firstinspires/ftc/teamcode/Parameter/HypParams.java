package org.firstinspires.ftc.teamcode.Parameter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

@Config
/**
 * 全局超参数配置类
 * 用于集中管理机器人控制系统中的所有超参数，超参应为无需拟合的常量，如工程参数等
 */
//todo：添加球门位置、hive初始状态、hive高度阈值等超参
public class HypParams {
    /**
     * 底盘最大速度（英寸/秒）
     */
    public static double maxV = 2.0;

    /**
     * 底盘最大角速
     */
    public static double maxOmega = Math.PI;

    /**
     * todo:红队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseRed = new Pose2d(-41.3, 55,0);

    /**
     * todo:蓝队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseBlue = new Pose2d(-41.3, -55, 0);
    /**
     * todo:蓝队自动停车姿态（单位：英寸，弧度）
     */
    public static Pose2d StopPoseBlue = new Pose2d(0, -24, Math.PI);
    /**
     * todo:红队自动停车姿态（单位：英寸，弧度）
     */
    public static Pose2d StopPoseRed = new Pose2d(0, 24, Math.PI);
    /**
     * 红队首个自动发射区域（单位：英寸，凸多边形顶点）。
     * 顶点：(24,-12)、(48,-30)、(48,-3)、(72,-30)、(72,-3)。
     */
    public static ConvexPolygon ShootFirstAreaRed = new ConvexPolygon(
            new Vector2D(24, -12),
            new Vector2D(48, -30),
            new Vector2D(48, -3),
            new Vector2D(72, -30),
            new Vector2D(72, -3)
    );
    /**
     * 蓝队首个自动发射区域：红队首个发射区域的原点中心对称（x,y 均取反）。
     * 顶点：(-24,12)、(-48,30)、(-48,3)、(-72,30)、(-72,3)。
     */
    public static ConvexPolygon ShootFirstAreaBlue = new ConvexPolygon(
            new Vector2D(-24, 12),
            new Vector2D(-48, 30),
            new Vector2D(-48, 3),
            new Vector2D(-72, 30),
            new Vector2D(-72, 3)
    );
    /**
     * 红队第二自动发射区域：红队首个发射区域的 y 轴镜像（x 取反）。
     * 顶点：(-24,-12)、(-48,-30)、(-48,-3)、(-72,-30)、(-72,-3)。
     */
    public static ConvexPolygon ShootSecondAreaRed = new ConvexPolygon(
            new Vector2D(-24, -12),
            new Vector2D(-48, -30),
            new Vector2D(-48, -3),
            new Vector2D(-72, -30),
            new Vector2D(-72, -3)
    );
    /**
     * 蓝队第二自动发射区域：蓝队首个发射区域的原点中心对称（x,y 均取反）。
     * 顶点：(24,12)、(48,30)、(48,3)、(72,30)、(72,3)。
     */
    public static ConvexPolygon ShootSecondAreaBlue = new ConvexPolygon(
            new Vector2D(24, 12),
            new Vector2D(48, 30),
            new Vector2D(48, 3),
            new Vector2D(72, 30),
            new Vector2D(72, 3)
    );
    /**
     * 底盘半长（英寸）：底盘中心到最后端 / 最前端中点的距离。
     * 用于在「最后端中点坐标」与「底盘中心坐标」之间换算，以及由 intake 目标点反推中心位姿。
     * todo: 实机测量标定
     */
    public static double chassisHalfLengthIn = 9.0;

    /**
     * 第一个取球点（intake 目标点，单位：英寸）。
     */
    public static Vector2D EatPoint1 = new Vector2D(24, -72);
    /**
     * 第二个取球点（intake 目标点，单位：英寸）。
     */
    public static Vector2D EatPoint2 = new Vector2D(-72, -24);

    /**
     * 底盘半宽（英寸）：底盘中心到左右侧中点的距离。
     * 与 {@link #chassisHalfLengthIn} 一起构成底盘外接矩形，用于判定「整机轮廓完全进入区域」。
     * todo: 实机测量标定
     */
    public static double chassisHalfWidthIn = 9.0;

    /**
     * Hive 正下方矩形区域（红队）：(24,-25)、(24,-1)、(-24,-1)、(-24,-25)。
     * 即 x∈[-24,24]、y∈[-25,-1]；蓝队为其中心对称区域（{@link #HiveRectBlue}）。
     */
    public static ConvexPolygon HiveRectRed = new ConvexPolygon(
            new Vector2D(24, -25),
            new Vector2D(24, -1),
            new Vector2D(-24, -1),
            new Vector2D(-24, -25)
    );
    /**
     * Hive 正下方矩形区域（蓝队）：红队区域的中心对称 (x,y)->(-x,-y)，
     * 即 x∈[-24,24]、y∈[1,25]。
     */
    public static ConvexPolygon HiveRectBlue = new ConvexPolygon(
            new Vector2D(-24, 25),
            new Vector2D(-24, 1),
            new Vector2D(24, 1),
            new Vector2D(24, 25)
    );

    /**
     * Hive 禁线（红队）：(24,-24) 与 (-24,-24) 两点间线段。机器人（含整机轮廓）不得与其接触。
     * 蓝队为其中心对称线段（{@link #HiveForbiddenLineBlueA}/{@link #HiveForbiddenLineBlueB}）。
     */
    public static Vector2D HiveForbiddenLineRedA = new Vector2D(24, -24);
    public static Vector2D HiveForbiddenLineRedB = new Vector2D(-24, -24);
    /**
     * Hive 禁线（蓝队）：中心对称后为 (-24,24) 与 (24,24) 两点间线段。
     */
    public static Vector2D HiveForbiddenLineBlueA = new Vector2D(-24, 24);
    public static Vector2D HiveForbiddenLineBlueB = new Vector2D(24, 24);

    /**
     * 绕行 Hive 矩形时，整机轮廓越出矩形边界方向上的额外安全间隙（英寸），
     * 用于 {@code GoTo*AvoidHiveAction} 的绕行路线。需实机标定。
     */
    public static double hiveAvoidClearanceIn = 2.0;

    /**
     * todo:红队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseRed = new Pose2d(63, -60.7, -Math.PI/2);

    /**
     * todo:蓝队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseBlue = new Pose2d(63, 60.7, Math.PI/2);

    /**
     * 初始操控模式标志
     * true=无头模式（场心地坐标系），false=有头模式（机器人坐标系）
     */
    public static boolean InitialUseNoHeadMode = false;

    /**
     * todo:停车时间阈值（单位：毫秒）
     * 自动阶段剩余时间小于此值时执行停车
     */
    public static long PARK_TIME_THRESHOLD_MS = 3000;

    /**
     * FTC 场地半边长（英寸）：12 ft × 12 ft 场地以原点为中心，边界为 x/y = ±72。
     * 用于自适应停靠 Action 判定「整机外接矩形是否接触场地边框」。
     */
    public static double fieldHalfSizeIn = 72.0;

    /**
     * 自适应停靠时整机外接矩形与场地边框保留的安全间隙（英寸）。
     * 检测到接触后，机器人会被推回到「外接矩形距边框 ≥ 该值」的位置。
     */
    public static double parkBorderMarginIn = 3.0;

    /**
     * 自动阶段总时长（单位：毫秒）
     */
    public static long AUTONOMOUS_DURATION_MS = 30000;

    /**
     * 球门位置（瞄准目标点，仅 x/y 有意义，heading 忽略）：
     * Red_Audience_Up: (17.7, -12.75)
     * Red_Audience_Down: (-17.7, -12.75)
     * Blue_Audience_Up: (17.7, 12.75)
     * Blue_Audience_Down: (-17.7, 12.75)
     */
    public static Pose2d RedAudienceUp = new Pose2d(17.7, -12.75, 0);
    public static Pose2d RedAudienceDown = new Pose2d(-17.7, -12.75, 0);
    public static Pose2d BlueAudienceUp = new Pose2d(17.7, 12.75, 0);
    public static Pose2d BlueAudienceDown = new Pose2d(-17.7, 12.75, 0);

    // ==================== HIVE 单地图解算超参 ====================
    // 单位约定：长度一律英寸（与场地坐标系一致），角度一律度（变量名带 Deg 后缀），
    // 解算内部再转换为弧度（见 MT1Localizer.solveHiveObservation）。

    /**
     * todo:HIVE 枢轴中心相对场地地面的高度 h（英寸）
     * 用于 MT1Localizer.md §3.2 的贴地约束解算；误差会线性传入 x 修正量，必须实机标定
     */
    public static double hivePivotHeightIn = 43.95;

    /**
     * todo:CELL 放下时（HIVE 水平/中间）的倾角上界（度）
     * |θ| ≤ 该值 → MIDDLE；同时也是"稳定位于一侧"的倾角下界，需实机标定
     */
    public static double hiveCellDownAngleDeg = 23.0;

    /**
     * todo:CELL 抬起时（HIVE 稳定位于一侧）的倾角上界（度），必须 > hiveCellDownAngleDeg
     * |θ| 超过该值 → 该帧直接拒绝（该值同时作为三角方程取根筛选的范围），需实机标定
     */
    public static double hiveCellUpAngleDeg = 35.0;

    /**
     * todo:姿态交叉校验容差（度）
     * 由位置解出的 φ 与由 rawPitch 独立推算的 φ 之差超过该值 → 判为 z' 退化帧, 丢弃该帧 HIVE 观测。
     * 前提: 机器人贴地平放（真实 pitch ≈ 0）。需实机标定
     */
    public static double hivePitchCheckTolDeg = 10.0;

    /**
     * todo:HIVE 倾角符号约定
     * true  → θ > 0 表示 AUDIENCE_UP 侧抬升
     * false → θ > 0 表示 AUDIENCE_DOWN 侧抬升
     * 具体对应关系由 fmap 与 HIVE 安装方向决定，需实机标定
     */
    public static boolean hivePositiveAngleIsAudienceUp = Boolean.FALSE;
}