package org.firstinspires.ftc.teamcode.Parameter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utility.ChassisGeometry;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * 自动阶段专用参数：<b>盟友活动区（我方禁止活动区）</b>的统一配置入口。
 *
 * <p><b>用途</b>：3～5 名队友在「我方区域」的某段时间内会占据一块场地。为避免抢位 / 相撞，
 * 把「队友活动范围 + 该范围生效的时间窗」登记为<b>我方在该时段内的禁止活动区域</b>。
 * 运动规划与动作投递前只需查询本类即可：
 * <ul>
 *   <li>{@link #activeZones(TeamColor, long)} —— 指定时刻生效的禁区列表；</li>
 *   <li>{@link #isForbidden(TeamColor, Pose2d, long)} —— 给定底盘位姿是否落入任一禁区；</li>
 *   <li>{@link #activeZoneNames(TeamColor, long)} —— 生效禁区名称（遥测 / 日志用）。</li>
 * </ul>
 *
 * <p><b>坐标系与红蓝镜像</b>：所有区域按<b>红队</b>视角以场地坐标（英寸）填写；
 * 蓝队区域由 {@link ConvexPolygon#inRelative(double, double, double)} 以
 * {@code (0,0,π)} 做原点中心对称得到，与 {@link HypParams} 中发射区域 / Hive 矩形的红蓝约定一致。
 *
 * <p><b>时间基准</b>：{@link AllyZone#startMs} / {@link AllyZone#endMs} 为
 * <b>相对自动阶段 START 的毫秒数</b>，与 OpMode 中的 {@code ElapsedTime matchTime} 同一基准。
 * 自动阶段总时长为 {@link HypParams#AUTONOMOUS_DURATION_MS}。
 *
 * <p><b>参数修改说明</b>：区域与时间窗在类加载时构建为不可变列表，
 * 修改后需重新运行 OpMode 生效（Dashboard 的 Config 页可改本类的标量参数）。
 */
@Config
public class AutoParams {

    // ==================== 全局开关与安全参数 ====================

    /**
     * 是否启用盟友活动区（禁止活动区）约束。false 时所有查询恒返回「不受限」。
     */
    public static boolean useAllyZones = true;

    /**
     * 禁区安全外扩量（英寸）：判定时把禁区向外扩张该距离，
     * 即「整机轮廓距禁区边界小于该值」也算闯入。用于吸收定位误差。todo: 实机标定
     */
    public static double allyZoneClearanceIn = 2.0;

    /**
     * 脱离禁区时的额外余量（英寸）：脱困目标点会停在
     * 「禁区边界 + {@link #allyZoneClearanceIn} + 本值」之外，
     * 避免刚好贴边、下一帧又被判为闯入而反复触发。todo: 实机标定
     */
    public static double allyZoneEscapeMarginIn = 1.0;

    // ==================== 盟友活动区（红队视角，场地坐标 / 英寸） ====================
    // todo: 以下区域与时间窗均为占位默认值，必须按队友实际跑位与战术安排修改。

    /** 队友 A：红队起始区附近（0～6 s） */
    public static AllyZone ALLY_ZONE_1 = new AllyZone(
            "AllyA-StartCorner", 0, 6000,
            new ConvexPolygon(
                    new Vector2D(-70, 40),
                    new Vector2D(-40, 40),
                    new Vector2D(-40, 60),
                    new Vector2D(-70, 60)));

    /** 队友 B：中路偏左（3～12 s） */
    public static AllyZone ALLY_ZONE_2 = new AllyZone(
            "AllyB-MidLeft", 3000, 12000,
            new ConvexPolygon(
                    new Vector2D(-20, 30),
                    new Vector2D(10, 30),
                    new Vector2D(10, 50),
                    new Vector2D(-20, 50)));

    /** 队友 C：右侧起始区（0～9 s） */
    public static AllyZone ALLY_ZONE_3 = new AllyZone(
            "AllyC-RightStart", 0, 9000,
            new ConvexPolygon(
                    new Vector2D(25, 35),
                    new Vector2D(55, 35),
                    new Vector2D(55, 55),
                    new Vector2D(25, 55)));

    /** 队友 D：靠近 Hive 上方通道（8～20 s） */
    public static AllyZone ALLY_ZONE_4 = new AllyZone(
            "AllyD-NearHive", 8000, 20000,
            new ConvexPolygon(
                    new Vector2D(-15, 8),
                    new Vector2D(15, 8),
                    new Vector2D(15, 28),
                    new Vector2D(-15, 28)));

    /** 队友 E：右下侧（12～30 s） */
    public static AllyZone ALLY_ZONE_5 = new AllyZone(
            "AllyE-RightLow", 12000, 30000,
            new ConvexPolygon(
                    new Vector2D(40, 5),
                    new Vector2D(68, 5),
                    new Vector2D(68, 25),
                    new Vector2D(40, 25)));

    /** 红队视角的盟友活动区（禁止活动区）列表 */
    public static final List<AllyZone> ALLY_ZONES_RED = Collections.unmodifiableList(
            Arrays.asList(ALLY_ZONE_1, ALLY_ZONE_2, ALLY_ZONE_3, ALLY_ZONE_4, ALLY_ZONE_5));

    /** 蓝队视角的盟友活动区（禁止活动区）列表：红队逐个原点中心对称 */
    public static final List<AllyZone> ALLY_ZONES_BLUE = Collections.unmodifiableList(mirror(ALLY_ZONES_RED));

    // ==================== 查询接口 ====================

    /** 指定队伍的盟友活动区（禁止活动区）全量列表（未按时间过滤）。 */
    public static List<AllyZone> zonesFor(TeamColor teamColor) {
        return (teamColor == TeamColor.BLUE) ? ALLY_ZONES_BLUE : ALLY_ZONES_RED;
    }

    /**
     * 指定时刻<b>生效</b>的禁止活动区列表。
     *
     * @param teamColor 队伍颜色
     * @param matchMs   自动阶段已流逝时间（ms，与 OpMode 的 matchTime 同基准）
     * @return 生效的禁区列表；未启用时返回空列表
     */
    public static List<AllyZone> activeZones(TeamColor teamColor, long matchMs) {
        if (!useAllyZones) {
            return Collections.emptyList();
        }
        List<AllyZone> result = new ArrayList<>();
        for (AllyZone zone : zonesFor(teamColor)) {
            if (zone.activeAt(matchMs)) {
                result.add(zone);
            }
        }
        return result;
    }

    /** 指定时刻生效的禁止活动区名称列表（遥测 / 日志用）。 */
    public static List<String> activeZoneNames(TeamColor teamColor, long matchMs) {
        List<String> names = new ArrayList<>();
        for (AllyZone zone : activeZones(teamColor, matchMs)) {
            names.add(zone.name);
        }
        return names;
    }

    /**
     * 判断给定底盘位姿此刻是否落入禁止活动区。
     *
     * <p>以整机外接矩形的<b>四个角点</b>逐点判定（{@link ChassisGeometry#footprintCorners(Pose2d)}），
     * 任一角点进入「禁区外扩 {@link #allyZoneClearanceIn}」的范围即视为闯入，
     * 因此判定的是<b>整机轮廓</b>而非单点。
     *
     * @param teamColor 队伍颜色
     * @param pose      底盘中心位姿
     * @param matchMs   自动阶段已流逝时间（ms）
     * @return 闯入任一禁区返回 {@code true}
     */
    public static boolean isForbidden(TeamColor teamColor, Pose2d pose, long matchMs) {
        return firstConflictingZone(teamColor, pose, matchMs) != null;
    }

    /**
     * 返回此刻与给定底盘位姿冲突的第一个禁区（无冲突返回 {@code null}），
     * 便于调用方决定绕行方向或打印冲突来源。
     */
    public static AllyZone firstConflictingZone(TeamColor teamColor, Pose2d pose, long matchMs) {
        if (!useAllyZones) {
            return null;
        }
        Vector2D[] corners = ChassisGeometry.footprintCorners(pose);
        for (AllyZone zone : activeZones(teamColor, matchMs)) {
            for (Vector2D corner : corners) {
                if (zone.distanceTo(corner.getX(), corner.getY()) <= allyZoneClearanceIn) {
                    return zone;
                }
            }
        }
        return null;
    }

    /**
     * 求「脱离当前生效禁区」的最近安全位姿（航向不变）。
     *
     * <p>以冲突禁区的<b>轴对齐包围盒</b>为界，把底盘中心推到最近的盒外位置；
     * 盒外还额外留出 {@link #allyZoneClearanceIn} + {@link #allyZoneEscapeMarginIn}，
     * 保证脱困后下一帧不会再被判为闯入。
     *
     * <p>目标点最终会被夹在场地内（依据 {@link HypParams#fieldHalfSizeIn} 与整机半投影），
     * 避免脱困时开出场地边框。
     *
     * @param teamColor 队伍颜色
     * @param pose      当前底盘位姿
     * @param matchMs   自动阶段已流逝时间（ms）
     * @return 最近安全位姿；无冲突时原样返回 {@code pose}
     */
    public static Pose2d nearestSafePose(TeamColor teamColor, Pose2d pose, long matchMs) {
        Pose2d result = pose;
        // 多个禁区重叠时逐个脱困，最多迭代 4 次
        for (int i = 0; i < 4; i++) {
            AllyZone conflict = firstConflictingZone(teamColor, result, matchMs);
            if (conflict == null) {
                break;
            }
            result = escapeFrom(conflict, result);
        }
        return result;
    }

    // ==================== 内部实现 ====================

    /** 以禁区包围盒为界，把底盘中心推到最近的盒外位置（航向不变） */
    private static Pose2d escapeFrom(AllyZone zone, Pose2d pose) {
        Vector2D[] vertices = zone.area.getVertices();
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (Vector2D v : vertices) {
            minX = Math.min(minX, v.getX());
            maxX = Math.max(maxX, v.getX());
            minY = Math.min(minY, v.getY());
            maxY = Math.max(maxY, v.getY());
        }

        // 整机外接矩形在场地轴上的半投影
        double heading = pose.heading.toDouble();
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double ex = Math.abs(HypParams.chassisHalfLengthIn * c)
                + Math.abs(HypParams.chassisHalfWidthIn * s);
        double ey = Math.abs(HypParams.chassisHalfLengthIn * s)
                + Math.abs(HypParams.chassisHalfWidthIn * c);
        double pad = allyZoneClearanceIn + allyZoneEscapeMarginIn;

        double loX = minX - ex - pad;
        double hiX = maxX + ex + pad;
        double loY = minY - ey - pad;
        double hiY = maxY + ey + pad;

        double x = pose.position.x;
        double y = pose.position.y;

        // 四个出盒方向取位移最小者（等价于最近出界点）
        double best = Math.abs(x - loX);
        double nx = loX;
        double ny = y;
        if (Math.abs(hiX - x) < best) {
            best = Math.abs(hiX - x);
            nx = hiX;
            ny = y;
        }
        if (Math.abs(y - loY) < best) {
            best = Math.abs(y - loY);
            nx = x;
            ny = loY;
        }
        if (Math.abs(hiY - y) < best) {
            nx = x;
            ny = hiY;
        }

        // 夹在场地内，避免脱困时开出边框
        double limitX = HypParams.fieldHalfSizeIn - ex;
        double limitY = HypParams.fieldHalfSizeIn - ey;
        return new Pose2d(clamp(nx, -limitX, limitX), clamp(ny, -limitY, limitY), heading);
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }

    /** 红队列表逐个做原点中心对称，得到蓝队列表 */
    private static List<AllyZone> mirror(List<AllyZone> redZones) {
        List<AllyZone> mirrored = new ArrayList<>(redZones.size());
        for (AllyZone zone : redZones) {
            mirrored.add(zone.mirrored());
        }
        return mirrored;
    }

    /**
     * 单个盟友活动区：一块区域 + 一个时间窗。
     *
     * <p>语义为「队友在该时间段内会占据该区域」，因此对我方而言，
     * 该时间窗内它就是<b>禁止活动区域</b>。
     */
    public static final class AllyZone {

        /** 区域名称（如队友编号 + 位置），用于遥测与日志 */
        public final String name;
        /** 时间窗起点（ms，相对自动阶段 START） */
        public final long startMs;
        /** 时间窗终点（ms，相对自动阶段 START，含端点） */
        public final long endMs;
        /** 区域形状（场地坐标，英寸） */
        public final ConvexPolygon area;

        /**
         * @param name    区域名称
         * @param startMs 时间窗起点（ms）
         * @param endMs   时间窗终点（ms）
         * @param area    区域形状
         */
        public AllyZone(String name, long startMs, long endMs, ConvexPolygon area) {
            this.name = name;
            this.startMs = Math.min(startMs, endMs);
            this.endMs = Math.max(startMs, endMs);
            this.area = area;
        }

        /**
         * 该禁区在给定时刻是否生效。
         *
         * @param matchMs 自动阶段已流逝时间（ms）
         */
        public boolean activeAt(long matchMs) {
            return matchMs >= startMs && matchMs <= endMs;
        }

        /** 点是否位于本区域内（不含安全外扩）。 */
        public boolean contains(double x, double y) {
            return area.Contains(x, y);
        }

        /**
         * 点到本区域的距离（区域内返回 0），用于安全外扩判定。
         *
         * @return 距离（英寸）
         */
        public double distanceTo(double x, double y) {
            return area.NearestVectorFrom(x, y).getDistance();
        }

        /**
         * 本区域的原点中心对称副本，时间窗不变（红队区域 → 蓝队区域）。
         */
        public AllyZone mirrored() {
            return new AllyZone(name, startMs, endMs, area.inRelative(0, 0, Math.PI));
        }

        @Override
        public String toString() {
            return name + "[" + startMs + "~" + endMs + "ms]";
        }
    }
}
