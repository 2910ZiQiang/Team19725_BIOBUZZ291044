package org.firstinspires.ftc.teamcode.utility;

/**
 * 时间基准。
 *
 * <p>全项目统一使用单调时钟 {@link System#nanoTime()}（秒，double），避免各模块各自
 * 复制 {@code System.nanoTime() / 1e9}。注意不要替换成 {@code currentTimeMillis()}：
 * 后者会随系统对时跳变，破坏各控制器 dt 的单调性。
 */
public final class Time {

    private Time() {
    }

    /** @return 单调时钟当前时刻（秒） */
    public static double seconds() {
        return System.nanoTime() / 1e9;
    }
}
