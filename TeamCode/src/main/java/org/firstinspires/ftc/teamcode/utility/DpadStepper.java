package org.firstinspires.ftc.teamcode.utility;

/**
 * 方向键"按下即走一步、长按按周期连续步进"的辅助器。
 * 松开或换向时计时复位，因此换向立刻响应。
 */
public class DpadStepper {

    private int lastDir = 0;
    private long nextTime = 0;

    /**
     * @param negative 负方向键是否按下
     * @param positive 正方向键是否按下
     * @param periodMs 长按连续步进周期（毫秒）
     * @return 本帧应施加的步进方向 (-1 / 0 / +1)
     */
    public int step(boolean negative, boolean positive, int periodMs) {
        int dir = (positive ? 1 : 0) - (negative ? 1 : 0);
        if (dir == 0) {
            lastDir = 0;
            return 0;
        }
        long now = System.currentTimeMillis();
        if (dir != lastDir || now >= nextTime) {
            lastDir = dir;
            nextTime = now + periodMs;
            return dir;
        }
        return 0;
    }
}
