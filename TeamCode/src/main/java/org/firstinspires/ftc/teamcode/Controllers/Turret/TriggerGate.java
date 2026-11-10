package org.firstinspires.ftc.teamcode.Controllers.Turret;

/**
 * 单个发射器扳机的"自动释放 + 最短保持"状态机。纯逻辑、无硬件、无时间源，
 * 因此可以被穷举输入序列直接单元测试。
 *
 * <p>行为（与重构前 {@code Turret.update(boolean...)} 内联的两段完全一致）：
 * <ol>
 *   <li>强制释放 → 立即释放，清保持窗口（松手后当帧即由下一分支关闭，不残留）；</li>
 *   <li>取消允许发射或平转不可行 → 立即关闭，清保持窗口；</li>
 *   <li>机构与平转均到位 → 释放，并把保持窗口推到 {@code now + holdSeconds}；</li>
 *   <li>否则 → 仅在"指向仍有效且仍在保持窗口内"时继续释放。</li>
 * </ol>
 *
 * <p>保持窗口只吸收转速/俯仰就绪的逐帧抖动，不吸收指向失效：窗口内的续开仍要求
 * {@code yawHoldOk}，否则球门切换或底盘持续转动时会被连续开到窗口结束。
 */
final class TriggerGate {

    /** 最近一次决策后的扳机状态 */
    private boolean fired = false;
    /** 本段释放的最短保持截止时刻（秒）；0 表示不在保持窗口 */
    private double holdUntil = 0;

    /**
     * 推进一帧。
     *
     * @param forceRelease 强制释放（无视就绪与平转可行性）
     * @param allowFire    是否允许发射
     * @param feasible     平转目标是否可达
     * @param canFire      平转到位且本发射器机构就绪
     * @param yawHoldOk    指向仍然有效（用于保持窗口内的续开）
     * @param nowSec       当前时刻（秒，单调）
     * @param holdSeconds  单段释放的最短保持时长
     * @return 本帧是否释放
     */
    boolean update(boolean forceRelease, boolean allowFire, boolean feasible,
                   boolean canFire, boolean yawHoldOk, double nowSec, double holdSeconds) {
        if (forceRelease) {
            fired = true;
            holdUntil = 0;
        } else if (!allowFire || !feasible) {
            fired = false;
            holdUntil = 0;
        } else if (canFire) {
            fired = true;
            holdUntil = nowSec + holdSeconds;
        } else {
            fired = yawHoldOk && nowSec < holdUntil;
        }
        return fired;
    }

    /** @return 本帧是否释放 */
    boolean isFired() {
        return fired;
    }

    /** 清空（停止炮台 / 切出手动时调用），避免 {@code is*Firing()} 残留 true */
    void clear() {
        fired = false;
        holdUntil = 0;
    }
}
