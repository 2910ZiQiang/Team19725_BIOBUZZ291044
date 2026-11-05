package org.firstinspires.ftc.teamcode.utility;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * ShotCycle 的纯逻辑单元测试：用显式时间戳驱动，不依赖系统时钟。
 *
 * <p>覆盖：长脉冲产生一次事件、短脉冲被滤除、持续发射只在释放时结束、
 * 事件只消费一次、reset 清状态。
 */
public class ShotCycleTest {

    @Test
    public void completesAfterLongPulse() {
        ShotCycle cycle = new ShotCycle(0.15);

        cycle.update(false, 0.0);
        cycle.update(true, 1.0);
        assertTrue(cycle.isFiring());
        assertFalse(cycle.consumeCompleted());

        cycle.update(true, 1.1);
        cycle.update(false, 1.2);

        assertFalse(cycle.isFiring());
        assertTrue(cycle.consumeCompleted());
        assertFalse(cycle.consumeCompleted()); // 事件只返回一次
        assertEquals(0.2, cycle.getLastDurationSec(), 1e-9);
    }

    @Test
    public void ignoresTooShortPulse() {
        ShotCycle cycle = new ShotCycle(0.15);
        cycle.update(true, 1.0);
        cycle.update(false, 1.05);
        assertFalse(cycle.consumeCompleted());
        assertEquals(0.05, cycle.getLastDurationSec(), 1e-9);
    }

    @Test
    public void continuousTriggerCompletesOnlyOnRelease() {
        ShotCycle cycle = new ShotCycle(0.15);
        cycle.update(true, 0.0);
        for (int i = 1; i <= 100; i++) {
            cycle.update(true, i * 0.02);
        }
        assertFalse(cycle.consumeCompleted());
        assertTrue(cycle.isFiring());

        cycle.update(false, 3.0);
        assertTrue(cycle.consumeCompleted());
    }

    @Test
    public void resetClearsFiringAndCompletedState() {
        ShotCycle cycle = new ShotCycle(0.15);
        cycle.update(true, 0.0);
        cycle.update(false, 1.0);
        assertTrue(cycle.isCompleted());

        cycle.reset();
        assertFalse(cycle.isFiring());
        assertFalse(cycle.isCompleted());
        assertEquals(0.0, cycle.getLastDurationSec(), 1e-9);
    }

    @Test
    public void defaultDurationIsUsed() {
        ShotCycle cycle = new ShotCycle();
        assertEquals(ShotCycle.DEFAULT_MIN_DURATION_SEC, cycle.getMinDurationSec(), 1e-12);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNegativeDuration() {
        new ShotCycle(-0.01);
    }
}
