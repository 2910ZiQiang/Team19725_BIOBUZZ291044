package org.firstinspires.ftc.teamcode.Processors.Sensors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * BallCounter 的纯逻辑单元测试：用假光电门（FakeBeam）驱动，不连接任何硬件。
 *
 * <p>覆盖：上升沿计数、停留不重复计数、容量饱和、出球递减不为负、
 * 清空后的边缘基准、容量修改钳制。
 */
public class BallCounterTest {

    private FakeBeam beam;
    private BallCounter counter;

    @Before
    public void setUp() {
        beam = new FakeBeam();
        counter = new BallCounter(beam, 4);
    }

    /** 模拟一帧：设定光电门本帧输出，再推进计数器 */
    private void frame(boolean blocked, int mode) {
        beam.blocked = blocked;
        counter.update(mode);
    }

    /** 一个完整小球：先离开光束，再挡光（产生一次上升沿） */
    private void ball() {
        frame(false, BallCounter.MODE_INTAKE);
        frame(true, BallCounter.MODE_INTAKE);
    }

    @Test
    public void countsEachRisingEdgeOnce() {
        ball();
        ball();
        assertEquals(2, counter.getCount());
        assertFalse(counter.isFull());
        assertEquals(2, counter.getRemaining());
    }

    @Test
    public void heldBeamDoesNotCountRepeatedly() {
        frame(false, BallCounter.MODE_INTAKE);
        frame(true, BallCounter.MODE_INTAKE);
        frame(true, BallCounter.MODE_INTAKE);
        frame(true, BallCounter.MODE_INTAKE);
        assertEquals(1, counter.getCount());
    }

    @Test
    public void saturatesAtCapacity() {
        for (int i = 0; i < 7; i++) {
            ball();
        }
        assertEquals(4, counter.getCount());
        assertEquals(4, counter.getCapacity());
        assertTrue(counter.isFull());
        assertEquals(0, counter.getRemaining());
    }

    @Test
    public void outputModeCountsDownAndClampsAtZero() {
        ball();
        ball();
        ball();
        assertEquals(3, counter.getCount());

        for (int i = 0; i < 5; i++) {
            frame(false, BallCounter.MODE_OUTPUT);
            frame(true, BallCounter.MODE_OUTPUT);
        }
        assertEquals(0, counter.getCount());
        assertFalse(counter.isFull());
    }

    @Test
    public void resetClearsCountAndRearmsEdgeDetection() {
        ball();
        ball();
        assertEquals(2, counter.getCount());

        // 清空时光电门仍被挡住：不能把停在门口的同一个球再计一次
        beam.blocked = true;
        counter.reset();
        assertEquals(0, counter.getCount());

        frame(true, BallCounter.MODE_INTAKE);
        assertEquals(0, counter.getCount());

        // 球离开后再来一个，才应重新计数
        frame(false, BallCounter.MODE_INTAKE);
        frame(true, BallCounter.MODE_INTAKE);
        assertEquals(1, counter.getCount());
    }

    @Test
    public void onShotCompleteClearsCount() {
        ball();
        ball();
        counter.onShotComplete();
        assertEquals(0, counter.getCount());
    }

    @Test
    public void setCapacityClampsCurrentCount() {
        ball();
        ball();
        ball();
        assertEquals(3, counter.getCount());

        counter.setCapacity(2);
        assertEquals(2, counter.getCount());
        assertEquals(2, counter.getCapacity());
        assertTrue(counter.isFull());
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonPositiveCapacityInConstructor() {
        new BallCounter(beam, 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonPositiveCapacityInSetter() {
        counter.setCapacity(0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsInvalidMode() {
        counter.update(2);
    }

    /**
     * 假光电门：不接 AnalogInput，只暴露一个可写的遮挡状态。
     * 继承 BreakBeamSensor 是为了复用 BallCounter 的构造签名，同时避免 Mockito
     * 在 Java 21 下的初始化问题。
     */
    private static class FakeBeam extends BreakBeamSensor {
        boolean blocked;

        FakeBeam() {
            super(null, 0.8, 0.5);
        }

        @Override
        public boolean update() {
            return blocked;
        }

        @Override
        public boolean isBlocked() {
            return blocked;
        }

        @Override
        public void reset() {
            // 假光电门没有滤波状态
        }
    }
}
