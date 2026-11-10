package org.firstinspires.ftc.teamcode.Processors.Sensors;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

/**
 * BallCounter 双向计数：进入看上升沿 +1，退出看下降沿 -1。
 */
public class BallCounterDirectionTest {

    private FakeBeam beam;
    private BallCounter counter;

    @Before
    public void setUp() {
        beam = new FakeBeam();
        counter = new BallCounter(beam);
    }

    private void frame(boolean blocked, int mode) {
        beam.blocked = blocked;
        counter.update(mode);
    }

    @Test
    public void firstFrameOnlyEstablishesBaseline() {
        frame(true, BallCounter.MODE_INTAKE);
        assertEquals(0, counter.getCount());
    }

    @Test
    public void intakeCountsRisingEdge() {
        frame(false, BallCounter.MODE_INTAKE); // baseline
        frame(true, BallCounter.MODE_INTAKE);
        assertEquals(1, counter.getCount());
    }

    @Test
    public void outputCountsFallingEdge() {
        frame(false, BallCounter.MODE_INTAKE);
        frame(true, BallCounter.MODE_INTAKE);   // count = 1
        frame(true, BallCounter.MODE_OUTPUT);   // 上升沿不计数
        assertEquals(1, counter.getCount());
        frame(false, BallCounter.MODE_OUTPUT);  // 离开光电门，-1
        assertEquals(0, counter.getCount());
    }

    @Test
    public void outputCountsBallAlreadyBlockingAtModeSwitch() {
        frame(false, BallCounter.MODE_INTAKE);
        frame(true, BallCounter.MODE_INTAKE);   // count = 1，最后 blocked = true
        frame(false, BallCounter.MODE_OUTPUT);  // 球离开，-1
        assertEquals(0, counter.getCount());
    }

    @Test
    public void resetKeepsBallStillInBeam() {
        frame(false, BallCounter.MODE_INTAKE);
        frame(true, BallCounter.MODE_INTAKE);
        counter.reset();
        assertEquals(1, counter.getCount());
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsInvalidMode() {
        counter.update(2);
    }

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
