package org.firstinspires.ftc.teamcode.Processors.Sensors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * 球道计数分层约束的单元测试。
 *
 * <p>对应算法架构图的分层要求：
 * <pre>
 *     RobotPosition  ->  BallCounter  ->  BreakBeamSensor  ->  AnalogInput
 * </pre>
 * 上层不得跨级：RobotPosition 不允许 import / new BreakBeamSensor，
 * 也不允许读 AnalogInput；构造光电门的职责属于 {@link BallCounter}。
 *
 * <p>本测试用 {@link FakeBeam} 注入假光电门（走 BallCounter 的
 * {@code (BreakBeamSensor, int)} 构造器），不连接任何硬件。
 */
public class BallCounterFactoryTest {

    private static final int CAPACITY = 4;

    /**
     * 假光电门：不接 AnalogInput，只暴露一个可写的遮挡状态。
     * 继承 BreakBeamSensor 是为了复用 BallCounter 的构造签名。
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

    /** 模拟一帧：设定光电门本帧输出，再推进计数器 */
    private static void frame(FakeBeam beam, BallCounter counter, boolean blocked, int mode) {
        beam.blocked = blocked;
        counter.update(mode);
    }

    /** 一个完整小球：先离开光束，再挡光（产生一次上升沿） */
    private static void ball(FakeBeam beam, BallCounter counter) {
        frame(beam, counter, false, BallCounter.MODE_INTAKE);
        frame(beam, counter, true, BallCounter.MODE_INTAKE);
    }

    // ==================== 构造器与容量 ====================

    @Test
    public void buildsCounterWithInjectedBeam() {
        FakeBeam beam = new FakeBeam();
        BallCounter counter = new BallCounter(beam, CAPACITY);

        assertNotNull(counter);
        assertEquals(CAPACITY, counter.getCapacity());
        assertEquals(0, counter.getCount());
        assertFalse(counter.isFull());
        assertEquals(CAPACITY, counter.getRemaining());
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNullBeam() {
        new BallCounter(null, CAPACITY);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonPositiveCapacity() {
        new BallCounter(new FakeBeam(), 0);
    }

    // ==================== 计数行为 ====================

    @Test
    public void countsEachRisingEdgeOnce() {
        FakeBeam beam = new FakeBeam();
        BallCounter counter = new BallCounter(beam, CAPACITY);

        ball(beam, counter);
        ball(beam, counter);
        assertEquals(2, counter.getCount());
        assertFalse(counter.isFull());
        assertEquals(2, counter.getRemaining());
    }

    @Test
    public void heldBeamDoesNotCountRepeatedly() {
        FakeBeam beam = new FakeBeam();
        BallCounter counter = new BallCounter(beam, CAPACITY);

        frame(beam, counter, false, BallCounter.MODE_INTAKE);
        frame(beam, counter, true, BallCounter.MODE_INTAKE);
        frame(beam, counter, true, BallCounter.MODE_INTAKE);
        frame(beam, counter, true, BallCounter.MODE_INTAKE);
        assertEquals(1, counter.getCount());
    }

    @Test
    public void saturatesAtCapacity() {
        FakeBeam beam = new FakeBeam();
        BallCounter counter = new BallCounter(beam, CAPACITY);

        for (int i = 0; i < 7; i++) {
            ball(beam, counter);
        }
        assertEquals(CAPACITY, counter.getCount());
        assertTrue(counter.isFull());
        assertEquals(0, counter.getRemaining());
    }

    @Test
    public void outputModeCountsDownAndClampsAtZero() {
        FakeBeam beam = new FakeBeam();
        BallCounter counter = new BallCounter(beam, CAPACITY);

        ball(beam, counter);
        ball(beam, counter);
        ball(beam, counter);
        assertEquals(3, counter.getCount());

        for (int i = 0; i < 5; i++) {
            frame(beam, counter, false, BallCounter.MODE_OUTPUT);
            frame(beam, counter, true, BallCounter.MODE_OUTPUT);
        }
        assertEquals(0, counter.getCount());
        assertFalse(counter.isFull());
    }

    @Test
    public void resetClearsCountAndRearmsEdgeDetection() {
        FakeBeam beam = new FakeBeam();
        BallCounter counter = new BallCounter(beam, CAPACITY);

        ball(beam, counter);
        ball(beam, counter);
        assertEquals(2, counter.getCount());

        // 清空时光电门仍被挡住：不能把停在门口的同一个球再计一次
        beam.blocked = true;
        counter.reset();
        assertEquals(0, counter.getCount());

        frame(beam, counter, true, BallCounter.MODE_INTAKE);
        assertEquals(0, counter.getCount());

        // 球离开后再来一个，才应重新计数
        ball(beam, counter);
        assertEquals(1, counter.getCount());
    }

    // ==================== 分层约束（静态检查） ====================

    /**
     * 架构图要求：RobotPosition 只调用 BallCounter，不允许跨级接触 BreakBeamSensor
     * 或 AnalogInput。这里读源码做静态断言（注释里提到类名是允许的）。
     */
    @Test
    public void robotPositionMustNotTouchBreakBeamLayer() throws IOException {
        Path src = locateRobotPositionSource();
        if (src == null) {
            // 从其它工作目录运行 Gradle 时找不到源码，跳过而不是误判失败
            return;
        }
        // 本项目按 source/target 8 编译，不能用 Java 11+ 的 Files.readString
        String code = stripComments(new String(Files.readAllBytes(src), StandardCharsets.UTF_8));

        assertFalse("RobotPosition 不得 import BreakBeamSensor",
                code.contains("import org.firstinspires.ftc.teamcode.Processors.Sensors.BreakBeamSensor"));
        assertFalse("RobotPosition 不得 import AnalogInput",
                code.contains("import com.qualcomm.robotcore.hardware.AnalogInput"));
        assertFalse("RobotPosition 不得 new BreakBeamSensor(...)",
                code.contains("new BreakBeamSensor"));

        // 必须通过 BallCounter 的构造器创建（光电门生命周期归 BallCounter）
        assertTrue("RobotPosition 应通过 new BallCounter(hardwareMap, ...) 创建计数器",
                code.contains("new BallCounter(hardwareMap"));
    }

    private static Path locateRobotPositionSource() {
        String rel = "src/main/java/org/firstinspires/ftc/teamcode/Processors/"
                + "RobotPosition/RobotPosition.java";
        Path[] candidates = {
                Paths.get(rel),
                Paths.get("TeamCode", rel),
                Paths.get("..", rel),
        };
        for (Path p : candidates) {
            if (Files.isRegularFile(p)) {
                return p;
            }
        }
        return null;
    }

    /** 去掉 // 行注释与块注释，避免注释里的类名触发误判 */
    private static String stripComments(String code) {
        String noBlock = code.replaceAll("(?s)/\\*.*?\\*/", "");
        return noBlock.replaceAll("(?m)//[^\\n]*", "");
    }
}
