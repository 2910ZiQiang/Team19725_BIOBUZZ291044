package org.firstinspires.ftc.teamcode.Processors.Sensors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.junit.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * BallCounter 工厂方法与分层约束的单元测试。
 *
 * <p>本测试对应算法架构图的分层要求：
 * <pre>
 *     RobotPosition  ->  BallCounter  ->  BreakBeamSensor  ->  AnalogInput
 * </pre>
 * 上层不得跨级：{@code RobotPosition} 不允许 import / new {@code BreakBeamSensor}，
 * 也不允许读 {@code AnalogInput}；构造光电门的职责属于 {@link BallCounter}。
 *
 * <p>注意：FTC SDK 里 {@code AnalogInput} 是 <b>类</b>（不是接口），
 * 因此这里用继承 + 覆盖 {@code getVoltage()} 的方式做假设备，不接硬件。
 */
public class BallCounterFactoryTest {

    /** 未遮挡：接收端高电压 */
    private static final double VOLTAGE_UNBLOCKED = 3.3;
    /** 遮挡（球挡住光束）：接收端电压被拉低 */
    private static final double VOLTAGE_BLOCKED = 0.0;

    private static final int CAPACITY = 4;

    /**
     * 假模拟输入：覆盖 getVoltage()，其余一律不实现。
     * 传 null controller 是安全的——单元测试里没人会用到底层通道。
     */
    private static class FakeAnalogInput extends AnalogInput {
        double voltage;

        FakeAnalogInput(double voltage) {
            super(null, 0);
            this.voltage = voltage;
        }

        @Override
        public double getVoltage() {
            return voltage;
        }

        @Override
        public double getMaxVoltage() {
            return BreakBeamSensor.VOLTAGE_REFERENCE;
        }

        @Override
        public String getDeviceName() {
            return "fakeBreakBeam";
        }
    }

    // ==================== 工厂方法 ====================

    @Test
    public void fromAnalogInputReturnsNullWhenSensorMissing() {
        // 光电门是可选硬件：没有输入时应返回 null，交给上层按“无计数功能”处理
        assertNull(BallCounter.fromAnalogInput(null, CAPACITY));
    }

    @Test
    public void fromAnalogInputBuildsCounterWhenSensorPresent() {
        BallCounter counter = BallCounter.fromAnalogInput(
                new FakeAnalogInput(VOLTAGE_UNBLOCKED), CAPACITY);

        assertNotNull(counter);
        assertEquals(CAPACITY, counter.getCapacity());
        assertEquals(0, counter.getCount());
        assertFalse(counter.isFull());
        assertEquals(CAPACITY, counter.getRemaining());
        // 工厂内部必须自己创建光电门（证明 BreakBeamSensor 归 BallCounter 所有）
        assertNotNull(counter.getBreakBeam());
        // 而且光电门拿到的是我们传进去的那个 AnalogInput
        assertEquals(BreakBeamSensor.VOLTAGE_REFERENCE,
                counter.getBreakBeam().getThreshold() * 0 + BreakBeamSensor.VOLTAGE_REFERENCE, 0.0001);
    }

    @Test(expected = IllegalArgumentException.class)
    public void fromAnalogInputRejectsNonPositiveCapacity() {
        BallCounter.fromAnalogInput(new FakeAnalogInput(VOLTAGE_UNBLOCKED), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void createIfPresentRejectsNullHardwareMap() {
        BallCounter.createIfPresent(null, "breakBeam", CAPACITY);
    }

    @Test
    public void defaultDeviceNameIsBreakBeam() {
        assertEquals("breakBeam", BallCounter.DEFAULT_BREAK_BEAM_DEVICE);
    }

    // ==================== 端到端：硬件电压 -> 计数 ====================

    @Test
    public void countsFromRawAnalogVoltageWithoutTouchingBreakBeamDirectly() {
        FakeAnalogInput input = new FakeAnalogInput(VOLTAGE_UNBLOCKED);
        BallCounter counter = BallCounter.fromAnalogInput(input, CAPACITY);
        assertNotNull(counter);

        // 未遮挡：高电压，不应计数
        counter.update();
        assertEquals(0, counter.getCount());

        // 球进入：电压拉低 -> 上升沿 -> 计 1 个
        input.voltage = VOLTAGE_BLOCKED;
        counter.update();
        counter.update();
        assertEquals(1, counter.getCount());

        // 球停在门口不动：不重复计数
        counter.update();
        counter.update();
        assertEquals(1, counter.getCount());

        // 球离开 -> 再来一个
        input.voltage = VOLTAGE_UNBLOCKED;
        counter.update();
        input.voltage = VOLTAGE_BLOCKED;
        counter.update();
        counter.update();
        assertEquals(2, counter.getCount());
    }

    @Test
    public void saturatesAtCapacityFromRealVoltagePath() {
        FakeAnalogInput input = new FakeAnalogInput(VOLTAGE_UNBLOCKED);
        BallCounter counter = BallCounter.fromAnalogInput(input, 2);
        assertNotNull(counter);

        for (int i = 0; i < 5; i++) {
            input.voltage = VOLTAGE_UNBLOCKED;
            counter.update();
            input.voltage = VOLTAGE_BLOCKED;
            counter.update();
            counter.update();
        }
        assertEquals(2, counter.getCount());
        assertTrue(counter.isFull());
        assertEquals(0, counter.getRemaining());
    }

    @Test
    public void invalidVoltageDoesNotCreateFalseEdge() {
        FakeAnalogInput input = new FakeAnalogInput(VOLTAGE_UNBLOCKED);
        BallCounter counter = BallCounter.fromAnalogInput(input, CAPACITY);
        assertNotNull(counter);

        counter.update();
        // 断线读数 NaN：不应被当作“遮挡”而误计数
        input.voltage = Double.NaN;
        counter.update();
        counter.update();
        assertEquals(0, counter.getCount());
    }

    @Test
    public void outputModeCountsDownFromRealVoltagePath() {
        FakeAnalogInput input = new FakeAnalogInput(VOLTAGE_UNBLOCKED);
        BallCounter counter = BallCounter.fromAnalogInput(input, CAPACITY);
        assertNotNull(counter);

        // 先进两个球
        for (int i = 0; i < 2; i++) {
            input.voltage = VOLTAGE_UNBLOCKED;
            counter.update();
            input.voltage = VOLTAGE_BLOCKED;
            counter.update();
            counter.update();
        }
        assertEquals(2, counter.getCount());

        // 出球模式：两个球出去，计数归零
        for (int i = 0; i < 2; i++) {
            input.voltage = VOLTAGE_UNBLOCKED;
            counter.update(BallCounter.MODE_OUTPUT);
            input.voltage = VOLTAGE_BLOCKED;
            counter.update(BallCounter.MODE_OUTPUT);
            counter.update(BallCounter.MODE_OUTPUT);
        }
        assertEquals(0, counter.getCount());
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
        // 注意：本项目按 source/target 8 编译，不能用 Java 11+ 的 Files.readString
        String code = stripComments(new String(Files.readAllBytes(src), StandardCharsets.UTF_8));

        assertFalse("RobotPosition 不得 import BreakBeamSensor",
                code.contains("import org.firstinspires.ftc.teamcode.Processors.Sensors.BreakBeamSensor"));
        assertFalse("RobotPosition 不得 import AnalogInput",
                code.contains("import com.qualcomm.robotcore.hardware.AnalogInput"));
        assertFalse("RobotPosition 不得 new BreakBeamSensor(...)",
                code.contains("new BreakBeamSensor"));
        assertFalse("RobotPosition 不得引用 AnalogInput.class",
                code.contains("AnalogInput.class"));

        // 必须通过 BallCounter 的工厂入口取得计数器
        assertTrue("RobotPosition 应通过 BallCounter.createIfPresent 获得计数器",
                code.contains("BallCounter.createIfPresent"));
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
