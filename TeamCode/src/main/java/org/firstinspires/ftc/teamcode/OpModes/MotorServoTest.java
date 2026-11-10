package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * 单电机 + 单舵机联合测试台：用同一个手柄同时控制一个电机与一个舵机。
 *
 * <p><b>手柄映射（一操）</b>
 * <ul>
 *   <li>方向键 上 / 下 → 增大 / 减小电机目标功率（点按一次走一个步长，按住按周期连续步进）</li>
 *   <li>方向键 右 / 左 → 增大 / 减小舵机目标位置（上升沿触发，步长见 Dashboard）</li>
 *   <li>Y 键 → 紧急停车（电机目标归零、功率清零）</li>
 * </ul>
 * 电机用「按住状态 + 周期步进」：点按给一个步长，按住则每 repeatPeriodMs 再加一个步长，
 * 避免步长较小时点按一次功率变化太小、看起来像没有反应。
 * 舵机仍用上升沿检测（{@code dpad*WasPressed()}），每按一次走一步，避免长按粘滞。
 *
 * <p><b>电机</b>
 * 运行在 {@code RUN_WITHOUT_ENCODER}，由程序直接下发功率（开环）：{@code setPower}。
 * 编码器仍可正常读取，遥测中的 {@code Velocity (tick/s)} 即编码器实测转速，
 * 用于观察当前功率下电机的实际转速。
 *
 * <p><b>舵机</b>
 * 目标位置为归一化值 [0,1]，被钳制在 {@link #minPosition} ~ {@link #maxPosition}
 * 之间（防止机械过冲撞限位），每帧下发 {@code Servo.setPosition}。
 *
 * <p><b>整定</b>：在 FTC Dashboard 的 {@code MotorServoTest} 分组中实时修改
 * powerStep / maxPower / positionStep / minPosition / maxPosition。
 */
@Config
@TeleOp(name = "MotorServoTest", group = "Tests")
public class MotorServoTest extends LinearOpMode {


    // ==================== 硬件设备名 ====================

    /** 被控电机在 Robot Configuration 中的名称（改动后需重新 Init） */
    public static String motorName = "motor";
    /** 舵机在 Robot Configuration 中的名称（改动后需重新 Init） */
    public static String servoName = "servo";

    // ==================== 可调参数（FTC Dashboard 实时生效） ====================

    /** 每次按键的功率增量 */
    public static double powerStep = 0.05;
    /** 目标功率限幅，范围 (0, 1] */
    public static double maxPower = 1.0;
    /** 按住方向键时的连续步进周期 (ms)，越小爬升越快 */
    public static int repeatPeriodMs = 120;

    // ---- 舵机 ----

    /** 每次按键的位置增量 [0,1] */
    public static double positionStep = 0.05;
    /** 允许的最小位置（防机械过冲），范围 [0,1] */
    public static double minPosition = 0.0;
    /** 允许的最大位置（防机械过冲），范围 [0,1] */
    public static double maxPosition = 1.0;

    // ==================== 运行时状态 ====================

    private DcMotorEx motor;
    private Servo servo;

    /** 电机目标功率 [-1, 1] */
    private double targetPower = 0;
    /** 舵机当前目标位置（归一化 [0,1]） */
    private double targetPosition = 0.5;

    /** 上一帧的步进方向，用于区分"新按下"与"按住重复" */
    private int lastStepDir = 0;
    /** 下一次允许连续步进的时间戳 (ms) */
    private long nextStepTime = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- 电机初始化（开环功率控制） ----
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        // ---- 舵机初始化 ----
        servo = hardwareMap.get(Servo.class, servoName);
        // 以舵机当前的指令位置作为初始目标，避免上电即跳到 0.5
        targetPosition = servo.getPosition();

        telemetry.addData("Motor", motorName);
        telemetry.addData("Servo", servoName);
        telemetry.addLine("Dpad U/D: motor power | Dpad L/R: servo position | Y: E-stop");
        telemetry.addLine("Params: Dashboard group MotorServoTest");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 方向键步进电机目标功率：点按一次走一步，按住按周期连续走
            // （dpad_up/dpad_down 是"当前是否按住"，比 WasPressed 的瞬时沿更适合做连续调节）
            int pwStep = step(gamepad1.dpad_down, gamepad1.dpad_up, repeatPeriodMs);
            if (pwStep != 0) {
                // 参数保护：maxPower / powerStep 被误设为 0、负数或 NaN 时回退默认值，
                // 否则 targetPower 永远不变，表现为"手柄控制电机没有反应"
                double limit = (maxPower > 0.0 && maxPower <= 1.0) ? maxPower : 1.0;
                double delta = (powerStep > 0.0 && powerStep <= 1.0) ? powerStep : 0.05;
                targetPower = clamp(targetPower + pwStep * delta, -limit, limit);
            }

            // 方向键上升沿步进舵机目标位置
            int pStep = 0;
            if (gamepad1.dpadRightWasPressed()) pStep += 1;
            if (gamepad1.dpadLeftWasPressed()) pStep -= 1;
            if (pStep != 0) {
                targetPosition = clamp(targetPosition + pStep * positionStep,
                        clamp01(minPosition), clamp01(maxPosition));
            }

            // Y 键急停
            if (gamepad1.yWasPressed()) {
                emergencyStop();
            }

            runControl();
            setTelemetry();
            telemetry.update();
        }

        // 退出时停机
        motor.setPower(0);
    }

    // ==================== 控制 ====================

    /** Y 键急停：目标功率归零并使电机停转 */
    private void emergencyStop() {
        targetPower = 0;
        motor.setPower(0);
    }

    /** 每帧下发：电机直通功率，舵机直通位置 */
    private void runControl() {
        motor.setPower(targetPower);
        servo.setPosition(targetPosition);
    }

    // ==================== 遥测 ====================

    private void setTelemetry() {
        // ---- 电机：目标功率 vs 编码器实测转速 ----
        telemetry.addData("Dpad U/D", "%d / %d", gamepad1.dpad_up ? 1 : 0, gamepad1.dpad_down ? 1 : 0);
        telemetry.addData("Target power", "%.3f", targetPower);
        telemetry.addData("Power", "%.4f", motor.getPower());
        telemetry.addData("Velocity (tick/s)", "%.0f", motor.getVelocity());
        telemetry.addData("Encoder position (tick)", motor.getCurrentPosition());

        // ---- 舵机 ----
        telemetry.addData("Target position", "%.3f", targetPosition);
        telemetry.addData("Servo position", "%.3f", servo.getPosition());
        telemetry.addData("Range", "%.2f ~ %.2f", minPosition, maxPosition);

        telemetry.addData("Battery (V)", "%.2f", getBatteryVoltage());
    }

    /**
     * 方向键步进：新按下时立即走一步，按住时每 periodMs 再走一步。
     *
     * @param negative 负向键当前是否按住
     * @param positive 正向键当前是否按住
     * @param periodMs 长按连续步进周期 (ms)
     * @return 本帧应施加的步进方向 (-1 / 0 / +1)
     */
    private int step(boolean negative, boolean positive, int periodMs) {
        int dir = (positive ? 1 : 0) - (negative ? 1 : 0);
        if (dir == 0) {
            lastStepDir = 0;
            return 0;
        }
        long now = System.currentTimeMillis();
        if (dir != lastStepDir || now >= nextStepTime) {
            lastStepDir = dir;
            nextStepTime = now + Math.max(20, periodMs);
            return dir;
        }
        return 0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** 把数值限制到 [0, 1] */
    private static double clamp01(double v) {
        return clamp(v, 0.0, 1.0);
    }

    /** 取各 Hub 电压最小值（近似电池电压） */
    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            min = Math.min(min, sensor.getVoltage());
        }
        return min;
    }
}
