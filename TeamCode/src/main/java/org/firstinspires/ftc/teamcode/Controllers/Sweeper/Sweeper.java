package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;

@Config
/**
 * 集球器（Sweeper）控制器
 *
 * 当前实现（固件速度闭环，推荐）：
 *   使用 REV Hub 内置速度 PIDF（RUN_USING_ENCODER + setVelocity + setVelocityPIDFCoefficients），
 *   PID 在 Hub 固件内以约 1kHz 完成，延迟低、且不占用 Robot Controller 线程。
 *
 * 未来如需软件闭环（自定义控制律：变增益/前馈/抗积分饱和等），可按以下步骤改造：
 *   1. 将 motor 切到 RUN_WITHOUT_ENCODER（让 setPower 直接控制功率）。
 *   2. 使用 utility.PID 包的 PIDSVAController + SlotConfig（或 PIDController）做速度闭环：
 *        PIDSVAController ctrl = new PIDSVAController().withSlot0(
 *            new SlotConfig()
 *                .withKP(kP).withKI(kI).withKD(kD)
 *                .withKS(kS).withKV(kV).withKA(kA)
 *                .withOutputLimits(-1.0, 1.0));
 *        // 每帧：double out = ctrl.calculate(targetVel, motor.getVelocity(), dt, true);
 *        //        motor.setPower(out);
 *   3. 具体用法参见 utility/PID/Guide.md 与 MotorPIDCore.java。
 * 注意：软件闭环延迟稍高且受 RC 线程节拍影响，仅在确需自定义控制律时切换。
 */
public class Sweeper {
    /** 吸球电机（私有：所有访问都必须经过本类的保护逻辑，避免绕过满球/堵转保护） */
    private final DcMotorEx motor;
    /** 铲子升降舵机：控制 intake 前方铲子在抬起 / 放下两个已知角度间切换 */
    private final Servo shovelServo;

    private Telemetry telemetry;

    /**
     * 吸球目标速度 (tick/s，电机编码器计数/秒)。
     * 注意：必须小于该电机在当前减速比下的自由转速，否则速度环饱和、等同开环。
     */
    public static int EatVel = 700;
    /**
     * 向炮台供料的目标速度 (tick/s)。
     * 注意：必须小于该电机在当前减速比下的自由转速，实车可用遥测里的
     * Target / Actual 对比确认是否可达；不可达时应下调而非继续加大。
     */
    public static int GiveTheArtifactVel = 2000;
    /** 反向吐出球的目标速度 (tick/s，负号表示与吸球相反方向) */
    public static int OutputVel = -500;

    /** 球道容量：球道计数（RobotPosition.getBallCount()）达到该值时自动停止吸球 */
    public static int BallMax = 4;

    /** 固件速度闭环 PIDF 系数（REV Hub 内置速度 PID，可在 FTC Dashboard 实时调参） */
    public static double VelocityP = 1.17;
    public static double VelocityI = 0.117;
    public static double VelocityD = 0.0;
    public static double VelocityF = 0.0;

    /** 每帧允许的最大速度增量 (tick/s)，用于抑制吸球/供料切换阶跃带来的电流冲击 */
    public static int VelocityRampPerFrame = 150;

    /** 堵转判定电流阈值 (A)：连续超过该值即强制停机，≤0 表示关闭该保护 */
    public static double StallCurrentAmps = 6.0;

    /** 连续超过堵转阈值的帧数（50 Hz 下 10 帧 ≈ 0.2 s） */
    public static int StallFramesToStop = 10;

    private int targetVelocity = 0;
    /** 本帧实际下发给电机的速度（经过满球、堵转、斜坡三重处理后的值） */
    private int lastCommandedVelocity = 0;

    /** 斜坡中间量：当前实际下发的速度 (tick/s) */
    private int commandedVelocity = 0;

    /** 连续超过堵转阈值的帧数 */
    private int stallFrames = 0;

    /** 堵转锁存：触发后强制停机，直到上层松开（一帧目标为 0）才解除 */
    private boolean stallLatched = false;

    /** 已写入固件的速度 PIDF，用于识别 Dashboard 改值后即时重下发 */
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;

    /** 本帧目标是否为"吸球"（仅吸球受球道满容量保护，供料与吐球不受限） */
    private boolean eatCommand = false;

    /** 最近一次 update() 采到的功率，供读取与遥测复用 */
    private double cachedPower = 0;
    /** 最近一次 update() 采到的转速 (tick/s)，供读取与遥测复用 */
    private double cachedVelocity = 0;
    /** 最近一次 update() 采到的电流 (A)，供读取与遥测复用 */
    private double cachedCurrent = 0;

    public static int ForR = 0;

    /** 铲子抬起时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelUpPos = 0.5;
    /** 铲子放下时的舵机位置 [0,1]（todo: 实机标定） */
    public static double ShovelDownPos = 0.0;

    /** 当前铲子舵机目标位置（[0,1]） */
    private double targetShovelPos = ShovelDownPos;

    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        this.shovelServo = hardwareMap.get(Servo.class, "shovelServo");
        setDirection();
        // 固件速度闭环：开启编码器速度模式并配置内置速度 PIDF
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyVelocityPidf();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // 上电默认铲子抬起，等待操控/自动切换
        setShovelUp();
        shovelServo.setPosition(targetShovelPos);
    }

    private void setDirection() {
        switch(ForR) {
            case 0:
                motor.setDirection(DcMotor.Direction.REVERSE);
                break;
            case 1:
                motor.setDirection(DcMotor.Direction.FORWARD);
                break;
        }
    }

    /** 吸球：固定为正向，忽略 Dashboard 上 EatVel 被误改成负值的情况 */
    public void setEat() {
        targetVelocity = Math.abs(EatVel);
        eatCommand = true;
    }

    /** 向炮台供料：固定为正向（与吸球同向） */
    public void setGiveArtifact() {
        targetVelocity = Math.abs(GiveTheArtifactVel);
        eatCommand = false;
    }

    /** 反向吐球：固定为反向（与吸球相反） */
    public void setOutput() {
        targetVelocity = -Math.abs(OutputVel);
        eatCommand = false;
    }

    public void setStop() {
        targetVelocity = 0;
        eatCommand = false;
    }

    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
        eatCommand = false;
    }

    /** 铲子抬起（切换到抬起角度） */
    public void setShovelUp() {
        targetShovelPos = ShovelUpPos;
    }

    /** 铲子放下（切换到放下角度） */
    public void setShovelDown() {
        targetShovelPos = ShovelDownPos;
    }

    public void update() {
        // 0) Dashboard 改动 PIDF 后即时生效（仅在数值变化时写固件，不额外产生每帧开销）
        applyVelocityPidf();

        // 1) 堵转判定（用上一帧采样）：连续超阈值即锁存停机，避免"停—爬—再堵"的自激振荡
        if (StallCurrentAmps > 0 && stallFrames >= Math.max(1, StallFramesToStop)) {
            stallLatched = true;
        }
        // 上层松开（本帧没有任何转动请求）即解除锁存，下次请求重新尝试
        if (stallLatched && targetVelocity == 0) {
            stallLatched = false;
        }

        int cmd = targetVelocity;
        // 2) 球道保护：只在"吸球"（setEat）且球道已满时强制停吸。
        // 供料（setGiveArtifact）与反向吐球（setOutput）不受限，
        // 否则满球后会既吐不出球、也供不上料，形成互锁
        if (isBallLaneFull() && eatCommand) {
            cmd = 0;
        }
        // 3) 堵转锁存期间强制停机
        if (stallLatched) {
            cmd = 0;
        }
        // 4) 加速度限制：吸球 ↔ 供料 ↔ 吐球切换时避免速度阶跃造成的电流冲击
        cmd = limitAccel(cmd);

        if (cmd == 0) {
            // 零目标用功率 0（配合 FLOAT 自由滑行）：速度 0 闭环会主动刹停，
            // 既持续耗电，也会挡住由其它机构推过滚轮的球
            motor.setPower(0);
        } else {
            // 固件闭环：把目标速度下发给 Hub 内置速度 PID
            motor.setVelocity(cmd);
        }
        lastCommandedVelocity = cmd;
        targetVelocity = 0; // 每帧必须重新调用set函数，否则自动归零
        eatCommand = false;
        // 舵机保持位置：每帧刷新，保证 Dashboard 上改 ShovelUpPos/ShovelDownPos 后即时生效
        shovelServo.setPosition(targetShovelPos);
        // 本帧唯一的硬件采样点：功率 / 转速 / 电流全部缓存，getter 与遥测只读缓存
        cachedPower = motor.getPower();
        cachedVelocity = motor.getVelocity();
        cachedCurrent = motor.getCurrent(CurrentUnit.AMPS);
        // 用本帧采样更新堵转计数（保护滞后 1 帧生效，可接受；阈值为 0 时关闭）
        stallFrames = (StallCurrentAmps > 0 && Math.abs(cachedCurrent) > StallCurrentAmps)
                ? stallFrames + 1 : 0;
    }

    /**
     * 把当前速度 PIDF 写入固件；仅在参数变化时下发，
     * 因此 Dashboard 上的修改会即时生效，且正常帧没有额外开销。
     */
    private void applyVelocityPidf() {
        if (VelocityP == appliedP && VelocityI == appliedI
                && VelocityD == appliedD && VelocityF == appliedF) {
            return;
        }
        motor.setVelocityPIDFCoefficients(VelocityP, VelocityI, VelocityD, VelocityF);
        appliedP = VelocityP;
        appliedI = VelocityI;
        appliedD = VelocityD;
        appliedF = VelocityF;
    }

    /**
     * 加速度限幅：限制每帧速度变化量的绝对值（双向），
     * 因此吸球 → 供料、吸球 → 反向吐球都不会产生阶跃冲击；
     * 目标为 0（停机）时立即生效，保证 setStop() 能马上停下。
     *
     * @param target 目标速度 (tick/s)
     * @return 本帧实际下发的速度 (tick/s)
     */
    private int limitAccel(int target) {
        if (target == 0) {
            commandedVelocity = 0;
            return 0;
        }
        int ramp = Math.max(1, VelocityRampPerFrame);
        int delta = target - commandedVelocity;
        if (delta > ramp) {
            commandedVelocity += ramp;
        } else if (delta < -ramp) {
            commandedVelocity -= ramp;
        } else {
            commandedVelocity = target;
        }
        return commandedVelocity;
    }

    /**
     * 球道是否已达到容量（依据 RobotPosition 透传的球道计数）。
     * 未初始化 RobotPosition、或 BallMax 被调成 ≤0（视为关闭保护）时返回 false。
     */
    private boolean isBallLaneFull() {
        if (BallMax <= 0) {
            return false;
        }
        RobotPosition rp = RobotPosition.getInstanceOrNull();
        return rp != null && rp.getBallCount() >= BallMax;
    }

    public double getPower() {
        return cachedPower;
    }

    public double getVel() {
        return cachedVelocity;
    }

    /** @return 本帧实际下发给电机的速度 (tick/s)，即经过满球/堵转/斜坡处理后的值 */
    public int getCommandedVelocity() {
        return lastCommandedVelocity;
    }

    public int getFR() {
        return ForR;
    }

    /** @return 铲子舵机当前指令位置（[0,1]） */
    public double getShovelPosition() {
        return targetShovelPos;
    }

    public double getCurrent() {
        return cachedCurrent;
    }

    public void setTelemetry() {
        telemetry.addData("Sweeper Cmd", lastCommandedVelocity);
        telemetry.addData("Sweeper Actual", cachedVelocity);
        telemetry.addData("Sweeper Power", "%.2f", getPower());
        telemetry.addData("Sweeper Current (A)", "%.2f", getCurrent());
        telemetry.addData("Sweeper BallFull", isBallLaneFull());
        telemetry.addData("Sweeper Stall", stallLatched);
    }
}