package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
    public DcMotorEx motor;
    /** 铲子升降舵机：控制 intake 前方铲子在抬起 / 放下两个已知角度间切换 */
    public Servo shovelServo;

    private Telemetry telemetry;

    public static int EatVel = 700;
    public static int GiveTheArtifactVel = 2000;
    public static int OutputVel = -500;

    /** 固件速度闭环 PIDF 系数（REV Hub 内置速度 PID，可在 FTC Dashboard 实时调参） */
    public static double VelocityP = 1.17;
    public static double VelocityI = 0.117;
    public static double VelocityD = 0.0;
    public static double VelocityF = 0.0;

    private int targetVelocity = 0;
    private int lastTargetVelocity = 0;

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
        motor.setVelocityPIDFCoefficients(VelocityP, VelocityI, VelocityD, VelocityF);
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

    public void setEat() {
        targetVelocity = EatVel;
    }

    public void setGiveArtifact() {
        targetVelocity = GiveTheArtifactVel;
    }

    public void setOutput() {
        targetVelocity = OutputVel;
    }

    public void setStop() {
        targetVelocity = 0;
    }

    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
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
        // 固件闭环：把目标速度下发给 Hub 内置速度 PID
        motor.setVelocity(targetVelocity);
        lastTargetVelocity = targetVelocity;
        targetVelocity = 0; // 每帧必须重新调用set函数，否则自动归零
        // 舵机保持位置：每帧刷新，保证 Dashboard 上改 ShovelUpPos/ShovelDownPos 后即时生效
        shovelServo.setPosition(targetShovelPos);
        // 本帧唯一的硬件采样点：功率 / 转速 / 电流全部缓存，getter 与遥测只读缓存
        cachedPower = motor.getPower();
        cachedVelocity = motor.getVelocity();
        cachedCurrent = motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getPower() {
        return cachedPower;
    }

    public double getVel() {
        return cachedVelocity;
    }

    public int getTargetVelocity() {
        return lastTargetVelocity;
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
        /*
        telemetry.addData("Sweeper Velocity", getVel());
        telemetry.addData("Sweeper Power*1000", getPower()*1000);
        telemetry.addData("Sweeper Current", getCurrent());

         */
    }
}