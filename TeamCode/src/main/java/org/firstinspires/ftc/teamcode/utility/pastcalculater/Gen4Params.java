package org.firstinspires.ftc.teamcode.utility.pastcalculater;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelCalculator;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams;

/**
 * 第 4 代（现行控制律）的参数容器 —— 对比实验专用副本。
 *
 * <p>{@link FlywheelParams} 的字段是<b>实例</b>字段（为了同时存 BIG / SMALL 两套），
 * Dashboard 只能改 {@code static} 字段，所以它本身改不动。
 * 本类把同一组量镜像成 {@code static}，于是第 4 代也能在 Dashboard 上调参。
 *
 * <p>默认值与 {@link FlywheelParams#BIG} 完全一致；若改了 {@code FlywheelParams.BIG}
 * 想同步回来，点一次 {@link #syncFromBig()}（或自己抄）。正式机器人控制仍走
 * {@code PrototypeShooter + FlywheelParams.BIG}，**本类只影响对比实验**。
 */
@Config
public class Gen4Params {

    public static double kS = 0.02945;
    public static double kff = 3.8307e-4;
    public static double kp = 1.9095e-3;
    public static double ki = 2.59224e-3;
    public static double maxI = 0.3;

    public static double dutyMin = 0.0;
    public static double dutyMax = 1.0;

    public static double quantStep = 20.0;
    public static double settleFrac = 0.02;
    public static int settleFrames = 10;

    public static double tauVMax = 0.055;
    public static double tauVMin = 0.012;
    public static double eLoFrac = 0.50;
    public static double eHiFrac = 1.50;
    // dropEnterFrac 已随 inDrop 状态机删除（2026-11-06，见 FlywheelCalculator 类注释「取舍记录」）
    public static double iFreezeTimeoutSec = 1.5;

    public static boolean useVoltageNorm = true;
    public static double vBusNominal = 12.0;
    public static double vScaleMin = 0.85;
    public static double vScaleMax = 1.25;
    public static double tauBus = 1.0;

    /** 把 {@link FlywheelParams#BIG} 的值抄进本容器（单向：BIG → 这里）。 */
    public static void syncFromBig() {
        FlywheelParams p = FlywheelParams.BIG;
        kS = p.kS;
        kff = p.kff;
        kp = p.kp;
        ki = p.ki;
        maxI = p.maxI;
        dutyMin = p.dutyMin;
        dutyMax = p.dutyMax;
        quantStep = p.quantStep;
        settleFrac = p.settleFrac;
        settleFrames = p.settleFrames;
        tauVMax = p.tauVMax;
        tauVMin = p.tauVMin;
        eLoFrac = p.eLoFrac;
        eHiFrac = p.eHiFrac;
        iFreezeTimeoutSec = p.iFreezeTimeoutSec;
        useVoltageNorm = p.useVoltageNorm;
        vBusNominal = p.vBusNominal;
        vScaleMin = p.vScaleMin;
        vScaleMax = p.vScaleMax;
        tauBus = p.tauBus;
    }

    /** 把本容器的值灌进一个 {@link FlywheelParams} 实例（每帧调用，Dashboard 改动即时生效）。 */
    public static void applyTo(FlywheelParams p) {
        p.kS = kS;
        p.kff = kff;
        p.kp = kp;
        p.ki = ki;
        p.maxI = maxI;
        p.dutyMin = dutyMin;
        p.dutyMax = dutyMax;
        p.quantStep = quantStep;
        p.settleFrac = settleFrac;
        p.settleFrames = settleFrames;
        p.tauVMax = tauVMax;
        p.tauVMin = tauVMin;
        p.eLoFrac = eLoFrac;
        p.eHiFrac = eHiFrac;
        p.iFreezeTimeoutSec = iFreezeTimeoutSec;
        p.useVoltageNorm = useVoltageNorm;
        p.vBusNominal = vBusNominal;
        p.vScaleMin = vScaleMin;
        p.vScaleMax = vScaleMax;
        p.tauBus = tauBus;
        p.motorReversed = FlywheelParams.BIG.motorReversed;
        p.speedMax = FlywheelParams.BIG.speedMax;
    }

    /** 灌进一个已 applyTo 过的计算器（连带把实例参数写进计算器字段）。 */
    public static void applyTo(FlywheelCalculator calc) {
        FlywheelParams p = FlywheelParams.copyOf(FlywheelParams.BIG);
        applyTo(p);
        p.applyTo(calc);
    }

    /** 一行摘要，写进 CSV 注释。 */
    public static String summary() {
        return String.format(java.util.Locale.US,
                "kS=%.6g kff=%.6g kp=%.6g ki=%.6g iZone=- tauV=%.3f/%.3f "
                        + "iFreeze=%.2f vNorm=%b vBusNom=%.2f tauBus=%.2f",
                kS, kff, kp, ki, tauVMax, tauVMin,
                iFreezeTimeoutSec, useVoltageNorm, vBusNominal, tauBus);
    }
}
