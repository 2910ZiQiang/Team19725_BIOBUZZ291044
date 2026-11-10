package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * 磁传感器测试程序（数字接口）。
 *
 * 每帧读取一次 DigitalChannel 的电平，实时返回是否检测到磁：
 *   Detected   1 = 检测到磁，0 = 未检测到
 *   Raw        原始数字电平（1 = 高电平，0 = 低电平）
 *   Polarity   检测到磁时对应的原始电平（detectedOnState，Dashboard 可调）
 *
 * 使用前：在 Robot Configuration 中把磁传感器配置为 Digital Device，
 *        设备名与 DEVICE_NAME 保持一致（默认 "magnet"）。
 *
 * 校准：先在"有磁 / 无磁"两种状态下观察 Raw，再把 detectedOnState
 *      设为"有磁"时读到的电平（高有效 = true，低有效 = false）。
 */
@Config
@TeleOp(name = "MagneticSensorTest", group = "Tests")
public class MagneticSensorTest extends LinearOpMode {

    /** 磁传感器在 Robot Configuration 中的设备名（改动后需重新 Init） */
    private static final String DEVICE_NAME = "magnet";

    /** 检测到磁时对应的原始电平：true = 高电平有效，false = 低电平有效 */
    public static boolean detectedOnState = true;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 磁传感器作为数字输入
        DigitalChannel magnet = hardwareMap.get(DigitalChannel.class, DEVICE_NAME);
        magnet.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Device", DEVICE_NAME);
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Calibrate detectedOnState in FTC Dashboard > Config");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 数字电平：true = 高电平，false = 低电平
            boolean raw = magnet.getState();
            boolean detected = (raw == detectedOnState);

            telemetry.addData("Detected", detected ? 1 : 0);
            telemetry.addData("Raw", raw ? 1 : 0);
            telemetry.addData("Polarity (detectedOnState)", detectedOnState ? 1 : 0);
            telemetry.update();
        }
    }
}
