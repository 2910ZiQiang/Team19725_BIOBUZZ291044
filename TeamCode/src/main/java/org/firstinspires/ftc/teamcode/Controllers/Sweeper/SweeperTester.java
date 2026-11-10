package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SweeperTester", group="Tests")
public class SweeperTester extends LinearOpMode {
    Sweeper sweeper;
    
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        sweeper = new Sweeper(hardwareMap, telemetry);
        sweeper.setStop();
        telemetry.addLine("Hold A: Eat | Hold B: GiveArtifact | Hold Y: Output | Hold X: Stop");
        telemetry.addLine("Dpad Up: Shovel Up | Dpad Down: Shovel Down");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
            // 必须用"电平"判定而非 *WasPressed()：Sweeper 每帧下发后会把目标归零，
            // 边沿触发只会出力一帧（约 20 ms），表现为吸球没力 / 供料不出球
            if (gamepad1.a) {
                sweeper.setEat();
            } else if (gamepad1.b) {
                sweeper.setGiveArtifact();
            } else if (gamepad1.y) {
                sweeper.setOutput();
            } else if (gamepad1.x) {
                sweeper.setStop();
            }
            
            // 铲子升降：在两个已知角度间切换
            if (gamepad1.dpadUpWasPressed()) {
                sweeper.setShovelUp();
            } else if (gamepad1.dpadDownWasPressed()) {
                sweeper.setShovelDown();
            }
            
            sweeper.update();
            sweeper.setTelemetry();
            telemetry.addData("Shovel Pos", "%.3f", sweeper.getShovelPosition());
            telemetry.update();
        }
    }
}
