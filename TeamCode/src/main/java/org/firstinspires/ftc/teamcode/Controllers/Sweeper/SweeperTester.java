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
        telemetry.addLine("A: Eat | B: GiveArtifact | Y: Output | X: Stop");
        telemetry.addLine("Dpad Up: Shovel Up | Dpad Down: Shovel Down");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                sweeper.setEat();
            } else if (gamepad1.bWasPressed()) {
                sweeper.setGiveArtifact();
            } else if (gamepad1.yWasPressed()) {
                sweeper.setOutput();
            } else if (gamepad1.xWasPressed()) {
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
