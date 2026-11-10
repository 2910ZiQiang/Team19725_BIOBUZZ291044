package org.firstinspires.ftc.teamcode.OpModes.CompetitionOpmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;

/**
 * 蓝方比赛主程序：底盘 + 吃球 + 炮台（自动 / 手动瞄准），控制方式见
 * {@link TeleOpMainBase}。
 */
@TeleOp(name = "TeleOpBlue", group = "TeleOp")
public class TeleOpBlue extends TeleOpMainBase {

    public TeleOpBlue() {
        super(TeamColor.BLUE);
    }

    @Override
    protected Pose2d defaultStopPose() {
        return HypParams.StopPoseBlue;
    }
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- init 阶段：选择队伍颜色 ----
        telemetry.addLine("Select Team Color");
        telemetry.addLine("Press A (Blue)");
        telemetry.addLine("Press B (Red)");
        telemetry.update();

        teamColor = TeamColor.BLUE;

        // ---- 读回自动阶段结束时刻持久化的机器状态，作为手动阶段的初始位姿 ----
        // 机器人自动结束后的实际位置就是手动阶段的起点，无需人工重新摆放/重置定位；
        // 无有效记录时（未跑自动 / 自动未完成 / 手动调试）回退到默认停车位姿
        Pose2d storedPose = RobotStateStore.loadPose();
        Pose2d initPose = (storedPose != null) ? storedPose : HypParams.StopPoseBlue;

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, initPose);
        sweeper = new Sweeper(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Team Color", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
        telemetry.addData("Init Pose Source", storedPose != null ? "Stored robot state" : "Default StopPose");
        telemetry.addData("Init Pose", "%.2f, %.2f, %.1f deg",
                initPose.position.x, initPose.position.y, Math.toDegrees(initPose.heading.toDouble()));
        telemetry.addData("--- P1 Controls ---", "");
        telemetry.addData("Left Stick", "Chassis Drive");
        telemetry.addData("Right Stick X", "Chassis Rotation (disabled while aiming)");
        telemetry.addData("Left Trigger", "Aim Blue Audience Up");
        telemetry.addData("Right Trigger", "Aim Blue Audience Down");
        telemetry.addData("X", "Toggle No-Head Mode");
        telemetry.addData("A", "Reset Pose to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red") + " ResetPose");
        telemetry.addData("Left Bumper", "Sweeper Eat");
        telemetry.addData("Right Bumper", "Sweeper Output");
        telemetry.addData("--- P2 Controls ---", "");
        telemetry.update();
        telemetry.setMsTransmissionInterval(250);

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            // ======== P1 Controls ========

            // 底盘移动：左摇杆平移 + 右摇杆旋转；
            // 按下扳机进入瞄准模式（右摇杆失效，航向自动指向所选球门）
            if (gamepad1.left_trigger > 0.5) {
                // 左扳机：瞄准 Blue_Audience_Up
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, HypParams.BlueAudienceUp);
            } else if (gamepad1.right_trigger > 0.5) {
                // 右扳机：瞄准 Blue_Audience_Down
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, HypParams.BlueAudienceDown);
            } else {
                // 不瞄准：常规操作
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            // 切换无头模式
            if (gamepad1.xWasReleased()) {
                chassis.exchangeUseNoHeadMode();
            }

            // 重置定位到对应颜色的 ResetPose
            if (gamepad1.aWasReleased()) {
                Pose2d resetPose = (teamColor == TeamColor.BLUE) ?
                        HypParams.ResetPoseBlue : HypParams.ResetPoseRed;
                RobotPosition.getInstance().ResetPoseTo(resetPose);
                telemetry.addData("ResetPose", "Reset to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red"));
            }

            if (gamepad1.left_bumper) {
                sweeper.setEat();
            }

            if (gamepad1.right_bumper) {
                sweeper.setOutput();
            }


            // ======== 更新 & 遥测 ========

            // 执行动作队列（每帧一次）。Chassis 在动作运行期间屏蔽手柄输入，
            // 因此这里必须在循环内调用，否则队列非空时底盘会一直被屏蔽。
            actionRunner.update();

            // ---- 球道计数：光电门采样已在 RobotPosition.update() 内完成 ----
            // 满球停吸由 Sweeper 依据 RobotPosition.getBallCount() 判定
            sweeper.update();

            // 手动清空放在满球停吸判定之后，保持「计数 → 停吸 → 手动清空」的帧内顺序
            if (gamepad1.bWasPressed()) {
                RobotPosition.getInstance().resetBallCount();
            }

            telemetry.addData("Team", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());

            // 位姿信息
            telemetry.addData("Pose X", "%.2f in", RobotPosition.getInstance().getX());
            telemetry.addData("Pose Y", "%.2f in", RobotPosition.getInstance().getY());
            telemetry.addData("Pose Theta", "%.2f deg", Math.toDegrees(RobotPosition.getInstance().getTheta()));

            chassis.telemetry();
            sweeper.setTelemetry();
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    @Override
    protected Pose2d aimGoal(boolean leftTrigger) {
        // 左扳机 → Blue_Audience_Up；右扳机 → Blue_Audience_Down
        return leftTrigger ? HypParams.BlueAudienceUp : HypParams.BlueAudienceDown;
    }
}
