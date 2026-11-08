package org.firstinspires.ftc.teamcode.OpModes.CompetitionOpmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;

/**
 * 红方比赛主程序：底盘 + 吃球 + 炮台（自动 / 手动瞄准），控制方式见
 * {@link TeleOpMainBase}。
 */
@TeleOp(name = "TeleOpRed", group = "TeleOp")
public class TeleOpRed extends TeleOpMainBase {

    public TeleOpRed() {
        super(TeamColor.RED);
    }

    @Override
    protected Pose2d defaultStopPose() {
        return HypParams.StopPoseRed;
    }

    @Override
    protected Pose2d aimGoal(boolean leftTrigger) {
        // 左扳机 → Red_Audience_Down；右扳机 → Red_Audience_Up
        return leftTrigger ? HypParams.RedAudienceDown : HypParams.RedAudienceUp;
    }
}
