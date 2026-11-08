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

    @Override
    protected Pose2d aimGoal(boolean leftTrigger) {
        // 左扳机 → Blue_Audience_Up；右扳机 → Blue_Audience_Down
        return leftTrigger ? HypParams.BlueAudienceUp : HypParams.BlueAudienceDown;
    }
}
