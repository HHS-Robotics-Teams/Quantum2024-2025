package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.redObservationPark;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.redObservationStartPose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotHeight;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotWidth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

@Autonomous
public class RedObservationSideParkAuto extends OpMode {

    private Follower follower;
    private Path park = new Path(new BezierCurve(redObservationStartPose,redObservationPark));

    @Override
    public void init() {
        RobotComponents.init(hardwareMap);
        follower = new Follower(hardwareMap);
        park.setConstantHeadingInterpolation(Math.toRadians(180));
        follower.setStartingPose(new Pose(144-(robotWidth/2),(144-48-(robotHeight/2)),Math.toRadians(180)));
    }

    @Override
    public void loop() {
        follower.followPath(park);
        follower.update();
    }
}
