package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueObservationPark;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueObservationStartPose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous
public class BlueObservationSideParkAuto extends OpMode {

    private Follower follower;
    private PathChain park;

    @Override
    public void init() {
        RobotComponents.init(hardwareMap);
        follower = new Follower(hardwareMap);
        buildPaths();
    }


    public void start() {
        follower.followPath(park);
    }

    @Override
    public void loop() {
        follower.update();
    }


    public void buildPaths() {
        park = follower.pathBuilder()
                .addPath(new BezierLine(blueObservationStartPose, blueObservationPark))
                .setConstantHeadingInterpolation(0)
                .build();
    }
}
