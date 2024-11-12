package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotWidth;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotLength;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class observationParkBoth extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory goRight = drive.trajectoryBuilder(new Pose2d(0,0))
                .strafeRight(5)
                .build();
        waitForStart();
        if(isStopRequested())return;
        drive.followTrajectory(goRight);
    }

}
