package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotLength;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotWidth;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class observationSpeceminAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory goRight = drive.trajectoryBuilder(new Pose2d((-(robotWidth/2)),(72-(robotLength)/2)))
                .lineTo(new Vector2d((-70+(robotWidth/2)),(72-(robotLength/2)-1)))
                .build();
        waitForStart();
        if(isStopRequested())return;
        drive.followTrajectory(goRight);
    }

}
