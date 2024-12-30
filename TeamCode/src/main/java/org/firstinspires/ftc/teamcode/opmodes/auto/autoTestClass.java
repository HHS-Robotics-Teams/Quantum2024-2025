package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.resetEncoders;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.basketChamberScore;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.chamberBasketVector;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.startPose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.toChamberBasket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class autoTestClass extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive;
        double outtakeTime;

            RobotComponents.init(hardwareMap);
            resetEncoders();
            drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(startPose);
            paths.buildPaths(drive);
            pivot_motor.setTargetPosition(pivotDownPosition);

            TrajectorySequence testSec = drive.trajectorySequenceBuilder(startPose)
                            .splineToConstantHeading(chamberBasketVector,Math.toRadians(90))
                            .build();

            waitForStart();

            drive.followTrajectorySequenceAsync(testSec);
            while(opModeIsActive() &&!isStopRequested()){
                drive.update();
            }
    }
}
