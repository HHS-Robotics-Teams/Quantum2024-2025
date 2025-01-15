package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.resetEncoders;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.closedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.openPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotLowBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPickupSubmersiblePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slidePickupSubmersiblePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.basketChamberScore;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.chamberBasketVector;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.startPose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.toChamberBasket;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

        RobotComponents.init(hardwareMap);
        resetEncoders();
        left_slide_motor.setPower(extendPower);
        right_slide_motor.setPower(extendPower);
        pivot_motor.setPower(pivotPower);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);



        TrajectorySequence forwardTrajectory = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13.25,(38 - 6.75),Math.toRadians(0)))
                .addTemporalMarker(1e-6,() -> {
                    intakeouttake_servo.setPosition(openPosition);
                    wrist_servo.setPosition(wristMiddlePosition);
                    pivot_motor.setTargetPosition(375);
                    left_slide_motor.setTargetPosition(slidePickupSubmersiblePosition);
                    right_slide_motor.setTargetPosition(slidePickupSubmersiblePosition);
                })
                .addTemporalMarker(2.25, () -> {
                    pivot_motor.setTargetPosition(310);
                })
                .addTemporalMarker(2.45, () -> {
                    intakeouttake_servo.setPosition(closedPosition);
                })
                .addTemporalMarker(3.2, () -> {
                    pivot_motor.setTargetPosition(pivotMiddleTarget - 15);
                    left_slide_motor.setTargetPosition(slideRetractedPosition + 150);
                    right_slide_motor.setTargetPosition(slideRetractedPosition + 150);
                })
                .waitSeconds(3.5)
                .lineToLinearHeading(new Pose2d(9,38, Math.toRadians(135)))
                .addTemporalMarker(4, () -> {
                    pivot_motor.setTargetPosition(pivotUpHighTarget);
                })
                .addTemporalMarker(4.5, () -> {
                    left_slide_motor.setTargetPosition(slideHighBasketPosition);
                    right_slide_motor.setTargetPosition(slideHighBasketPosition);
                })
                .addTemporalMarker(6, () -> {
                    pivot_motor.setTargetPosition(pivotUpHighTarget - 50);
                })
                .addTemporalMarker(6.5, () -> {
                    intakeouttake_servo.setPosition(openPosition);
                })
                .addTemporalMarker(6.75, () -> {
                    pivot_motor.setTargetPosition(pivotUpHighTarget + 50);
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                })
                .waitSeconds(5)
                .build();

        TrajectorySequence pickupOne = drive.trajectorySequenceBuilder(forwardTrajectory.end())
                .waitSeconds(5)
                .build();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        intakeouttake_servo.setPosition(closedPosition);
        drive.followTrajectorySequence(forwardTrajectory);
        drive.followTrajectorySequence(pickupOne);

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
