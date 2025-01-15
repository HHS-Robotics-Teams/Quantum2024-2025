package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.resetEncoders;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.closedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.openPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarScore;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotLowBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPickupSubmersiblePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarScorePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slidePickupSubmersiblePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class basketPath extends LinearOpMode {


    /* REFERENCE
                +X

    +Y
    +R

               bot
     */


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
                .addDisplacementMarker(() -> {
                    intakeouttake_servo.setPosition(closedPosition);
                    pivot_motor.setTargetPosition(pivotHighBarTarget);
                })
                .addDisplacementMarker(1e-7,() -> {
                    wrist_servo.setPosition(wristBarPosition);
                    left_slide_motor.setTargetPosition(slideHighBarPosition);
                    right_slide_motor.setTargetPosition(slideHighBarPosition);
                    pivot_motor.setTargetPosition(pivotHighBarTarget);
                })
                .lineToConstantHeading(new Vector2d(20, -8))
                .addDisplacementMarker((Math.sqrt(Math.pow(18.5, 2) + Math.pow(8, 2))),() -> {
                    pivot_motor.setPower(pivotPower / 2);
                    pivot_motor.setTargetPosition(pivotLowBarTarget);
                })
                .waitSeconds(1.25)
                .back(10)
                .addDisplacementMarker((Math.sqrt(Math.pow(18.5, 2) + Math.pow(8, 2)) + 4), () -> {
                    pivot_motor.setPower(pivotPower);
                    pivot_motor.setTargetPosition(pivotDownPosition);
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                    wrist_servo.setPosition(wristMiddlePosition);
                    intakeouttake_servo.setPosition(openPosition);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence pickupOne = drive.trajectorySequenceBuilder(forwardTrajectory.end())
                .lineToLinearHeading(new Pose2d(13.25,(38 - 6.75),Math.toRadians(0)))
                .addDisplacementMarker(12,() -> {
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
                .addTemporalMarker(4.5, () -> {
                    pivot_motor.setTargetPosition(pivotUpHighTarget);
                })
                .addTemporalMarker(5, () -> {
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
