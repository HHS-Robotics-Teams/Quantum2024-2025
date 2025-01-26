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
                    pivot_motor.setTargetPosition(950);
                })
                .addDisplacementMarker(1e-4,() -> {
                    wrist_servo.setPosition(wristMiddlePosition);
                    left_slide_motor.setTargetPosition(1100 + 200);
                    right_slide_motor.setTargetPosition(1100 + 200);
                    pivot_motor.setTargetPosition(975);
                })
                .lineToConstantHeading(new Vector2d(20, -6))
                .addDisplacementMarker(((Math.sqrt(Math.pow(20, 2) + Math.pow(6, 2))) + .5),() -> {
                    pivot_motor.setPower(pivotPower);
                    pivot_motor.setTargetPosition(850);
                })
                .addTemporalMarker(1, () -> {
                    left_slide_motor.setTargetPosition(1000);
                    right_slide_motor.setTargetPosition(1000);
                })
                .addDisplacementMarker(((Math.sqrt(Math.pow(20, 2) + Math.pow(6.000001, 2))) + 4.1), () -> {
                    intakeouttake_servo.setPosition(openPosition);
                    left_slide_motor.setTargetPosition(350);
                    right_slide_motor.setTargetPosition(350);
                })
                .waitSeconds(.25)
                .back(10)
                .addDisplacementMarker((Math.sqrt(Math.pow(25, 2) + Math.pow(6.00001, 2)) + 6), () -> {
                    pivot_motor.setPower(pivotPower);
                    wrist_servo.setPosition(wristMiddlePosition);
                    pivot_motor.setTargetPosition(700);
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                })
                .build();

        TrajectorySequence pickupOne = drive.trajectorySequenceBuilder(forwardTrajectory.end())
                .lineToLinearHeading(new Pose2d(13.25,(38 - 6.75),Math.toRadians(0)))
                .addDisplacementMarker(12,() -> {
                    intakeouttake_servo.setPosition(openPosition);
                    wrist_servo.setPosition(wristMiddlePosition);
                    pivot_motor.setTargetPosition(500);
                    left_slide_motor.setTargetPosition(slidePickupSubmersiblePosition + 150);
                    right_slide_motor.setTargetPosition(slidePickupSubmersiblePosition + 150);
                })
                .addTemporalMarker(2.25, () -> {
                    pivot_motor.setTargetPosition(310);
                })
                .addTemporalMarker(2.45, () -> {
                    intakeouttake_servo.setPosition(closedPosition);
                })
                .addTemporalMarker(3.2, () -> {
                    pivot_motor.setTargetPosition(pivotMiddleTarget - 15);
                    left_slide_motor.setTargetPosition(slideRetractedPosition + 100);
                    right_slide_motor.setTargetPosition(slideRetractedPosition + 100);
                })
                .waitSeconds(3.5)
                .lineToLinearHeading(new Pose2d(9,38, Math.toRadians(142.5)))
                .addTemporalMarker(4.5, () -> {
                    pivot_motor.setTargetPosition(pivotUpHighTarget);
                })
                .addTemporalMarker(5.5, () -> {
                    left_slide_motor.setTargetPosition(slideHighBasketPosition);
                    right_slide_motor.setTargetPosition(slideHighBasketPosition);
                })
                .addTemporalMarker(6.5, () -> {
                    pivot_motor.setTargetPosition(pivotUpHighTarget - 50);
                })
                .addTemporalMarker(7, () -> {
                    intakeouttake_servo.setPosition(openPosition);
                })
                .addTemporalMarker(7.35, () -> {
                    pivot_motor.setTargetPosition(pivotUpHighTarget + 200);
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                })
                .addTemporalMarker(9, () -> {
                    pivot_motor.setTargetPosition(0);
                    left_slide_motor.setTargetPosition(0);
                    right_slide_motor.setTargetPosition(0);
                })
                .waitSeconds(2.25)
                .lineToLinearHeading(new Pose2d(52,20,Math.toRadians(0)))
                .lineTo(new Vector2d(52, 38))
                .lineToLinearHeading(new Pose2d(9.5,38,Math.toRadians(0)))
                .lineTo(new Vector2d(52, 38))
                .lineTo(new Vector2d(52,45))
                .lineToLinearHeading(new Pose2d(12, 45, Math.toRadians(0)))
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
