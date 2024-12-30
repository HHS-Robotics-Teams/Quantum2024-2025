package org.firstinspires.ftc.teamcode.opmodes.auto.autoutil;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.outtakeDuration;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class paths {
    public static Pose2d startPose = new Pose2d(0,0,Math.toRadians(90));

    public static Vector2d chamberBasketVector = new Vector2d(4,30);

    static Pose2d spikemarkPickupOne = new Pose2d(-24, 36);
    static Pose2d spikemarkPickupTwo = new Pose2d(-34, 36);
    static Pose2d spikemarkPickupThree = new Pose2d(-44, 36);
    static Pose2d basketScorePose = new Pose2d(-36, 12);

    public static TrajectorySequence toChamberBasket;
    public static TrajectorySequence basketChamberScore;
    public static TrajectorySequence chamberToSamplePickup;
    public static TrajectorySequence pickupTwo;

    public static void buildPaths(SampleMecanumDrive drive) {

        toChamberBasket = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(chamberBasketVector)
                .build();

        basketChamberScore = drive.trajectorySequenceBuilder(toChamberBasket.end())
                .back(5)
                .build();

        chamberToSamplePickup = drive.trajectorySequenceBuilder(basketChamberScore.end())
                .addDisplacementMarker(20, () -> {
                    pivot_motor.setTargetPosition(pivotIntakePosition);
                    left_slide_motor.setTargetPosition(slideIntakePosition);
                    right_slide_motor.setTargetPosition(slideIntakePosition);
                    wrist_servo.setPosition(wristMiddlePosition);
                })
                .back(15)
                .splineToLinearHeading(spikemarkPickupOne, Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    intakeouttake_servo.setDirection(DcMotorSimple.Direction.REVERSE);
                    intakeouttake_servo.setPower(1);
                })
                .waitSeconds(outtakeDuration)
                .addDisplacementMarker(() -> {
                    intakeouttake_servo.setPower(0);
                    intakeouttake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
                })
                .splineToLinearHeading(basketScorePose, Math.toRadians(-45))
                .build();

        pickupTwo = drive.trajectorySequenceBuilder(chamberToSamplePickup.end())
                .addDisplacementMarker(10, () -> {
                    pivot_motor.setTargetPosition(pivotIntakePosition);
                    left_slide_motor.setTargetPosition(slideIntakePosition);
                    right_slide_motor.setTargetPosition(slideIntakePosition);
                    wrist_servo.setPosition(wristMiddlePosition);
                })
                .splineToLinearHeading(spikemarkPickupTwo, Math.toRadians(0))
                .addDisplacementMarker( () -> {
                    intakeouttake_servo.setDirection(DcMotorSimple.Direction.REVERSE);
                    intakeouttake_servo.setPower(1);
                })
                .waitSeconds(outtakeDuration)
                .addDisplacementMarker(() -> {
                    intakeouttake_servo.setPower(0);
                })
                .splineToLinearHeading(basketScorePose, Math.toRadians(-45))
                .build();

    }
}
