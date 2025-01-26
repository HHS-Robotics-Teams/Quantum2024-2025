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
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarScore;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotLowBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarScorePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class observationSpeceminAuto extends LinearOpMode {

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
                .waitSeconds(10)
                .addTemporalMarker(9.95,() -> {
                    intakeouttake_servo.setPosition(closedPosition);
                    pivot_motor.setTargetPosition(950);
                })
                .addTemporalMarker(10.15,() -> {
                    wrist_servo.setPosition(wristMiddlePosition);
                    left_slide_motor.setTargetPosition(1100 + 200);
                    right_slide_motor.setTargetPosition(1100 + 200);
                    pivot_motor.setTargetPosition(1000);
                })
                .lineToConstantHeading(new Vector2d(20, 8))
                .addDisplacementMarker(((Math.sqrt(Math.pow(20, 2) + Math.pow(6, 2))) + .5),() -> {
                    pivot_motor.setPower(pivotPower);
                    pivot_motor.setTargetPosition(850);
                })
                .addTemporalMarker(11, () -> {
                    left_slide_motor.setTargetPosition(1000);
                    right_slide_motor.setTargetPosition(1000);
                })
                .addDisplacementMarker(((Math.sqrt(Math.pow(20, 2) + Math.pow(6.000001, 2))) + 4.1), () -> {
                    intakeouttake_servo.setPosition(openPosition);
                    left_slide_motor.setTargetPosition(350);
                    right_slide_motor.setTargetPosition(350);
                })
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    intakeouttake_servo.setPosition(wristBarPosition);
                })
                .back(10)
                .addDisplacementMarker((Math.sqrt(Math.pow(25, 2) + Math.pow(6.00001, 2)) + 6), () -> {
                    pivot_motor.setPower(pivotPower);
                    wrist_servo.setPosition(wristMiddlePosition);
                    pivot_motor.setTargetPosition(700);
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                })
                .build();

        TrajectorySequence secondTrajectory = drive.trajectorySequenceBuilder(forwardTrajectory.end())
                .lineTo(new Vector2d(5, -20))
                .lineTo(new Vector2d(50, -26))
                .lineTo(new Vector2d(50, -31))
                .lineTo(new Vector2d(5, -31))
                .lineTo(new Vector2d(50, -31))
                .lineTo(new Vector2d(50, -38))
                .lineTo(new Vector2d(5, -38))
                .lineTo(new Vector2d(50,-38))
                .lineTo(new Vector2d(50,-46))
                .lineTo(new Vector2d(5,-46))
                .addTemporalMarker(2, () -> {
                    wrist_servo.setPosition(.55);
                    pivot_motor.setTargetPosition(0);
                    left_slide_motor.setTargetPosition(0);
                    right_slide_motor.setTargetPosition(0);
                })

                .build();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        intakeouttake_servo.setPosition(closedPosition);
        wrist_servo.setPosition(wristBarPosition);

        drive.followTrajectorySequence(forwardTrajectory);
        drive.followTrajectorySequence(secondTrajectory);

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
