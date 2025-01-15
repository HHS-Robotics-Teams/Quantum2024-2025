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
                .addDisplacementMarker(1,() -> {
                    left_slide_motor.setTargetPosition(slideHighBarPosition);
                    right_slide_motor.setTargetPosition(slideHighBarPosition);
                    pivot_motor.setTargetPosition(pivotHighBarTarget);
                })
                .lineToConstantHeading(new Vector2d(20, 8))
                .addDisplacementMarker(() -> {
                    wrist_servo.setPosition(wristMiddlePosition);
                })
                .waitSeconds(1)
                .back(1)
                .addDisplacementMarker(() -> {
                    pivot_motor.setTargetPosition(pivotLowBarTarget);
                })
                .waitSeconds(.5)
                .back(10)
                .addDisplacementMarker(Math.sqrt(Math.pow(19.5, 2) + Math.pow((8 + 1e-2) + 8, 2)),() -> {
                    pivot_motor.setTargetPosition(pivotDownPosition);
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                    wrist_servo.setPosition(wristMiddlePosition);
                    intakeouttake_servo.setPosition(openPosition);
                })
                .lineToLinearHeading(new Pose2d(5,-40,Math.toRadians(0)))
                .build();


        waitForStart();

        if (isStopRequested()) {
            return;
        }
        intakeouttake_servo.setPosition(closedPosition);
        pivot_motor.setTargetPosition(pivotHighBarTarget);
        wrist_servo.setPosition(wristBarPosition);

        drive.followTrajectorySequence(forwardTrajectory);

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
