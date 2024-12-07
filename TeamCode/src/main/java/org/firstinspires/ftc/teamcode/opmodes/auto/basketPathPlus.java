package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarScore;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarScorePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class basketPathPlus extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotComponents.init(hardwareMap);
        left_slide_motor.setPower(extendPower);
        right_slide_motor.setPower(extendPower);
        pivot_motor.setPower(pivotPower);

        while (opModeInInit()) {
            wrist_servo.setPosition(wristMiddlePosition);
            pivot_motor.setTargetPosition(pivotMiddleTarget);
            left_slide_motor.setTargetPosition(25);
            right_slide_motor.setTargetPosition(25);
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11, 61, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence forwardTrajectory = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.5, () -> {
                    wrist_servo.setPosition(wristBarPosition);
                    left_slide_motor.setTargetPosition(slideHighBarPosition);
                    right_slide_motor.setTargetPosition(slideHighBarPosition);
                    pivot_motor.setTargetPosition(pivotHighBarTarget);
                })
                .waitSeconds(.5)
                .forward(32)
                .strafeRight(4)
                .turn(Math.toRadians(22))
                .addDisplacementMarker(() -> {
                    pivot_motor.setTargetPosition(pivotHighBarScore);
                    left_slide_motor.setTargetPosition(slideHighBarScorePosition);
                    right_slide_motor.setTargetPosition(slideHighBarScorePosition);
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                    wrist_servo.setPosition(wristMiddlePosition);
                })

                .back(2)
                .waitSeconds(.5)
                .turn(Math.toRadians(80))
                .waitSeconds(.2)
                .forward(32)
                .waitSeconds(.2)
                .strafeRight(13)
                .addDisplacementMarker( () -> {
                    intakeouttake_servo.setPower(intakePower);
                })
                .addDisplacementMarker(.2, () -> {
                    left_slide_motor.setTargetPosition(slideIntakePosition);
                    right_slide_motor.setTargetPosition(slideIntakePosition);
                    pivot_motor.setTargetPosition(pivotIntakePosition);
                })
                .waitSeconds(2)
                .forward(4)
                .waitSeconds(.2)
                .addDisplacementMarker(() -> {
                    intakeouttake_servo.setPower(0);
                    intakeouttake_servo.setDirection(DcMotorSimple.Direction.REVERSE);
                    pivot_motor.setTargetPosition(pivotDownPosition);
                })
                .back(4)
                .strafeLeft(20)
                .forward(20)
                //end reset to start pos
                .UNSTABLE_addDisplacementMarkerOffset(4, () -> {
                    wrist_servo.setPosition(0);
                    pivot_motor.setTargetPosition(0);
                    left_slide_motor.setTargetPosition(0);
                    right_slide_motor.setTargetPosition(0);
                })
                .build();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        drive.followTrajectorySequence(forwardTrajectory);

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
