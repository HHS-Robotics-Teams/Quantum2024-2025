package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.resetEncoders;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarScore;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarScorePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

        while (opModeInInit()) {
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence forwardTrajectory = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    pivot_motor.setTargetPosition(pivotHighBarTarget);
                    telemetry.addLine("huh");
                    telemetry.addData("pivottarget", pivot_motor.getTargetPosition());
                    telemetry.update();
                })
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    wrist_servo.setPosition(wristBarPosition);
                    left_slide_motor.setTargetPosition(slideHighBarPosition);
                    right_slide_motor.setTargetPosition(slideHighBarPosition);
                })
                .turn(Math.toRadians(18))
                .forward(24)
                .turn(Math.toRadians(-10))
                .forward(4)
                .addDisplacementMarker( () -> {
                    pivot_motor.setTargetPosition(pivotHighBarScore);
                    left_slide_motor.setTargetPosition(slideHighBarScorePosition);
                    right_slide_motor.setTargetPosition(slideHighBarScorePosition);
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                })
                .back(22)
                .waitSeconds(.5)
                .turn(Math.toRadians(80))
                .back(40)
                .waitSeconds(.2)
                .turn(Math.toRadians(2))
                .strafeLeft(12)


        //end reset to start pos
                .addDisplacementMarker(() -> {
                    wrist_servo.setPosition(wristBarPosition);
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
