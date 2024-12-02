package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.isDone;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.overlyLargeNumber;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotIdle;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideIdle;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.autoPivotIntakePos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.intakeTime;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotLength;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotWidth;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class basketPath extends OpMode {

    SampleMecanumDrive drive;
    TrajectorySequence toChamber;
    TrajectorySequence pickupSequenceOne;
    TrajectorySequence pickupSequenceTwo;
    TrajectorySequence pickupSequenceThr;
    TrajectorySequence parkSequence;
    Pose2d basketPos = new Pose2d(-72 + 21, 72 - 21);
    Pose2d chamberScorePos = new Pose2d(-12, 120);
    Pose2d startPos = new Pose2d(-24 - (robotWidth / 2), (0 - (robotLength / 2)));
    Vector2d parkPos = new Vector2d(-72 + 1 + (robotWidth / 2), -48 + 2 + (robotWidth / 2));
    Vector2d pickupControlPos = new Vector2d(-36, 48);
    Vector2d pickupPosOne = new Vector2d(-48 + 4 - (robotLength / 2), 24 - (robotWidth / 2));
    Vector2d pickupPosTwo = new Vector2d(-48 + 4 - (robotLength / 2), 24 - (robotWidth / 2));
    Vector2d pickupPosThr = new Vector2d(-48 + 4 - (robotLength / 2), 24 - (robotWidth / 2));
    private boolean armDone = false;
    // tells program whether arm is going up or down
    private boolean armUp = false;
    private boolean wristMoved = false;
    public double currentStep = 0;
    private int armStep = 0;
    public double time = 0;
    private double wristWaitTime;
    private double outtakeTime;

    @Override
    public void init() {
        time = getRuntime();
        RobotComponents.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
    }
    @Override
    public void init_loop() {

        if (isDone(pivot_motor, 100)) {
            currentStep = 42000;
        }

        if (Math.abs(time - getRuntime()) > 5) {
            telemetry.clearAll();
            telemetry.addLine("Init failed");
            telemetry.update();
            requestOpModeStop();
        }
        telemetry.addLine("Robot is starting up");
        telemetry.update();

        pivot_motor.setTargetPosition(pivotDownPosition);
        pivot_motor.setPower(pivotPower);

        buildPaths();
        if(currentStep == 42000){
            telemetry.clearAll();
            telemetry.addLine("Robot has started up!!!");
            time = getRuntime();

            pivot_motor.setPower(pivotPower2);

            right_slide_motor.setPower(slideIdle);
            left_slide_motor.setPower(slideIdle);

            left_slide_motor.setTargetPosition(slideRetractedPosition);
            right_slide_motor.setTargetPosition(slideRetractedPosition);

            wrist_servo.setPosition(wristMiddlePosition);
        }

        }



    public void start() {
        telemetry.clearAll();
        time = getRuntime();
        currentStep=1;
    }

    public void loop() {
        updateThings();
        drive.update();

        if (currentStep == overlyLargeNumber) {
            telemetry.addLine("Auto is done!!!");
            requestOpModeStop();
        }

        switch ((int) currentStep ) {
            case(1):
                drive.followTrajectorySequence(toChamber);
                if(!drive.isBusy()){
                    currentStep++;
                    break;
                }
            case(2):
                chamberScore();
                if(armDone) {
                    armDone = false;
                    currentStep++;
                    currentStep=overlyLargeNumber;
                    break;
                }
        }

    }



        private void chamberScore () {
            switch (armStep){
                case(0):
                    pivot_motor.setTargetPosition(pivotHighBarTarget);

                    if(isDone(pivot_motor, 150)){
                        armStep++;
                        break;
                    }

                case(1):
                left_slide_motor.setTargetPosition(slideHighBarPosition);
                right_slide_motor.setTargetPosition(slideHighBarPosition);

                wrist_servo.setPosition(wristBarPosition);

                if(isDone(left_slide_motor, 150)){
                    armStep++;
                    time = getRuntime();
                    break;
                }

                case(2):
                drive.setMotorPowers(-1,-1,-1,-1);
                if(Math.abs(time-getRuntime()) > .5) {
                    drive.setMotorPowers(0,0,0,0);
                    currentStep = overlyLargeNumber;
                    break;
                }
            }
        }


        public void updateThings () {
            if (drive.isBusy()) {
                drive.update();
            }
            pivot_motor.setPower(pivotIdle);
            left_slide_motor.setPower(slideIdle);
            right_slide_motor.setPower(slideIdle);

            telemetry.addData("Current step:", currentStep);
            telemetry.addData("Current Arm Step:", armStep);
            telemetry.addData("Time since last step:", (getRuntime() - time));

            telemetry.addLine("-------- Debug Values --------");
            telemetry.addData("Pivot position:", pivot_motor.getCurrentPosition());
            telemetry.addData("Extendo position:", left_slide_motor.getCurrentPosition());
            telemetry.addData("Drive Error:", drive.getLastError());

            telemetry.addLine("-------- Flag States --------");

            telemetry.update();
        }
        public void buildPaths () {

            toChamber = drive.trajectorySequenceBuilder(startPos)
                    .lineToLinearHeading(chamberScorePos)
                    .build();

            pickupSequenceOne = drive.trajectorySequenceBuilder(chamberScorePos)
                    .back(20)
                    .strafeLeft(10)
                    .lineToConstantHeading(pickupControlPos)
                    .splineTo(pickupPosOne, 90)
                    .addDisplacementMarker(1.5, () -> {
                        pivot_motor.setTargetPosition(autoPivotIntakePos);
                    })
                    .waitSeconds(.25)
                    .addDisplacementMarker(() -> {
                        intakeouttake_servo.setPower(intakePower);
                    })
                    .waitSeconds(intakeTime)
                    .addDisplacementMarker(() -> {
                        intakeouttake_servo.setPower(0);
                    })
                    .splineToLinearHeading(basketPos, Math.toRadians(45))
                    .build();

            pickupSequenceTwo = drive.trajectorySequenceBuilder(basketPos)
                    .lineToConstantHeading(pickupControlPos)
                    .addTemporalMarker(2, () -> {
                        pivot_motor.setTargetPosition(autoPivotIntakePos);
                    })
                    .splineTo(pickupPosTwo, Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        intakeouttake_servo.setPower(intakePower);
                    })
                    .waitSeconds(intakeTime)
                    .addDisplacementMarker(() -> {
                        intakeouttake_servo.setPower(0);
                    })
                    .splineToLinearHeading(basketPos, Math.toRadians(45))
                    .build();

            pickupSequenceThr = drive.trajectorySequenceBuilder(basketPos)
                    .lineToConstantHeading(pickupControlPos)
                    .addTemporalMarker(2, () -> {
                        pivot_motor.setTargetPosition(autoPivotIntakePos);
                    })
                    .splineTo(pickupPosThr, Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        intakeouttake_servo.setPower(intakePower);
                    })
                    .waitSeconds(intakeTime)
                    .addDisplacementMarker(() -> {
                        intakeouttake_servo.setPower(0);
                    })
                    .splineToLinearHeading(basketPos, Math.toRadians(45))
                    .build();

            parkSequence = drive.trajectorySequenceBuilder(basketPos)
                    .splineTo(new Vector2d(0, 40), Math.toRadians(90))
                    .splineTo(parkPos, Math.toRadians(180))
                    .build();
        }
        public void armUpScore () {
            drive.setMotorPowers(0, 0, 0, 0);
            if (!armUp) {
                switch (armStep) {
                    case (1):
                        pivot_motor.setTargetPosition(pivotUpHighTarget);
                        pivot_motor.setPower(pivotPower2);
                        if (RobotComponents.isDone(pivot_motor, pivotMargin)) {
                            armStep++;
                        }
                        break;

                    case (2):
                        left_slide_motor.setPower(extendPower);
                        right_slide_motor.setPower(extendPower);
                        left_slide_motor.setTargetPosition(slideHighBasketPosition);
                        right_slide_motor.setTargetPosition(slideHighBasketPosition);
                        wrist_servo.setPosition(wristMiddlePosition);
                        if (RobotComponents.isDone(left_slide_motor, slideMargin)) {
                            armStep = 0;
                            armUp = true;
                        }
                        break;
                }
            }
            if (armUp) {

                drive.setMotorPowers(0, 0, 0, 0);
                switch (armStep) {
                    case (0):
                        wrist_servo.setPosition(wristMiddlePosition);
                        left_slide_motor.setTargetPosition(slideRetractedPosition);
                        right_slide_motor.setTargetPosition(slideRetractedPosition);
                        left_slide_motor.setPower(extendPower);
                        right_slide_motor.setPower(extendPower);
                        if (isDone(left_slide_motor, slideMargin)) {
                            armStep++;
                        }
                        break;
                    case (1):
                        pivot_motor.setTargetPosition(pivotDownPosition);
                        pivot_motor.setPower(pivotPower2);
                        if (isDone(pivot_motor, pivotMargin * 2)) {
                            armStep = 0;
                            armUp = false;
                            armDone = true;
                        }
                        break;
                }

            }
        }
}
