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
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.autoPivotIntakePos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.intakeTime;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotLength;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotWidth;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class basketPath extends LinearOpMode {

    SampleMecanumDrive drive;
    TrajectorySequence toBasket;
    TrajectorySequence pickupSequenceOne;
    TrajectorySequence pickupSequenceTwo;
    TrajectorySequence pickupSequenceThr;
    TrajectorySequence parkSequence;
    Pose2d basketPos = new Pose2d(-72+21,72-21);
    Pose2d startPos = new Pose2d(-24-(robotWidth/2), (0-(robotLength/2)));
    Vector2d parkPos = new Vector2d(-72+1+(robotWidth/2), -48+2+(robotWidth/2));
    Vector2d pickupControlPos = new Vector2d(-36,48);
    Vector2d pickupPosOne = new Vector2d(-48+4-(robotLength/2),24-(robotWidth/2));
    Vector2d pickupPosTwo = new Vector2d(-48+4-(robotLength/2), 24-(robotWidth/2));
    Vector2d pickupPosThr = new Vector2d(-48+4-(robotLength/2),24-(robotWidth/2));
    private boolean armDone = false;
    private boolean armUp = false;
    private boolean wristMoved = false;
    public double currentStep = 0;
    private int armStep = 0;
    public double time = 0;
    private double wristWaitTime;
    private double outtakeTime;

    @Override
    public void runOpMode() throws InterruptedException {
        time = getRuntime();

        while(currentStep == 0){
            RobotComponents.init(hardwareMap);
            drive = new SampleMecanumDrive(hardwareMap);
            if(isDone(pivot_motor,100)){
                currentStep++;
                break;
            }
            if((time - getRuntime() ) > 5) {
                currentStep = -1;
                break;
            }
            telemetry.addLine("Robot is starting up");
            telemetry.update();
            pivot_motor.setTargetPosition(pivotDownPosition);
            wrist_servo.setPosition(wristMiddlePosition);
            buildPaths();
            break;
        }

        if(currentStep == -1) {
            telemetry.clearAll();
            telemetry.addLine("Init failed");
            telemetry.update();
            requestOpModeStop();
        }

        while(currentStep > 0){
            if(currentStep == overlyLargeNumber){
                telemetry.clearAll();
                telemetry.addLine("Auto is done!!!");
                requestOpModeStop();
                break;
            }

            switch ((int) currentStep){
                case(1):
                    drive.followTrajectorySequence(toBasket);
                    updateThings();
                    if(!drive.isBusy()) {
                        currentStep++;
                        break;
                    }
                case(2):
                    armUpScore();
                    if(armDone){
                        armDone = false;
                        currentStep++;
                        break;
                    }
                case(3):
                    drive.followTrajectorySequence(pickupSequenceOne);
                    updateThings();
                    if(!drive.isBusy()) {
                        currentStep++;
                        break;
                    }
                case(4):
                    armUpScore();
                    if(armDone){
                        armDone = false;
                        currentStep++;
                        break;
                    }
                case(5):
                    drive.followTrajectorySequence(pickupSequenceTwo);
                    updateThings();
                    if(!drive.isBusy()) {
                        currentStep++;
                        break;
                    }
                case(6):
                    armUpScore();
                    if(armDone){
                        armDone = false;
                        currentStep++;
                        break;
                    }
                case(7):
                    drive.followTrajectorySequence(pickupSequenceThr);
                    updateThings();
                    if(!drive.isBusy()){
                        currentStep++;
                        break;
                    }
                case(8):
                    armUpScore();
                    if(armDone){
                        armDone = false;
                        currentStep++;
                        break;
                    }
                case(9):
                    drive.followTrajectorySequence(parkSequence);
                    updateThings();
                    if(!drive.isBusy()){
                        currentStep = overlyLargeNumber;
                        break;
                    }
            }
        }

        waitForStart();
    }
    public void updateThings() {
        drive.update();
        telemetry.addData("Current step:", currentStep);
        telemetry.addData("Arm Step:", armStep);
        telemetry.addData("Pivot position:", pivot_motor.getCurrentPosition());
        telemetry.addData("Extendo position:", left_slide_motor.getCurrentPosition());
        telemetry.addData("Time since last step:", (getRuntime() - time));
        telemetry.update();
    }
    public void buildPaths() {

        toBasket = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(basketPos)
                .build();

        pickupSequenceOne = drive.trajectorySequenceBuilder(basketPos)
                .lineToConstantHeading(pickupControlPos)
                .splineTo(pickupPosOne,90)
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
                .splineTo(pickupPosTwo,Math.toRadians(90))
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
                .splineTo(pickupPosThr,Math.toRadians(90))
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
                .splineTo(new Vector2d(0,40),Math.toRadians(90))
                .splineTo(parkPos,Math.toRadians(180))
                .build();
    }
    public void armUpScore() {
        drive.setMotorPowers(0,0,0,0);
        if(!armUp) {
            switch (armStep){
                case(0):
                    pivot_motor.setTargetPosition(pivotMiddleTarget);
                    pivot_motor.setPower(pivotPower);
                    if(Math.abs(pivot_motor.getTargetPosition()- pivot_motor.getCurrentPosition()) < 20){armStep = 1;}
                    break;

                case(1):
                    pivot_motor.setTargetPosition(pivotUpHighTarget);
                    pivot_motor.setPower(pivotPower2);
                    if(Math.abs(pivot_motor.getTargetPosition()- pivot_motor.getCurrentPosition()) < 20){armStep = 2;}
                    break;

                case(2):
                    left_slide_motor.setPower(extendPower);
                    right_slide_motor.setPower(extendPower);
                    left_slide_motor.setTargetPosition(slideHighBasketPosition);
                    right_slide_motor.setTargetPosition(slideHighBasketPosition);
                    if(Math.abs(left_slide_motor.getTargetPosition()- left_slide_motor.getCurrentPosition()) < 20){armStep = 3;}
                    break;

                case(3):
                    RobotComponents.wrist_servo.setPosition(wristMiddlePosition);
                    armStep = 4;
                    wristWaitTime = getRuntime();
                    break;
                case(4):
                    if(getRuntime()-wristWaitTime > 100&&!wristMoved) {
                        intakeouttake_servo.setDirection(DcMotorSimple.Direction.REVERSE);
                        intakeouttake_servo.setPower(1);
                        outtakeTime = getRuntime();
                        wristMoved = true;
                    }
                    if(getRuntime()-outtakeTime > 250 && wristMoved) {
                        intakeouttake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
                        intakeouttake_servo.setPower(0);
                        outtakeTime = 0;
                        wristWaitTime = 0;
                        armUp = true;
                        wristMoved = false;
                    }
                    break;
            }
        }
        if(armUp) {

            drive.setMotorPowers(0,0,0,0);
            switch (armStep){
                case(0):
                    intakeouttake_servo.setPower(0);
                    RobotComponents.wrist_servo.setPosition(wristMiddlePosition);
                    armStep = 1;
                    break;

                case(1):
                    left_slide_motor.setPower(extendPower);
                    right_slide_motor.setPower(extendPower);
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                    if(Math.abs(left_slide_motor.getTargetPosition()- left_slide_motor.getCurrentPosition()) < 20){armStep = 2;}
                    break;

                case(2):
                    pivot_motor.setTargetPosition(pivotMiddleTarget);
                    pivot_motor.setPower(pivotPower);
                    if(Math.abs(pivot_motor.getTargetPosition()- pivot_motor.getCurrentPosition()) < 20){armStep = 3;}
                    break;

                case(3):
                    pivot_motor.setTargetPosition(pivotDownPosition);
                    pivot_motor.setPower(pivotPower2);
                    armStep = 0;
                    armDone = true;
                    armUp = false;
                    break;

            }

        }
    }
}
