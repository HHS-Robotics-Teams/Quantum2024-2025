package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.autoPivotIntakePos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.intakeTime;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotLength;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.robotWidth;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.teleOpWristStartPos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class basketPath extends OpMode {

    SampleMecanumDrive drive;
    Trajectory toBasket;
    TrajectorySequence pickupSequenceOne;
    TrajectorySequence pickupSequenceTwo;
    TrajectorySequence pickupSequenceThr;
    TrajectorySequence parkSequence;
    Pose2d basketPos = new Pose2d(55,55);
    Pose2d startPos = new Pose2d(24-(robotWidth/2), (72-(robotLength/2)));
    Vector2d parkPos = new Vector2d(-48+1+(robotWidth/2), 72-(robotWidth/2));
    Vector2d pickupControlPos = new Vector2d(36,48);
    Vector2d pickupPosOne = new Vector2d(48-4-(robotLength/2),24-(robotWidth/2));
    Vector2d pickupPosTwo = new Vector2d(48+10-4-(robotLength/2), 24-(robotWidth/2));
    Vector2d pickupPosThr = new Vector2d(48+10+10-4-(robotLength/2),24-(robotWidth/2));
    private boolean armDone = false;
    private boolean armUp = false;
    private boolean wristMoved = false;
    public int currentStep = 0;
    private int armStep = 0;
    public double startTime = 0;
    private double wristWaitTime;
    private double outtakeTime;
    @Override
    public void init() {
        RobotComponents.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        buildPaths();
        startTime = getRuntime();
    }

    /*@Override
    public void start() {
        RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
        RobotComponents.wrist_servo.setPosition(teleOpWristStartPos);
    }*/

    @Override
    public void loop() {
        telemetry.addData("Current Step:", currentStep);
        telemetry.addData("Current Arm Step", armStep);
        telemetry.addLine();
        if(currentStep == 0){
            RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
            RobotComponents.wrist_servo.setPosition(teleOpWristStartPos);
        }
        RobotComponents.pivot_motor.setPower(.5);
        switch (currentStep){
            case(0):
                drive.followTrajectory(toBasket);
                if(!drive.isBusy()&&getRuntime() - startTime > 50){currentStep++;}
                break;
            case(1):
                if(armDone){
                    armUp = false;
                    armDone = false;
                    currentStep++;
                    break;
                }
                if(armUp){
                    armDown();
                    break;
                }
                armUpScore();
                break;
            case(2):
                drive.followTrajectorySequence(pickupSequenceOne);
                if(!drive.isBusy()){
                    currentStep++;
                }
                break;
            case(3):
                if(armDone){
                    armUp = false;
                    armDone = false;
                    currentStep = 4;
                    break;
                }
                if(armUp){
                    armDown();
                    break;
                }
                armUpScore();
                break;
            case(4):
                drive.followTrajectorySequence(pickupSequenceTwo);
                if(!drive.isBusy()){
                    currentStep++;
                }
                break;
            case(5):
                if(armDone){
                    armUp = false;
                    armDone = false;
                    currentStep = 6;
                    break;
                }
                if(armUp){
                    armDown();
                    break;
                }
                armUpScore();
                break;
            case(6):
                drive.followTrajectorySequence(pickupSequenceThr);
                if(!drive.isBusy()){
                    currentStep++;
                }
                break;
            case(7):
                if(armDone){
                    armUp = false;
                    armDone = false;
                    currentStep = 8;
                    break;
                }
                if(armUp){
                    armDown();
                    break;
                }
                armUpScore();
                break;
            case(8):
                drive.followTrajectorySequence(parkSequence);
                if(!drive.isBusy()){currentStep++;}
                break;
            case(9):
                RobotComponents.wrist_servo.setPosition(teleOpWristStartPos);
                break;
        }
        drive.update();
    }

    public void buildPaths() {
        toBasket = drive.trajectoryBuilder(startPos,Math.toRadians(180))
                .lineToLinearHeading(basketPos)
                .build();

        pickupSequenceOne = drive.trajectorySequenceBuilder(basketPos)
                .lineToConstantHeading(pickupControlPos)
                .splineTo(pickupPosOne,90)
                .addDisplacementMarker(1.5, () -> {
                    RobotComponents.pivot_motor.setTargetPosition(autoPivotIntakePos);
                })
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    RobotComponents.intakeouttake_servo.setPower(intakePower);
                })
                .waitSeconds(intakeTime)
                .addDisplacementMarker(() -> {
                    RobotComponents.intakeouttake_servo.setPower(0);
                })
                .splineToLinearHeading(basketPos, Math.toRadians(45))
                .build();

        pickupSequenceTwo = drive.trajectorySequenceBuilder(basketPos)
                .lineToConstantHeading(pickupControlPos)
                .addTemporalMarker(2, () -> {
                    RobotComponents.pivot_motor.setTargetPosition(autoPivotIntakePos);
                })
                .splineTo(pickupPosTwo,Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    RobotComponents.intakeouttake_servo.setPower(intakePower);
                })
                .waitSeconds(intakeTime)
                .addDisplacementMarker(() -> {
                    RobotComponents.intakeouttake_servo.setPower(0);
                })
                .splineToLinearHeading(basketPos, Math.toRadians(45))
                .build();

        pickupSequenceThr = drive.trajectorySequenceBuilder(basketPos)
                .lineToConstantHeading(pickupControlPos)
                .addTemporalMarker(2, () -> {
                    RobotComponents.pivot_motor.setTargetPosition(autoPivotIntakePos);
                })
                .splineTo(pickupPosThr,Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    RobotComponents.intakeouttake_servo.setPower(intakePower);
                })
                .waitSeconds(intakeTime)
                .addDisplacementMarker(() -> {
                    RobotComponents.intakeouttake_servo.setPower(0);
                })
                .splineToLinearHeading(basketPos, Math.toRadians(45))
                .build();

        parkSequence = drive.trajectorySequenceBuilder(basketPos)
                .splineTo(new Vector2d(0,40),Math.toRadians(90))
                .splineTo(parkPos,Math.toRadians(180))
                .build();
    }

    public void armUpScore() {
        switch (armStep){
            case(0):
            RobotComponents.pivot_motor.setTargetPosition(pivotMiddleTarget);
            RobotComponents.pivot_motor.setPower(pivotPower);
            if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20){armStep = 1;}
            break;

            case(1):
                RobotComponents.pivot_motor.setTargetPosition(pivotUpHighTarget);
                RobotComponents.pivot_motor.setPower(pivotPower2);
                if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20){armStep = 2;}
                break;

            case(2):
                RobotComponents.left_slide_motor.setPower(extendPower);
                RobotComponents.right_slide_motor.setPower(extendPower);
                RobotComponents.left_slide_motor.setTargetPosition(slideHighBasketPosition);
                RobotComponents.right_slide_motor.setTargetPosition(slideHighBasketPosition);
                if(Math.abs(RobotComponents.left_slide_motor.getTargetPosition()-RobotComponents.left_slide_motor.getCurrentPosition()) < 20){armStep = 3;}
                break;

            case(3):
                RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                armStep = 4;
                wristWaitTime = getRuntime();
                break;
            case(4):
                if(getRuntime()-wristWaitTime > 100&&!wristMoved) {
                    RobotComponents.intakeouttake_servo.setDirection(DcMotorSimple.Direction.REVERSE);
                    RobotComponents.intakeouttake_servo.setPower(1);
                    outtakeTime = getRuntime();
                    wristMoved = true;
                }
                if(getRuntime()-outtakeTime > 250&&wristMoved) {
                    RobotComponents.intakeouttake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
                    RobotComponents.intakeouttake_servo.setPower(0);
                    outtakeTime = 0;
                    wristWaitTime = 0;
                    armUp = true;
                    wristMoved = false;
                }
                break;
        }
    }
    public void armDown() {
        switch (armStep){
            case(0):
                RobotComponents.intakeouttake_servo.setPower(0);
                RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                armStep = 1;
                break;

            case(1):
                RobotComponents.left_slide_motor.setPower(extendPower);
                RobotComponents.right_slide_motor.setPower(extendPower);
                RobotComponents.left_slide_motor.setTargetPosition(slideRetractedPosition);
                RobotComponents.right_slide_motor.setTargetPosition(slideRetractedPosition);
                if(Math.abs(RobotComponents.left_slide_motor.getTargetPosition()-RobotComponents.left_slide_motor.getCurrentPosition()) < 20){armStep = 2;}
                break;

            case(2):
                RobotComponents.pivot_motor.setTargetPosition(pivotMiddleTarget);
                RobotComponents.pivot_motor.setPower(pivotPower);
                if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20){armStep = 3;}
                break;

            case(3):
                RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
                RobotComponents.pivot_motor.setPower(pivotPower2);
                armStep = 0;
                armDone = true;
                armUp = false;
                break;

        }
    }
}
