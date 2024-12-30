package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.isDone;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.armUp;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.outtakeDuration;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.overlyLargeNumber;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.basketChamberScore;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.basketScorePose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.chamberBasketVector;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.chamberToSamplePickup;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.spikemarkPickupOne;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.spikemarkPickupTwo;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.startPose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.toChamberBasket;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.pickupTwo;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.CompDrive25.armGoUpHighBasket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class newBasketPath extends OpMode {

    int AutoStep = 0;
    int currentArmStepAUTO = 0;
    double lastStepTime = 0;

    SampleMecanumDrive drive;

    @Override
    public void init() {
        RobotComponents.init(hardwareMap);
        RobotComponents.resetEncoders();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
    }

    @Override
    public void loop() {
        switch(AutoStep){
            case(0):
                drive.followTrajectorySequence(toChamberBasket);
                AutoStep++;
                break;
            case(1):
                armGoUpHighChamberAUTO();
                AutoStep = (int) (overlyLargeNumber);
                lastStepTime = getRuntime();
                break;
            case(2):
                drive.followTrajectorySequence(basketChamberScore);
                AutoStep = (int) (overlyLargeNumber - 1);
                lastStepTime = getRuntime();
                break;
            case(3):
                armGoUpHighBasket();
                if(armUp){
                    AutoStep++;
                }
                break;
            case((int) overlyLargeNumber):
                if(!drive.isBusy() && lastStepTime > (getRuntime() + 1)){
                    AutoStep = 2;
                }
                break;
            case((int) (overlyLargeNumber - 1)):
                if(!drive.isBusy() && lastStepTime > (getRuntime() + 1)){
                    AutoStep = 3;
                }
                break;
        }
        drive.update();
    }

    void armGoUpHighChamberAUTO() {
        switch (currentArmStepAUTO) {
            case (0):
                wrist_servo.setPosition(wristBarPosition);
                pivot_motor.setTargetPosition(pivotHighBarTarget);
                if (isDone(pivot_motor, pivotMargin)) {
                    currentArmStepAUTO++;
                }
                break;
            case (1):
                left_slide_motor.setTargetPosition(slideHighBarPosition);
                right_slide_motor.setTargetPosition(slideHighBarPosition);
                if (isDone(left_slide_motor, pivotMargin)) {
                    currentArmStepAUTO++;
                }
                break;
            case (2):
                currentArmStepAUTO = 0;
                armUp = true;
                break;
        }
    }

    void buildPaths(SampleMecanumDrive drive) {

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
