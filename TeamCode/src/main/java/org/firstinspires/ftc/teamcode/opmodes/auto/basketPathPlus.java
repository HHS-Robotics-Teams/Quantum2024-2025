package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.isDone;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.resetEncoders;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.armMoving;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.armUp;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.basketChamberTime;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.closedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.openPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.outtakeDuration;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarScore;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarScorePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.basketChamberScore;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.basketScorePose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.chamberBasketVector;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.chamberToSamplePickup;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.pickupTwo;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.spikemarkPickupOne;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.spikemarkPickupTwo;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.startPose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.toChamberBasket;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.CompDrive25.armGoUpHighBasket;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths;

@Autonomous
public class basketPathPlus extends OpMode {

    private boolean stepTimeGot = false;
    private double stepStartTime;
    static int currentArmStepAUTO = 0;
    private String currentAutoStep = "Path to Chamber";
    double outtakeTime;
    boolean pathInitialized = false;
    Trajectory toChamber;
    SampleMecanumDrive drive;

    @Override
    public void init() {
        RobotComponents.init(hardwareMap);
        resetEncoders();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        buildPaths(drive);
    }

    @Override
    public void start() {
        pivot_motor.setTargetPosition(pivotDownPosition);
        pivot_motor.setPower(pivotPower2);
        wrist_servo.setPosition(wristMiddlePosition);
    }

    @Override
    public void loop() {
        drive.update();

        telemetry.addData("Current Auto Step:", currentAutoStep);
        telemetry.addData("Current Arm Step:", currentArmStepAUTO);
        telemetry.addData("Arm Up Flag Status: ", armUp);

        switch (currentAutoStep) {
            case ("Path to Chamber"):
                if(!stepTimeGot){
                    getStepStartTime();
                }
                drive.followTrajectoryAsync(toChamber);
                if(!drive.isBusy() && arbitraryStepTimeElapsed()) {
                    currentAutoStep = "Score Chamber";
                }
                break;
            case("Score Chamber"):
                switch (currentArmStepAUTO){
                    case(0):
                        if(isDone(left_slide_motor, slideMargin) && isDone(pivot_motor, pivotMargin)){
                            currentArmStepAUTO++;
                        }
                    case(1):
                        pivot_motor.setTargetPosition(pivotHighBarScore);
                        if(isDone(pivot_motor, 10)){
                            intakeouttake_servo.setPosition(openPosition);
                            wrist_servo.setPosition(wristMiddlePosition);
                            currentArmStepAUTO = 0;
                            currentAutoStep = "Pickup One";
                            break;
                        }
                }
        }
    }

    private void getStepStartTime() {
        stepStartTime = getRuntime();
        stepTimeGot = true;
    }

    private boolean arbitraryStepTimeElapsed() {
        if ((getRuntime() - stepStartTime) > 0.125) {
            stepTimeGot = false;
            return true;
        } else {
            return false;
        }
    }

    private void buildPaths (SampleMecanumDrive drive) {
        toChamber = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(2, 18))
                .addDisplacementMarker(5, () -> {
                    pivot_motor.setTargetPosition(pivotHighBarTarget);
                    wrist_servo.setPosition(wristBarPosition);
                })
                .addDisplacementMarker(12, () -> {
                    left_slide_motor.setTargetPosition(slideHighBarPosition);
                    right_slide_motor.setTargetPosition(slideHighBarPosition);
                })
                .build();


    }

}