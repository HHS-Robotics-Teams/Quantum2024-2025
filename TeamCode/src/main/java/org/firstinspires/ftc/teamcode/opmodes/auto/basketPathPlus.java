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
import static org.firstinspires.ftc.teamcode.opmodes.Constants.outtakeDuration;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarScore;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarScorePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.basketChamberScore;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.chamberToSamplePickup;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.pickupTwo;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.startPose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths.toChamberBasket;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.CompDrive25.armGoUpHighBasket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.auto.autoutil.paths;

@Autonomous
public class basketPathPlus extends OpMode {

    static String currentAutoStep = "Path to Basket";
    static int currentArmStepAUTO = 0;
    double outtakeTime;
    boolean pathInitialized = false;
    SampleMecanumDrive drive;

        @Override
        public void init() {
            RobotComponents.init(hardwareMap);
            resetEncoders();
            drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(startPose);
            paths.buildPaths(drive);
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

            switch(currentAutoStep){

                case("Path to Basket"):
                    if(!armUp) {
                        armGoUpHighChamberAUTO();
                    }
                    if(!pathInitialized) {
                        drive.followTrajectorySequenceAsync(toChamberBasket);
                        pathInitialized = true;
                    }
                    if(getRuntime() >= basketChamberTime){
                        currentAutoStep = "Score Specimen";
                        armUp = false;
                        pathInitialized = false;
                    }
                    break;

                case("Score Specimen"):
                    pivot_motor.setTargetPosition(pivotHighBarScore);
                    left_slide_motor.setTargetPosition(slideHighBarScorePosition);
                    right_slide_motor.setTargetPosition(slideHighBarScorePosition);
                    drive.followTrajectorySequenceAsync(basketChamberScore);
                    if(!drive.isBusy()){
                        currentAutoStep = "Pickup One";
                    }
                    break;

                case("Pickup One"):
                    drive.followTrajectorySequenceAsync(chamberToSamplePickup);
                    if(!drive.isBusy()){
                        currentAutoStep = "Score Basket One";
                    }
                    break;

                case("Score Basket One"):
                    if(armUp) {
                        intakeouttake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
                        intakeouttake_servo.setPower(1);
                        outtakeTime = getRuntime();
                        armUp = false;
                        break;
                    }

                    if(armMoving) {
                        outtakeTime = getRuntime();
                        break;
                    }

                    if( (Math.abs(outtakeTime - getRuntime()) ) > outtakeDuration){
                        break;
                    }

                    armGoUpHighBasket();
                    armMoving = true;
                    break;

                case("Pickup Two"):
                    drive.followTrajectorySequenceAsync(pickupTwo);
                    if(!drive.isBusy()){
                        currentAutoStep = "Score Basket Two";
                    }
                    break;

                case("Score Basket Two"):
                    if(armUp) {
                        intakeouttake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
                        intakeouttake_servo.setPower(1);
                        outtakeTime = getRuntime();
                        armUp = false;
                        break;
                    }

                    if(armMoving) {
                        outtakeTime = getRuntime();
                        break;
                    }

                    if( (Math.abs(outtakeTime - getRuntime()) ) > outtakeDuration){
                        currentAutoStep = "Pickup Three";
                        break;
                    }

                    armGoUpHighBasket();
                    armMoving = true;
                    break;
            }
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
}