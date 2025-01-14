package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.isDone;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.leftFront;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.leftRear;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.rightFront;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.rightRear;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.closedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarScore;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMotorPickupPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;

@Autonomous
public class noRoadrunnerParkSpecimen extends LinearOpMode {

    double startTime;
    double runTime = 0.75;
    int step = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotComponents.init(hardwareMap);
        RobotComponents.resetEncoders();

        waitForStart();
        if(isStopRequested()){
            requestOpModeStop();
        }

        leftFront.setPower(.5);
        rightRear.setPower(.5);
        rightFront.setPower(.5);
        leftRear.setPower(.5);

        pivot_motor.setPower(pivotPower);
        left_slide_motor.setPower(slideMotorPickupPower);
        right_slide_motor.setPower(slideMotorPickupPower);

        startTime = getRuntime();

        pivot_motor.setTargetPosition(pivotDownPosition);
        pivot_motor.setPower(pivotPower2);

        intakeouttake_servo.setPosition(closedPosition);

        while(opModeIsActive()) {
            switch(step){
                case(1):
                if((getRuntime() - startTime) > runTime) {
                    leftFront.setPower(0);
                    rightRear.setPower(0);
                    rightFront.setPower(0);
                    leftRear.setPower(0);
                    pivot_motor.setTargetPosition(pivotHighBarTarget);
                    wrist_servo.setPosition(wristBarPosition);
                    step++;
                }
                break;
                case(2):
                    if(isDone(pivot_motor, (pivotMargin * 2))){
                        left_slide_motor.setTargetPosition(slideHighBarPosition);
                        right_slide_motor.setTargetPosition(slideHighBarPosition);
                        step++;
                    }
                    break;
                case(3):
                    if(isDone(left_slide_motor, slideMargin)){
                        leftFront.setPower(-0.5);
                        rightRear.setPower(-0.5);
                        rightFront.setPower(-0.5);
                        leftRear.setPower(-0.5);
                        startTime = getRuntime();
                        pivot_motor.setTargetPosition(pivotHighBarScore);
                        left_slide_motor.setTargetPosition(slideRetractedPosition);
                        right_slide_motor.setTargetPosition(slideRetractedPosition);
                        step++;
                    }
                    break;
                case(4):
                    if((getRuntime() - startTime) > 0.75) {
                        leftFront.setPower(-0.5);
                        rightRear.setPower(0.5);
                        rightFront.setPower(-0.5);
                        leftRear.setPower(0.5);
                        pivot_motor.setTargetPosition(pivotDownPosition);
                        break;
                    }

            }

        }

    }
}
