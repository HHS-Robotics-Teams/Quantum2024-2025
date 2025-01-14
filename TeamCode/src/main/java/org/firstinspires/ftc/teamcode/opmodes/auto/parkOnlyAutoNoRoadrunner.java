package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.leftFront;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.leftRear;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.rightFront;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.rightRear;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.closedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;

@Autonomous
public class parkOnlyAutoNoRoadrunner extends LinearOpMode {

    double startTime;
    double runTime = 2;

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

        startTime = getRuntime();

        pivot_motor.setTargetPosition(pivotDownPosition);
        pivot_motor.setPower(pivotPower2);

        intakeouttake_servo.setPosition(closedPosition);

        while(opModeIsActive()) {
            if((getRuntime() - startTime) > runTime) {
                leftFront.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);

                requestOpModeStop();
            }
        }

    }
}
