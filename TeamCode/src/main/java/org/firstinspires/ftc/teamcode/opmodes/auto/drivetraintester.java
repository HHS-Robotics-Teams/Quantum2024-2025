package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.leftFront;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.leftRear;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.rightFront;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.rightRear;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;

@Autonomous
public class drivetraintester extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        RobotComponents.init(hardwareMap);
        waitForStart();

        float[] wheelMultipliers = new float[] {
                1f, 1f, 1f, 1f
        };
        //power
        double p = 0.05;

        leftFront.setPower(p * wheelMultipliers[0]);
        rightFront.setPower(p * wheelMultipliers[3]);
        leftRear.setPower(p * wheelMultipliers[1]);
        rightRear.setPower(p * wheelMultipliers[2]);

        while(opModeIsActive()){
            telemetry.addData("Left Front", leftFront.getVelocity());
            telemetry.addData("Right Front", rightFront.getVelocity());
            telemetry.addData("Left Rear", leftRear.getVelocity());
            telemetry.addData("Right Rear", rightRear.getVelocity());
        }
    }
}
