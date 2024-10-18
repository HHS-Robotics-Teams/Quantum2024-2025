package org.firstinspires.ftc.teamcode.macros.ArmMovement;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineResult;
import org.firstinspires.ftc.teamcode.macros.MacroStep;

public class DunkPlace extends MacroStep {
    public boolean dunked = false;
    public static double wristIntakePosition = .2;
    public static double intakePower = .6;
    @Override
    public void onStart() {
        if((HighBasket.extended||LowBasket.extended) && !dunked) {
            HighBasket.extended = false;
            LowBasket.extended = false;
            RobotComponents.wrist_servo.setPosition(wristIntakePosition);
            dunked = true;
        }

    }

    @Override
    public void onTick(OpMode opMode) {
        if(dunked) {
            try {
                RobotComponents.coroutines.wait(35);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            RobotComponents.intakeouttake_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            RobotComponents.intakeouttake_servo.setPower(intakePower);
            try {
                RobotComponents.coroutines.wait(35);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            RobotComponents.intakeouttake_servo.setPower(0);

            RobotComponents.coroutines.startRoutineLater((mode, d) -> {
                finish();
                return CoroutineResult.Stop;
            }, 130);
        }

    }
}
