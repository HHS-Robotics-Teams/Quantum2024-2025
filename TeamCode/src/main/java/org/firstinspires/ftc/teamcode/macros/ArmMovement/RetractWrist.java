package org.firstinspires.ftc.teamcode.macros.ArmMovement;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineResult;
import org.firstinspires.ftc.teamcode.macros.MacroStep;

public class RetractWrist extends MacroStep {
    public boolean wristRetracted = false;
    public static double wristRetractedPosition = .8;
    @Override
    public void onStart() {
        RobotComponents.wrist_servo.setPosition(wristRetractedPosition);
        wristRetracted = true;
    }

    @Override
    public void onTick(OpMode opMode) {
        if(wristRetracted) {
            wristRetracted = false;
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
