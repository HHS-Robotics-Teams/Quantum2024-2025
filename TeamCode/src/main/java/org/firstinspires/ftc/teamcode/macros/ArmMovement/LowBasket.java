package org.firstinspires.ftc.teamcode.macros.ArmMovement;

import static org.firstinspires.ftc.teamcode.macros.ArmMovement.HighBasket.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.macros.ArmMovement.HighBasket.pivotUpTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideLowBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristRetractedPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.MotorPath;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineResult;
import org.firstinspires.ftc.teamcode.macros.MacroStep;

public class LowBasket extends MacroStep {


    MotorPath middlePivot;
    MotorPath upPivot;
    MotorPath extendLeft;
    MotorPath extendRight;
    @Override
    public void onStart() {
        middlePivot = MotorPath.runToPosition(RobotComponents.pivot_motor,pivotMiddleTarget, pivotPower);
    }
    public boolean pivotedMiddle = false;
    public boolean pivotedUp = false;
    public static boolean extended = false;
    @Override
    public void onTick(OpMode CompDrive25) {
        if(!pivotedMiddle && middlePivot.isComplete(25, 2000)) {
            pivotedMiddle = true;
            upPivot = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotUpTarget, pivotPower2);
        }
        if(!pivotedUp && upPivot.isComplete(25, 1000000)) {
            pivotedUp = true;
            extendLeft = MotorPath.runToPosition(RobotComponents.left_slide_motor,slideLowBasketPosition,extendPower );
            extendRight = MotorPath.runToPosition(RobotComponents.right_slide_motor,slideLowBasketPosition,extendPower);
        }
        if(!extended && extendLeft.isComplete(25,1000000)){
            extended = true;
            RobotComponents.coroutines.startRoutineLater((mode, d) -> {
                finish();
                return CoroutineResult.Stop;
            }, 130);
        }
    }
}
