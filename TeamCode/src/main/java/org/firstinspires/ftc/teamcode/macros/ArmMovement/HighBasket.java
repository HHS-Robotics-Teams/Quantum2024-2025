package org.firstinspires.ftc.teamcode.macros.ArmMovement;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.MotorPath;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineResult;
import org.firstinspires.ftc.teamcode.macros.MacroStep;

public class HighBasket extends MacroStep {

    public static final int pivotMiddleTarget = 500;
    public static final int pivotUpTarget = 732;
    public static double pivotPower = .8;
    public static double pivotPower2 = .6;
    public static int slideHighBasketPosition = 1230;
    public static double extendPower = .6;
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
    public void onTick(OpMode HighBasket) {
        if(!pivotedMiddle && middlePivot.isComplete(50, 2000)) {
            pivotedMiddle = true;
            upPivot = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotUpTarget, pivotPower2);
        }
        if(!pivotedUp && upPivot.isComplete(50, 2000)) {
            pivotedUp = true;
            extendLeft = MotorPath.runToPosition(RobotComponents.left_slide_motor,slideHighBasketPosition,extendPower );
            extendRight = MotorPath.runToPosition(RobotComponents.right_slide_motor,slideHighBasketPosition,extendPower);
        }
        if(!extended && extendLeft.isComplete(50,2000)){
            extended = true;
            RobotComponents.coroutines.startRoutineLater((mode, d) -> {
                finish();
                return CoroutineResult.Stop;
            }, 130);
        }
    }
}
