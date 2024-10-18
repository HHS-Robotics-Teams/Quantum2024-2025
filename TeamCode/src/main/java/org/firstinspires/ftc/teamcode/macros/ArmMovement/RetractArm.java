package org.firstinspires.ftc.teamcode.macros.ArmMovement;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.MotorPath;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineResult;
import org.firstinspires.ftc.teamcode.macros.MacroStep;

public class RetractArm extends MacroStep {
    public static int pivotDownPosition = 20; //done
    public static int pivotMiddlePosition = 200;
    public static int slideRetractedPosition = 0;
    public static double extendPower = .6;
    public static double pivotPower = .6;
    public static double pivotPower2 = .2;

    MotorPath downPivot;
    MotorPath extendLeft;
    MotorPath extendRight;
    @Override
    public void onStart() {
        extendLeft = MotorPath.runToPosition(RobotComponents.left_slide_motor,slideRetractedPosition,extendPower );
        extendRight = MotorPath.runToPosition(RobotComponents.right_slide_motor,slideRetractedPosition,extendPower);
    }
    public boolean retracted = false;
    @Override
    public void onTick(OpMode opMode) {
        if(!retracted && extendLeft.isComplete(50, 1000000)) {
            retracted = true;
            downPivot = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotDownPosition, pivotPower);

        }
        if(downPivot.isComplete(50, 1000000)) {
            RobotComponents.coroutines.startRoutineLater((mode, d) -> {
                finish();
                return CoroutineResult.Stop;
            }, 130);
        }
    }
}
