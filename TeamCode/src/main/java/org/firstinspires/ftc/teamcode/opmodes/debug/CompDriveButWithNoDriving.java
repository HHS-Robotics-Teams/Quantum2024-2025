package org.firstinspires.ftc.teamcode.opmodes.debug;

import static org.firstinspires.ftc.teamcode.opmodes.Constants.PIVOTPOWERDOWN;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.climbServoPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotLowBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpLowTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideLowBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideLowBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMotorPickupPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristRetractedPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.Input;
import org.firstinspires.ftc.teamcode.excutil.MotorPath;


@TeleOp(group = "A most important group", name = "Table 'Drive'")
public class CompDriveButWithNoDriving extends OpMode {
    private Input input ;
    //Set true on arm input, set false upon completion of steps
    public boolean armMoving = false;
    //Set true upon step completion, Set false upon down completion
    public boolean armUp = false;
    //Set true when extending for pickup, set false otherwise
    public boolean extendingPickupMode = false;
    //True if scoring basket, false otherwise
    public boolean basket = true;
    //True if dpadRight pressed, false upon dpadLeft press
    public boolean isExtending = false;
    public String armDirection;
    public int currentArmStep;

    MotorPath pivotMiddle;
    MotorPath pivotDown;
    MotorPath pivotUpHigh;
    MotorPath pivotUpLow;
    MotorPath extendLeftHigh;
    MotorPath extendRightHigh;
    @Override
    public void init() {
        input = new Input();
        RobotComponents.init(hardwareMap);
    }

    public void start() {
        RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
        RobotComponents.wrist_servo.setPosition(wristIntakePosition);
    }


    @Override
    public void loop() {
        RobotComponents.pivot_motor.setPower(pivotPower2/2);
        RobotComponents.right_slide_motor.setPower(slideMotorPickupPower);
        RobotComponents.left_slide_motor.setPower(slideMotorPickupPower);

        //ARM CODE
        if(input.dpad_up.down()&&!armMoving&&!extendingPickupMode&&!isExtending){
            armMoving = true;
            armDirection = "High Pole";
            currentArmStep = 0;
        }

        if(input.dpad_down.down()&&!armMoving&&!extendingPickupMode&&!isExtending){
            armMoving = true;
            armDirection = "Low Pole";
            currentArmStep = 0;
        }

        if((gamepad1.right_stick_button||gamepad1.left_stick_button)&&!armMoving&&!extendingPickupMode) {
            armMoving = true;
            armDirection = "Retract";
            currentArmStep = 0;
        }

        if(input.start.down()){
            basket = !basket;
        }

        if(armMoving){
            telemetry.addLine("ARM IS MOVING");
            telemetry.addData("ArmStep:", currentArmStep);
            if(basket){
                telemetry.addLine("BASKET MODE");
                switch(armDirection) {
                    case "High Pole":

                        telemetry.addLine("Going High Basket");

                        switch (currentArmStep){
                            case(0):
                                pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                                if(pivotMiddle.isComplete(50, 2000)){currentArmStep = 1;}
                                break;

                            case(1):
                                pivotUpHigh = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotUpHighTarget, pivotPower2);
                                if(pivotUpHigh.isComplete(50,2000)){currentArmStep = 2;}
                                break;

                            case(2):
                                extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideHighBasketPosition, extendPower);
                                extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideHighBasketPosition, extendPower);
                                if(extendLeftHigh.isComplete(50, 2000)&&extendRightHigh.isComplete(50,2000)){currentArmStep = 3;}
                                break;

                            case(3):
                                RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                                currentArmStep = 0;
                                armMoving = false;
                                armUp = true;
                                break;
                        }
                        break;

                    case "Low Pole":
                        telemetry.addLine("Going Low Basket");

                        switch (currentArmStep){
                            case(0):
                                pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                                if(pivotMiddle.isComplete(50, 2000)){currentArmStep = 1;}
                                break;

                            case(1):
                                pivotUpLow = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotUpLowTarget, pivotPower2);
                                if(pivotUpHigh.isComplete(50,2000)){currentArmStep = 2;}
                                break;

                            case(2):
                                extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideLowBasketPosition, extendPower);
                                extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideLowBasketPosition, extendPower);
                                if(extendLeftHigh.isComplete(50, 2000)&&extendRightHigh.isComplete(50,2000)){currentArmStep = 3;}
                                break;

                            case(3):
                                RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                                currentArmStep = 0;
                                armMoving = false;
                                armUp = true;
                                break;
                        }
                        break;

                    case "Retract":

                        telemetry.addLine("Going Retracting");

                        switch (currentArmStep){
                            case(0):
                                RobotComponents.intakeouttake_servo.setPower(0);
                                RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                                currentArmStep = 1;
                                break;

                            case(1):
                                extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideRetractedPosition, extendPower);
                                extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideRetractedPosition, extendPower);
                                if(extendLeftHigh.isComplete(50, 2000)&&extendRightHigh.isComplete(50,2000)){currentArmStep = 2;}
                                break;

                            case(2):
                                if(isExtending){currentArmStep=3;break;}
                                pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                                if(pivotMiddle.isComplete(50, 2000)){currentArmStep = 3;}
                                break;

                            case(3):
                                pivotDown = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotDownPosition, PIVOTPOWERDOWN);
                                currentArmStep = 0;
                                armMoving = false;
                                armUp = false;
                                break;

                        }
                        break;
                }
            }
            if(!basket){
                switch(armDirection) {
                    case "High Pole":

                        telemetry.addLine("Going High Bar");

                        switch (currentArmStep){
                            case(0):
                                pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                                if(pivotMiddle.isComplete(50, 2000)){currentArmStep = 1;}
                                break;

                            case(1):
                                pivotUpHigh = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotHighBarTarget, pivotPower2);
                                if(pivotUpHigh.isComplete(50,2000)){currentArmStep = 2;}
                                break;

                            case(2):
                                extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideHighBarPosition, extendPower);
                                extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideHighBarPosition, extendPower);
                                if(extendLeftHigh.isComplete(50, 2000)&&extendRightHigh.isComplete(50,2000)){currentArmStep = 3;}
                                break;

                            case(3):
                                RobotComponents.wrist_servo.setPosition(wristBarPosition);
                                currentArmStep = 0;
                                armMoving = false;
                                armUp = true;
                                break;
                        }
                        break;

                    case "Low Pole":
                        telemetry.addLine("Going Low Bar");

                        switch (currentArmStep){
                            case(0):
                                pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                                if(pivotMiddle.isComplete(50, 2000)){currentArmStep = 1;}
                                break;

                            case(1):
                                pivotUpLow = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotLowBarTarget, pivotPower2);
                                if(pivotUpHigh.isComplete(50,2000)){currentArmStep = 2;}
                                break;

                            case(2):
                                extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideLowBarPosition, extendPower);
                                extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideLowBarPosition, extendPower);
                                if(extendLeftHigh.isComplete(50, 2000)&&extendRightHigh.isComplete(50,2000)){currentArmStep = 3;}
                                break;

                            case(3):
                                RobotComponents.wrist_servo.setPosition(wristBarPosition);
                                currentArmStep = 0;
                                armMoving = false;
                                armUp = true;
                                break;
                        }
                        break;

                    case "Retract":

                        telemetry.addLine("Going Retracting");

                        switch (currentArmStep){
                            case(0):
                                RobotComponents.intakeouttake_servo.setPower(0);
                                RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                                currentArmStep = 1;
                                break;

                            case(1):
                                extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideRetractedPosition, extendPower);
                                extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideRetractedPosition, extendPower);
                                if(extendLeftHigh.isComplete(50, 2000)&&extendRightHigh.isComplete(50,2000)){currentArmStep = 2;}
                                break;

                            case(2):
                                if(isExtending){currentArmStep=3;break;}
                                pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                                if(pivotMiddle.isComplete(50, 2000)){currentArmStep = 3;}
                                break;

                            case(3):
                                pivotDown = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotDownPosition, PIVOTPOWERDOWN);
                                currentArmStep = 0;
                                armMoving = false;
                                armUp = false;
                                break;

                        }
                        break;
                }
            }
        }
        //END OF ARM CODE

        //IntakeOuttake CODE
        if(input.right_trigger.held()){
            RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.REVERSE);
            RobotComponents.intakeouttake_servo.setPower(intakePower);
        }
        else if(input.left_trigger.held()) {
            RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.FORWARD);
            RobotComponents.intakeouttake_servo.setPower(intakePower);
        }
        else {
            RobotComponents.intakeouttake_servo.setPower(0);
        }


        //Wrist Code
        if(input.left_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(wristIntakePosition);
        }

        if(input.right_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(wristRetractedPosition);
        }

        //EXTEND FOR PICKUP CODE
        if(input.y.held()){
            RobotComponents.pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getTargetPosition()+5);
        }
        if(input.a.held()){
            RobotComponents.pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getTargetPosition()-5);
        }
        if(input.dpad_right.held()){
            isExtending = true;
            RobotComponents.right_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getTargetPosition() + 15);
            RobotComponents.left_slide_motor.setTargetPosition(RobotComponents.left_slide_motor.getTargetPosition() + 15);
        }
        if(input.dpad_left.down()) {
            isExtending = false;
            RobotComponents.left_slide_motor.setTargetPosition(slideRetractedPosition);
            RobotComponents.right_slide_motor.setTargetPosition(slideRetractedPosition);
            RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
        }
        //END OF i absolutely hate this naming convention

        //CLIMB CODE
        if(input.b.held()) {
            RobotComponents.right_climb1_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            RobotComponents.left_climb1_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            RobotComponents.left_climb1_servo.setPower(climbServoPower);
            RobotComponents.right_climb1_servo.setPower(climbServoPower);
        }
        else if(input.x.held()) {
            RobotComponents.left_climb1_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            RobotComponents.right_climb1_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            RobotComponents.left_climb1_servo.setPower(climbServoPower);
            RobotComponents.right_climb1_servo.setPower(climbServoPower);
        }
        else {
            RobotComponents.left_climb1_servo.setPower(0);
            RobotComponents.right_climb1_servo.setPower(0);
        }
        //END OF CLIMB CODE

        input.pollGamepad(gamepad1);

        //TELEMETRY CODE
        telemetry.addLine("--------------- POSITIONS ---------------");
        telemetry.addData("Left slide motor current position:", RobotComponents.left_slide_motor.getCurrentPosition());
        telemetry.addData("Right slide motor current position:", RobotComponents.left_slide_motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Pivot motor current position:", RobotComponents.pivot_motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Wrist Position:", RobotComponents.wrist_servo.getPosition());

        telemetry.addLine("--------------- TARGETS ---------------");
        telemetry.addData("Left slide motor target position:", RobotComponents.left_slide_motor.getTargetPosition());
        telemetry.addData("Right slide motor target position:", RobotComponents.left_slide_motor.getTargetPosition());
        telemetry.addLine();
        telemetry.addData("Pivot motor target position:", RobotComponents.pivot_motor.getTargetPosition());


        telemetry.addLine("------ CONDITIONALS BELOW THIS LINE ------");
        if(extendingPickupMode) {
            telemetry.addLine("EXTENDO IN INTAKE MODE");
        }

        if(basket)   {
            telemetry.addLine("BASKET MODE");
        }

        else         {
            telemetry.addLine("SPECIMEN MODE");
        }

        if(armMoving){
            telemetry.addLine("ARM IS MOVING");
        }

        if(armUp)    {
            telemetry.addLine("ARM IS UP");
        }

        //END OF TELEMETRY
    }

}
