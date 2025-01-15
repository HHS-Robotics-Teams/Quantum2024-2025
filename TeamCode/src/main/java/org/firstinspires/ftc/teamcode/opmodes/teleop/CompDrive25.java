package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.isDone;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.leftFront;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.leftRear;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_climb1_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.rightFront;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.rightRear;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_climb1_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.armMoving;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.armUp;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.basket;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.climbServoPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.closedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendoPivotAmount;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.isExtending;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.isRetracting;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.macroTimetoTimeout;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.macrosDisabled;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.openPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.overlyLargeNumber;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pickupPivotAmount;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPickupSubmersiblePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotWallPickup;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMargin;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMaxExtensionTeleOp;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMotorPickupPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slidePickupSubmersiblePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.Input;

@Disabled
@TeleOp(name = "Competition Drive")
public class CompDrive25 extends OpMode {

    public Input input ;
    public double timeoutTime;
    //Used to determine which macro is run
    public String armDirection;
    public static int currentArmStep = 0;

    @Override
    public void init() {
        input = new Input();
        RobotComponents.init(hardwareMap);
    }

    public void start() {
        pivot_motor.setTargetPosition(pivotDownPosition);
        basket = true;
    }


    @Override
    public void loop() {

        pivot_motor.setPower(pivotPower2);
        right_slide_motor.setPower(slideMotorPickupPower);
        left_slide_motor.setPower(slideMotorPickupPower);

        //ARM CODE
        if(input.y.down()&&!armMoving){
            armMoving = true;
            armDirection = "High";
            timeoutTime = getRuntime();
        }

        if(input.a.down()&&!armMoving){
            armMoving = true;
            armDirection = "Low";
            timeoutTime = getRuntime();
        }

        if((gamepad1.right_stick_button||gamepad1.left_stick_button) || (armMoving &&((getRuntime() - timeoutTime) >= macroTimetoTimeout))) {
            armMoving = true;
            isRetracting = true;
            timeoutTime = getRuntime();
        }

        if(input.start.down()){
            basket = !basket;
            if(basket){
                timeoutTime = getRuntime();
            }
            else {
                timeoutTime = overlyLargeNumber;
            }
        }

        //macro usage toggle
        if(input.back.down()){
            timeoutTime = overlyLargeNumber;
            macrosDisabled = !macrosDisabled;
            isRetracting = true;
        }

        //IntakeOuttake code
        if(input.right_trigger.down()){
            intakeouttake_servo.setPosition(closedPosition);
        }

        if(input.left_trigger.down()) {
            intakeouttake_servo.setPosition(openPosition);
        }

        //ARM
        if(isRetracting){
            armRetract();
        }

        else if(armMoving && !macrosDisabled){

            telemetry.addLine("Arm is moving");
            telemetry.addData("Arm Step: ", currentArmStep);

                if (basket) {

                    switch (armDirection) {
                        case ("High"):
                            armGoUpHighBasket();
                            break;

                        case ("Low"):
                            armGoUpLowBasket();
                            break;

                    }

                } else {

                    switch (armDirection) {
                        case ("High"):
                            armGoUpHighChamber();
                            break;

                        case ("Low"):
                            armGoUpLowChamber();
                            break;

                    }

                }



        }

        //Wrist Code
        if(input.right_bumper.down()) {
            wrist_servo.setPosition(wristMiddlePosition);
        }

        if(input.left_bumper.down()) {
            wrist_servo.setPosition(wristBarPosition);
        }

        //EXTEND FOR PICKUP CODE

        if(Math.abs(left_slide_motor.getTargetPosition() - slideMaxExtensionTeleOp) < slideMargin) {
            left_slide_motor.setTargetPosition((int) (left_slide_motor.getCurrentPosition() - slideMargin));
        } else {
            if (input.dpad_right.held()) {
                left_slide_motor.setTargetPosition(left_slide_motor.getTargetPosition()+extendoPivotAmount);
                right_slide_motor.setTargetPosition(right_slide_motor.getTargetPosition()+extendoPivotAmount);
            }
            if (input.dpad_left.held()) {
                left_slide_motor.setTargetPosition(left_slide_motor.getTargetPosition()-extendoPivotAmount);
                right_slide_motor.setTargetPosition(right_slide_motor.getTargetPosition()-extendoPivotAmount);
            }
        }

        if(input.dpad_up.held()){
            pivot_motor.setTargetPosition(pivot_motor.getTargetPosition()+pickupPivotAmount);
        }
        if(input.dpad_down.held()){
            pivot_motor.setTargetPosition(pivot_motor.getTargetPosition()-pickupPivotAmount);
        }


        //CLIMB CODE
        if(input.b.held()) {
            right_climb1_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            left_climb1_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            left_climb1_servo.setPower(climbServoPower);
            right_climb1_servo.setPower(climbServoPower);
        }
        else if(input.x.held()) {
            left_climb1_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            right_climb1_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            left_climb1_servo.setPower(climbServoPower);
            right_climb1_servo.setPower(climbServoPower);
        }
        else {
            left_climb1_servo.setPower(0);
            right_climb1_servo.setPower(0);
        }

        //DRIVETRAIN CODE
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        //slowdown multiplier for when arm is up
        if(armUp){
            y=y/4;
            x=x/3;
            rx=rx/3;
        }
        //slowdown multiplier for when arm is moving
        else if (armMoving){
            y=y/2;
            x=x/1.5;
            rx=rx/1.5;
        }

        leftFront.setPower(y + x + rx);
        leftRear.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightRear.setPower(y + x - rx);
        //END OF DRIVETRAIN CODE

        //TELEMETRY CODE
        telemetry.addLine("--------------- POSITIONS ---------------");
        telemetry.addData("Left slide motor current position:", left_slide_motor.getCurrentPosition());
        telemetry.addData("Right slide motor current position:", left_slide_motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Pivot motor current position:", pivot_motor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Wrist Position:", wrist_servo.getPosition());

        telemetry.addLine("--------------- TARGETS ---------------");
        telemetry.addData("Left slide motor target position:", left_slide_motor.getTargetPosition());
        telemetry.addData("Right slide motor target position:", left_slide_motor.getTargetPosition());
        telemetry.addLine();
        telemetry.addData("Pivot motor target position:", pivot_motor.getTargetPosition());


        telemetry.addLine("------ CONDITIONALS BELOW THIS LINE ------");
        if(isExtending) {
            telemetry.addLine("EXTENDO IN INTAKE MODE");
        }

        if(basket) {
            telemetry.addLine("BASKET MODE");
        }

        else {
            telemetry.addLine("SPECIMEN MODE");
        }

        if(armMoving) {
            telemetry.addLine("ARM IS MOVING");
        }

        if(armUp) {
            telemetry.addLine("ARM IS UP");
        }
        //END OF TELEMETRY
        input.pollGamepad(gamepad1);
    }


    public static void armGoUpHighBasket() {

        switch (currentArmStep) {

            case (0):
                left_slide_motor.setTargetPosition(slideRetractedPosition);
                right_slide_motor.setTargetPosition(slideRetractedPosition);
                left_slide_motor.setPower(extendPower);
                right_slide_motor.setPower(extendPower);
                pivot_motor.setTargetPosition(pivotUpHighTarget);
                pivot_motor.setPower(pivotPower);
                if (RobotComponents.isDone(pivot_motor, pivotMargin)&&RobotComponents.isDone(left_slide_motor, slideMargin)) {
                    currentArmStep = 1;
                }
                break;

            case (1):
                wrist_servo.setPosition(wristMiddlePosition);
                left_slide_motor.setPower(extendPower);
                right_slide_motor.setPower(extendPower);
                left_slide_motor.setTargetPosition(slideHighBasketPosition);
                right_slide_motor.setTargetPosition(slideHighBasketPosition);
                if (RobotComponents.isDone(left_slide_motor, slideMargin)) {
                    currentArmStep = 0;
                    armMoving = false;
                    armUp = true;
                }
                break;
        }

    }

    public static void armGoUpLowBasket () {

        switch (currentArmStep) {

            case (0):
                left_slide_motor.setTargetPosition(slideRetractedPosition);
                right_slide_motor.setTargetPosition(slideRetractedPosition);
                left_slide_motor.setPower(extendPower);
                right_slide_motor.setPower(extendPower);
                wrist_servo.setPosition(wristMiddlePosition);
                intakeouttake_servo.setPosition(openPosition);
                if (RobotComponents.isDone(pivot_motor, pivotMargin)&&RobotComponents.isDone(left_slide_motor, slideMargin)) {
                    currentArmStep = 1;
                }
                break;

            case (1):
                pivot_motor.setTargetPosition(pivotPickupSubmersiblePosition);
                pivot_motor.setPower(pivotPower);
                if (RobotComponents.isDone(pivot_motor, pivotMargin)) {
                    currentArmStep = 2;
                }
                break;

            case (2):
                left_slide_motor.setPower(extendPower);
                right_slide_motor.setPower(extendPower);
                left_slide_motor.setTargetPosition(slidePickupSubmersiblePosition);
                right_slide_motor.setTargetPosition(slidePickupSubmersiblePosition);
                if (RobotComponents.isDone(left_slide_motor, slideMargin)) {
                    currentArmStep = 0;
                    armMoving = false;
                    armUp = true;
                }
                break;
        }

    }

    public static void armGoUpHighChamber () {

        switch (currentArmStep) {

            case (0):
                if(left_slide_motor.getCurrentPosition() > slideMargin){
                    left_slide_motor.setTargetPosition(slideRetractedPosition);
                    right_slide_motor.setTargetPosition(slideRetractedPosition);
                } else {
                    currentArmStep++;
                }
                if(isDone(left_slide_motor,slideMargin)){
                    currentArmStep++;
                }
                break;

            case (1):
                pivot_motor.setTargetPosition(pivotHighBarTarget);
                pivot_motor.setPower(pivotPower2);
                if (RobotComponents.isDone(pivot_motor, pivotMargin)) {
                    currentArmStep = 2;
                }
                break;

            case (2):
                wrist_servo.setPosition(wristBarPosition);
                left_slide_motor.setPower(extendPower);
                right_slide_motor.setPower(extendPower);
                left_slide_motor.setTargetPosition(slideHighBarPosition);
                right_slide_motor.setTargetPosition(slideHighBarPosition);
                if (RobotComponents.isDone(left_slide_motor, slideMargin)) {
                    currentArmStep = 0;
                    armMoving = false;
                    armUp = true;
                }
                break;

        }

    }

    public static void armGoUpLowChamber () {

        switch (currentArmStep) {

            case (0):
                left_slide_motor.setTargetPosition(slideRetractedPosition);
                right_slide_motor.setTargetPosition(slideRetractedPosition);
                wrist_servo.setPosition(wristBarPosition);
                intakeouttake_servo.setPosition(openPosition);
                if (RobotComponents.isDone(right_slide_motor, slideMotorPickupPower)) {
                    currentArmStep = 1;
                }
                break;

            case (1):
                pivot_motor.setTargetPosition(pivotWallPickup);
                pivot_motor.setPower(pivotPower2);
                if (RobotComponents.isDone(pivot_motor, pivotMargin)) {
                    currentArmStep = 2;
                }
                break;

            case (2):
                if (RobotComponents.isDone(left_slide_motor, slideMargin)) {
                    currentArmStep = 0;
                    armMoving = false;
                    armUp = true;
                }
                break;
        }

    }

    public static void armRetract() {

        switch (currentArmStep){
            case(0):
                intakeouttake_servo.setPosition(openPosition);
                pivot_motor.setTargetPosition(pivotMiddleTarget);
                if(isDone(pivot_motor, pivotMargin)){
                    currentArmStep++;
                }
                break;
            case(1):
                wrist_servo.setPosition(wristBarPosition);
                left_slide_motor.setTargetPosition(slideRetractedPosition);
                right_slide_motor.setTargetPosition(slideRetractedPosition);
                left_slide_motor.setPower(extendPower);
                right_slide_motor.setPower(extendPower);
                if(isDone(left_slide_motor,slideMargin)){
                    currentArmStep++;
                }
            case(2):
                if(pivot_motor.getCurrentPosition() < pivotMiddleTarget){
                    currentArmStep++;
                    break;
                }
                pivot_motor.setTargetPosition(pivotMiddleTarget);
                pivot_motor.setPower(pivotPower);
                if(isDone(pivot_motor,pivotMargin)){
                    currentArmStep++;
                }
                break;
            case(3):
                pivot_motor.setTargetPosition(pivotDownPosition);
                pivot_motor.setPower(pivotPower);
                if(isDone(pivot_motor,pivotMargin*2)){
                    currentArmStep = 0;
                    armMoving = false;
                    armUp = false;
                    isRetracting = false;
                }
                break;
        }

    }
}
