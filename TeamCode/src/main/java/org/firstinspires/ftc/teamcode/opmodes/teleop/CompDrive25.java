package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.opmodes.Constants.climbServoPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.macroTimetoTimeout;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotHighBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotLowBarTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMaxTeleOp;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpLowTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideLowBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideLowBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMaxExtensionTeleOp;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMotorPickupPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristBarPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristIntakePosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.Input;
import org.firstinspires.ftc.teamcode.opmodes.Constants;

@TeleOp(group = "A most important group", name = "Competition Drive")
public class CompDrive25 extends OpMode {
    //TODO REMEMBER TO PUT IN THE TUNING FOR MEEPMEEP
    public Input input ;
    //Set true on arm input, set false upon completion of steps
    public boolean armMoving = false;
    //Set true upon step completion, Set false upon down completion
    public boolean armUp = false;
    //True if scoring basket, false otherwise
    public boolean basket = true;
    //True if dpadRight pressed, false upon dpadLeft press
    public boolean isExtending = false;
    //Used to determine which macro is run
    public String armDirection;
    //Used to step the macros
    public int currentArmStep;
    public double macroTimeout;

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
        if(!armUp&&!armMoving&&!isExtending){RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);}
        RobotComponents.pivot_motor.setPower(pivotPower2);
        RobotComponents.right_slide_motor.setPower(slideMotorPickupPower);
        RobotComponents.left_slide_motor.setPower(slideMotorPickupPower);

        //ARM CODE
        if(input.dpad_up.down()&&!armMoving&&!isExtending){
            macroTimeout = getRuntime();
            armMoving = true;
            armDirection = "High Pole";
            currentArmStep = 0;
        }

        if(input.dpad_down.down()&&!armMoving&&!isExtending){
            macroTimeout = getRuntime();
            armMoving = true;
            armDirection = "Low Pole";
            currentArmStep = 0;
        }

        if((gamepad1.right_stick_button||gamepad1.left_stick_button)&&!armMoving) {
            macroTimeout = getRuntime();
            armMoving = true;
            armDirection = "Retract";
            currentArmStep = 0;
        }

        if(input.start.down()){
            basket = !basket;
        }

        //ARM LOGIC
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
                            RobotComponents.pivot_motor.setTargetPosition(pivotMiddleTarget);
                            RobotComponents.pivot_motor.setPower(pivotPower);
                            if((Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20)
                                ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                    currentArmStep = 1;
                                    macroTimeout = getRuntime();
                            }
                            break;

                        case(1):
                            RobotComponents.pivot_motor.setTargetPosition(pivotUpHighTarget);
                            RobotComponents.pivot_motor.setPower(pivotPower2);
                            if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                    currentArmStep = 2;
                                    macroTimeout=getRuntime();
                            }
                            break;

                        case(2):
                            RobotComponents.left_slide_motor.setPower(extendPower);
                            RobotComponents.right_slide_motor.setPower(extendPower);
                            RobotComponents.left_slide_motor.setTargetPosition(slideHighBasketPosition);
                            RobotComponents.right_slide_motor.setTargetPosition(slideHighBasketPosition);
                            if(Math.abs(RobotComponents.left_slide_motor.getTargetPosition()-RobotComponents.left_slide_motor.getCurrentPosition()) < 20
                                ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                    currentArmStep = 3;
                                    macroTimeout=getRuntime();
                            }
                            break;

                        case(3):
                            RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                            currentArmStep = 0;
                            armMoving = false;
                            armUp = true;
                            macroTimeout=getRuntime();
                            break;
                    }
                    break;

                case "Low Pole":
                    telemetry.addLine("Going Low Basket");

                    switch (currentArmStep){
                        case(0):
                            RobotComponents.pivot_motor.setTargetPosition(pivotMiddleTarget);
                            RobotComponents.pivot_motor.setPower(pivotPower);
                            if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                    currentArmStep = 1;
                                    macroTimeout=getRuntime();
                            }
                            break;

                        case(1):
                            RobotComponents.pivot_motor.setTargetPosition(pivotUpLowTarget);
                            RobotComponents.pivot_motor.setPower(pivotPower2);
                            if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                    currentArmStep = 2;
                                    macroTimeout=getRuntime();
                            }
                            break;

                        case(2):
                            RobotComponents.left_slide_motor.setPower(extendPower);
                            RobotComponents.right_slide_motor.setPower(extendPower);
                            RobotComponents.left_slide_motor.setTargetPosition(slideLowBasketPosition);
                            RobotComponents.right_slide_motor.setTargetPosition(slideLowBasketPosition);
                            if(Math.abs(RobotComponents.left_slide_motor.getTargetPosition()-RobotComponents.left_slide_motor.getCurrentPosition()) < 20
                                ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                    currentArmStep = 3;
                                    macroTimeout=getRuntime();
                            }
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
                            RobotComponents.left_slide_motor.setPower(extendPower);
                            RobotComponents.right_slide_motor.setPower(extendPower);
                            RobotComponents.left_slide_motor.setTargetPosition(slideRetractedPosition);
                            RobotComponents.right_slide_motor.setTargetPosition(slideRetractedPosition);
                            if(Math.abs(RobotComponents.left_slide_motor.getTargetPosition()-RobotComponents.left_slide_motor.getCurrentPosition()) < 20
                                ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                    currentArmStep = 2;
                                    macroTimeout=getRuntime();
                            }
                            break;

                        case(2):
                            if(isExtending){currentArmStep=3;isExtending=false;break;}
                            RobotComponents.pivot_motor.setTargetPosition(pivotMiddleTarget);
                            RobotComponents.pivot_motor.setPower(pivotPower);
                            if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                    currentArmStep = 3;
                                    macroTimeout=getRuntime();
                            }
                            break;

                        case(3):
                            RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
                            RobotComponents.pivot_motor.setPower(pivotPower2);
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
                                RobotComponents.pivot_motor.setTargetPosition(pivotMiddleTarget);
                                RobotComponents.pivot_motor.setPower(pivotPower);
                                if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                    ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                        currentArmStep = 1;
                                        macroTimeout=getRuntime();
                                }
                                break;

                            case(1):
                                RobotComponents.pivot_motor.setTargetPosition(pivotHighBarTarget);
                                RobotComponents.pivot_motor.setPower(pivotPower2);
                                if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                    ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                        currentArmStep = 2;
                                        macroTimeout=getRuntime();
                                }
                                break;

                            case(2):
                                RobotComponents.left_slide_motor.setPower(extendPower);
                                RobotComponents.right_slide_motor.setPower(extendPower);
                                RobotComponents.left_slide_motor.setTargetPosition(slideHighBarPosition);
                                RobotComponents.right_slide_motor.setTargetPosition(slideHighBarPosition);
                                if(Math.abs(RobotComponents.left_slide_motor.getTargetPosition()-RobotComponents.left_slide_motor.getCurrentPosition()) < 20
                                    || ((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                        currentArmStep = 3;
                                        macroTimeout=getRuntime();
                                }
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
                        telemetry.addLine("Going Low Basket");

                        switch (currentArmStep){
                            case(0):
                                RobotComponents.pivot_motor.setTargetPosition(pivotMiddleTarget);
                                RobotComponents.pivot_motor.setPower(pivotPower);
                                if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                    ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                        currentArmStep = 1;
                                        macroTimeout=getRuntime();
                                }
                                break;

                            case(1):
                                RobotComponents.pivot_motor.setTargetPosition(pivotLowBarTarget);
                                RobotComponents.pivot_motor.setPower(pivotPower2);
                                if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                    ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                        currentArmStep = 2;
                                        macroTimeout=getRuntime();
                                }
                                break;

                            case(2):
                                RobotComponents.left_slide_motor.setPower(extendPower);
                                RobotComponents.right_slide_motor.setPower(extendPower);
                                RobotComponents.left_slide_motor.setTargetPosition(slideLowBarPosition);
                                RobotComponents.right_slide_motor.setTargetPosition(slideLowBarPosition);
                                if(Math.abs(RobotComponents.left_slide_motor.getTargetPosition()-RobotComponents.left_slide_motor.getCurrentPosition()) < 20
                                    ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                        currentArmStep = 3;
                                        macroTimeout=getRuntime();
                                }
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
                                RobotComponents.left_slide_motor.setPower(extendPower);
                                RobotComponents.right_slide_motor.setPower(extendPower);
                                RobotComponents.left_slide_motor.setTargetPosition(slideRetractedPosition);
                                RobotComponents.right_slide_motor.setTargetPosition(slideRetractedPosition);
                                if(Math.abs(RobotComponents.left_slide_motor.getTargetPosition()-RobotComponents.left_slide_motor.getCurrentPosition()) < 20
                                    ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                        currentArmStep = 2;
                                        macroTimeout=getRuntime();
                                }
                                break;

                            case(2):
                                if(isExtending){currentArmStep=3;isExtending=false;break;}
                                RobotComponents.pivot_motor.setTargetPosition(pivotMiddleTarget);
                                RobotComponents.pivot_motor.setPower(pivotPower);
                                if(Math.abs(RobotComponents.pivot_motor.getTargetPosition()-RobotComponents.pivot_motor.getCurrentPosition()) < 20
                                    ||((getRuntime()-macroTimeout)>macroTimetoTimeout)){
                                        currentArmStep = 3;
                                        macroTimeout=getRuntime();
                                }
                                break;

                            case(3):
                                RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
                                RobotComponents.pivot_motor.setPower(pivotPower2);
                                currentArmStep = 0;
                                armMoving = false;
                                armUp = false;
                                break;

                        }
                        break;
                }
            }

            telemetry.addLine();
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
            RobotComponents.wrist_servo.setPosition(RobotComponents.wrist_servo.getPosition() - .25);
        }

        if(input.right_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(RobotComponents.wrist_servo.getPosition() + .25);
        }

        //EXTEND FOR PICKUP CODE
        if(input.y.held()&&RobotComponents.pivot_motor.getCurrentPosition() < pivotMaxTeleOp){
            isExtending = true;
            RobotComponents.pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getTargetPosition()+5);
        }
        if(input.a.held()&&RobotComponents.pivot_motor.getCurrentPosition() > 5){
            isExtending = true;
            RobotComponents.pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getTargetPosition()-5);
        }
        if(input.dpad_right.held()&&RobotComponents.left_slide_motor.getCurrentPosition() < slideMaxExtensionTeleOp) {
            isExtending = true;
            RobotComponents.right_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getTargetPosition() + 15);
            RobotComponents.left_slide_motor.setTargetPosition(RobotComponents.left_slide_motor.getTargetPosition() + 15);
        }
        if(input.dpad_left.down()&&RobotComponents.left_slide_motor.getCurrentPosition() > slideRetractedPosition) {
            isExtending = false;
            RobotComponents.left_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getTargetPosition() - 15);
            RobotComponents.right_slide_motor.setTargetPosition(RobotComponents.left_slide_motor.getTargetPosition() - 15);
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

        //DRIVETRAIN CODE
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        if(armUp){
            y=y/4;
            x=x/3;
            rx=rx/3;
        }
        else if (armMoving){
            y=y/2;
            x=x/1.5;
            rx=rx/1.5;
        }

        RobotComponents.leftFront.setPower(y + x + rx);
        RobotComponents.leftRear.setPower(y - x + rx);
        RobotComponents.rightFront.setPower(y - x - rx);
        RobotComponents.rightRear.setPower(y + x - rx);
        //END OF DRIVETRAIN CODE

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

}
