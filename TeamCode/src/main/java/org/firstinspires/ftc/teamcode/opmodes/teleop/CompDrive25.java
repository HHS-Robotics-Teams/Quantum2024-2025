package org.firstinspires.ftc.teamcode.opmodes.teleop;


import static org.firstinspires.ftc.teamcode.opmodes.Constants.PIVOTPOWERDOWN;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.climbServoPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakeRunTime;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpLowTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideLowBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideMotorPickupPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
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
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


@TeleOp(group = "a important group", name = "Competition Drive")
public class CompDrive25 extends OpMode {
    private Input input ;
    private Follower follower;
    public boolean armMoving = false;
    public boolean stopRequested = false;
    public boolean stopped = false;
    public boolean armUp= false;
    public int armDirection;
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
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
    }


    @Override
    public void loop() {

        //ARM CODE
        if(input.dpad_up.down()&&!armMoving){
            armMoving = true;
            armDirection = 1;
            currentArmStep = 0;
        }
        if(input.dpad_down.down()&&!armMoving){
            armMoving = true;
            armDirection = 2;
            currentArmStep = 0;
        }

        if(gamepad1.right_stick_button&&!armMoving) {
            armMoving = true;
            armDirection = 0;
            currentArmStep = 0;
        }
        if(armMoving){
            telemetry.addLine("ARM IS MOVING");
            /* case 1 for high basket
               case 2 for low basket
               case 0 for retracted */
            switch(armDirection) {
                case 1:
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
                            currentArmStep = 4;
                            break;
                        case(4):
                            if(!stopRequested) {
                                RobotComponents.coroutines.runLater( () -> {
                                    RobotComponents.intakeouttake_servo.setPower(0);
                                    stopped = true;
                                },intakeRunTime);
                                stopRequested = true;
                            }
                            if(!stopped) {
                                RobotComponents.coroutines.runLater(() -> {
                                    RobotComponents.intakeouttake_servo.setPower(intakePower);
                                }, intakeRunTime);
                            } else {
                                currentArmStep = 0;
                                armMoving = false;
                                stopRequested = false;
                                stopped = false;
                                armUp = true;
                            }
                            break;
                    }
                    break;
                case 2:
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
                            currentArmStep = 4;
                            break;
                        case(4):
                            if(!stopRequested) {
                                RobotComponents.coroutines.runLater( () -> {
                                    RobotComponents.intakeouttake_servo.setPower(0);
                                    stopped = true;
                                },intakeRunTime);
                                stopRequested = true;
                            }
                            if(!stopped) {
                                RobotComponents.coroutines.runLater(() -> {
                                    RobotComponents.intakeouttake_servo.setPower(intakePower);
                                }, intakeRunTime);
                            } else {
                                currentArmStep = 0;
                                armMoving = false;
                                stopRequested = false;
                                stopped = false;
                                armUp = true;
                            }
                            break;
                    }
                    break;
                case 0:
                    telemetry.addLine("Going Retracting");
                    switch (currentArmStep){
                        case(0):
                            RobotComponents.intakeouttake_servo.setPower(0);
                            currentArmStep = 1;
                            break;
                        case(1):
                            RobotComponents.wrist_servo.setPosition(wristRetractedPosition);
                            currentArmStep = 2;
                            break;
                        case(2):
                            extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideRetractedPosition, extendPower);
                            extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideRetractedPosition, extendPower);
                            if(extendLeftHigh.isComplete(50, 2000)&&extendRightHigh.isComplete(50,2000)){currentArmStep = 3;}
                            break;
                        case(3):
                            pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                            if(pivotMiddle.isComplete(50, 2000)){currentArmStep = 4;}
                            break;
                        case(4):
                            pivotDown = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotDownPosition, PIVOTPOWERDOWN);
                            currentArmStep = 0;
                            armMoving = false;
                            armUp = false;
                            break;
                    }
                    break;
            }
            telemetry.addData("ArmStep:", currentArmStep);
        }
        //END OF ARM CODE

        //IntakeOuttake CODE
        if(input.right_trigger.down()){
            RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.FORWARD);
            RobotComponents.intakeouttake_servo.setPower(intakePower);
        }
        if(input.left_trigger.down()) {
            RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.REVERSE);
            RobotComponents.intakeouttake_servo.setPower(intakePower);
        }
        // TODO CHECK CORRECT POSITIONS
        if(input.left_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(wristIntakePosition);
        }
        if(input.right_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(wristRetractedPosition);
        }

        if(input.x.held()&&!armMoving&&!armUp) {
            RobotComponents.right_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotComponents.left_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotComponents.right_slide_motor.setPower(slideMotorPickupPower);
            RobotComponents.left_slide_motor.setPower(slideMotorPickupPower);
        } else {
            RobotComponents.right_slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotComponents.left_slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //END OF i already hate this naming convention

        //CLIMB CODE
        if(input.b.held()) {
            RobotComponents.right_climb1_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            RobotComponents.left_climb1_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            RobotComponents.left_climb1_servo.setPower(climbServoPower);
            RobotComponents.right_climb1_servo.setPower(climbServoPower);
        }
        if(input.a.held()) {
            RobotComponents.left_climb1_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            RobotComponents.right_climb1_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            RobotComponents.left_climb1_servo.setPower(climbServoPower);
            RobotComponents.right_climb1_servo.setPower(climbServoPower);
        }
        //END OF CLIMB CODE

        //DRIVETRAIN CODE for PedroPathing
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        //END OF DRIVETRAIN CODE
        input.pollGamepad(gamepad1);

        //TELEMETRY CODE
        telemetry.addLine("---------------EXTENDO POSITIONS---------------");
        telemetry.addData("Left slide motor current position:", RobotComponents.left_slide_motor.getCurrentPosition());
        telemetry.addData("Right slide motor current position:", RobotComponents.left_slide_motor.getCurrentPosition());

        telemetry.addLine("---------------PIVOT POSITIONS---------------");
        telemetry.addData("Pivot motor current position", RobotComponents.pivot_motor.getCurrentPosition());


        telemetry.addLine("--------CONDITIONALS BELOW THIS LINE----------");
        if(RobotComponents.right_slide_motor.getMode()== DcMotor.RunMode.RUN_TO_POSITION) {
            telemetry.addLine("EXTENDO IN INTAKE MODE");
        }
        //END OF TELEMETRY
    }

}
