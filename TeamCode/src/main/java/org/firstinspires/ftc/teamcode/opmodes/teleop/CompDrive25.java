package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.Input;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(group = "default", name = "Competition Drive")
public class CompDrive25 extends OpMode {
    private Input input ;
    private Follower follower;

    //TODO GET RIGHT VALUES
    public static double intakePower = .6;
    public static double climbServoPower = .4;
    public static double wristLeftPosition = .2;
    public static double wristRightPosition = .8;
    public static int pivotAmount = 200;
    public static int slideMotorPickupPower = 29;
    public static int pivotUpPosition;
    public static int pivotDownPosition;
    public static int leftSlideHighBasketPosition;
    public static int leftSlideLowBasketPosition;
    public static int leftSlideRetractedPosition;
    public static int rightSlideHighBasketPosition;
    public static int rightSlideLowBasketPosition;
    public static int rightSlideRetractedPosition;
    boolean isUp = false;
    @Override
    public void init() {
        input = new Input();
        RobotComponents.init(hardwareMap);
        follower = new Follower(hardwareMap);

        pivotUpPosition = RobotComponents.pivot_motor.getCurrentPosition() - pivotAmount;
        pivotDownPosition = RobotComponents.pivot_motor.getCurrentPosition() + pivotAmount;
        leftSlideRetractedPosition = RobotComponents.left_slide_motor.getCurrentPosition();
        leftSlideHighBasketPosition = (leftSlideRetractedPosition + 400);
        leftSlideLowBasketPosition = (leftSlideRetractedPosition + 200);
        rightSlideRetractedPosition = RobotComponents.left_slide_motor.getCurrentPosition();
        rightSlideHighBasketPosition = (leftSlideRetractedPosition + 400);
        rightSlideLowBasketPosition = (leftSlideRetractedPosition + 200);

        follower.startTeleopDrive();
    }


    @Override
    public void loop() {

        //ARM CODE
        if(input.dpad_up.down()){
            RobotComponents.pivot_motor.setTargetPosition(pivotUpPosition);
            isUp = true;
        }
        if(input.dpad_down.down()){
            RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
            isUp = false;
        }

        if(input.y.down()) {
            RobotComponents.right_slide_motor.setTargetPosition(rightSlideHighBasketPosition);
            RobotComponents.left_slide_motor.setTargetPosition(leftSlideHighBasketPosition);
        }
        if(input.a.down()) {
            RobotComponents.right_slide_motor.setTargetPosition(rightSlideLowBasketPosition);
            RobotComponents.left_slide_motor.setTargetPosition(leftSlideLowBasketPosition);
        }

        if(gamepad1.right_stick_button) {
            RobotComponents.right_slide_motor.setTargetPosition(rightSlideRetractedPosition);
            RobotComponents.left_slide_motor.setTargetPosition(leftSlideRetractedPosition);
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

        if(input.left_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(wristLeftPosition);
        }
        if(input.right_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(wristRightPosition);
        }

        if(input.x.held()) {
            RobotComponents.right_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotComponents.left_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotComponents.right_slide_motor.setPower(slideMotorPickupPower);
            RobotComponents.left_slide_motor.setPower(slideMotorPickupPower);
        }
        if(!(input.x.held())) {
            RobotComponents.right_slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotComponents.left_slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //END OF i already hate this naming convention

        //CLIMB CODE
        if(input.b.held()) {
            RobotComponents.left_climb1_servo.setPower(climbServoPower);
            RobotComponents.right_climb1_servo.setPower(climbServoPower);
        }
        //END OF CLIMB CODE

        //DRIVETRAIN CODE for PedroPathing Mecanum drive
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        //END OF DRIVETRAIN CODE


        //TELEMETRY CODE
        telemetry.addLine("---------------EXTENDO POSITIONS---------------");
        telemetry.addData("Left slide motor current position:", RobotComponents.left_slide_motor.getCurrentPosition());
        telemetry.addData("Right slide motor current position:", RobotComponents.left_slide_motor.getCurrentPosition());

        telemetry.addLine("---------------PIVOT POSITIONS---------------");
        telemetry.addData("Pivot motor current position", RobotComponents.pivot_motor.getCurrentPosition());


        telemetry.addLine("----------------CONDITIONALS BELOW THIS LINE---------------");
        if(RobotComponents.right_slide_motor.getMode()== DcMotor.RunMode.RUN_TO_POSITION) {
            telemetry.addLine("EXTENDO IN PICKUP MODE");
        }
        if(isUp) {
            telemetry.addLine("ARM IS UP");
        }
        //END OF TELEMETRY
    }

}
