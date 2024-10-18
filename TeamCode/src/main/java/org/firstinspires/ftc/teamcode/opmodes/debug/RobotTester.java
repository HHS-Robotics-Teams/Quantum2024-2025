package org.firstinspires.ftc.teamcode.opmodes.debug;


import static org.firstinspires.ftc.teamcode.opmodes.Constants.climbServoPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.Input;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineManager;
import org.firstinspires.ftc.teamcode.opmodes.teleop.CompDrive25;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp
public class RobotTester extends OpMode {

    double pivotPower;
    double slidePower;


    private Input input ;
    private Follower follower;
    boolean isUp = false;
    @Override
    public void init() {
        input = new Input();
        RobotComponents.init(hardwareMap);
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
        telemetry.speak("Testing Mode Enabled");
        telemetry.addLine("Press Play to enter testing mode");
        RobotComponents.pivot_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotComponents.left_slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotComponents.right_slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        input.pollGamepad(gamepad1);
        pivotPower = 0.4;
        slidePower = 0;

        //ARM CODE
        if(input.dpad_up.held()){
            RobotComponents.pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getCurrentPosition()+15);
            pivotPower = .8;
        }
        if(input.dpad_down.held()){
            RobotComponents.pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getCurrentPosition()-15);
            pivotPower = .8;
        }
        if(Math.abs(RobotComponents.pivot_motor.getCurrentPosition()-RobotComponents.pivot_motor.getTargetPosition())>25) {
            pivotPower = .75;
        }

        if(input.y.held()) {
            RobotComponents.right_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getCurrentPosition()+15);
            RobotComponents.left_slide_motor.setTargetPosition(RobotComponents.left_slide_motor.getCurrentPosition()+15);
            slidePower = .6;
        }
        if(input.a.held()) {
            RobotComponents.right_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getCurrentPosition()-15);
            RobotComponents.left_slide_motor.setTargetPosition(RobotComponents.left_slide_motor.getCurrentPosition()-15);
            slidePower = .6;
        }

        if(gamepad1.right_stick_button) {
            RobotComponents.right_slide_motor.setTargetPosition(0);
            RobotComponents.left_slide_motor.setTargetPosition(0);
            slidePower = .6;

        }
        //END OF ARM CODE

        //IntakeOuttake CODE
        if(input.right_trigger.held()) {
            RobotComponents.intakeouttake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            RobotComponents.intakeouttake_servo.setPower(intakePower);
        }
         else if(input.left_trigger.held()) {
            RobotComponents.intakeouttake_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            RobotComponents.intakeouttake_servo.setPower(intakePower);
        } else {
            RobotComponents.intakeouttake_servo.setPower(0);
        }

        if(input.left_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(RobotComponents.wrist_servo.getPosition()+.05);
        }
        if(input.right_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(RobotComponents.wrist_servo.getPosition()-.05);
        }
        //END OF i already hate this naming convention

        //CLIMB CODE
        if(input.b.held()) {
            RobotComponents.right_climb1_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            RobotComponents.left_climb1_servo.setDirection(DcMotorSimple.Direction.FORWARD);
            RobotComponents.left_climb1_servo.setPower(climbServoPower);
            RobotComponents.right_climb1_servo.setPower(climbServoPower);
        }
        else if(input.x.held()) {
            RobotComponents.left_climb1_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            RobotComponents.right_climb1_servo.setDirection(DcMotorSimple.Direction.REVERSE);
            RobotComponents.left_climb1_servo.setPower(climbServoPower);
            RobotComponents.right_climb1_servo.setPower(climbServoPower);
        } else {
            RobotComponents.left_climb1_servo.setPower(0);
            RobotComponents.right_climb1_servo.setPower(0);
        }
        //END OF CLIMB CODE

        //DRIVETRAIN CODE for PedroPathing Mecanum drive
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        //END OF DRIVETRAIN CODE
        RobotComponents.pivot_motor.setPower(pivotPower);
        RobotComponents.left_slide_motor.setPower(slidePower);
        RobotComponents.right_slide_motor.setPower(slidePower);


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
