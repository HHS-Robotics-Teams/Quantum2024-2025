package org.firstinspires.ftc.teamcode.opmodes.debug;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.Input;
import org.firstinspires.ftc.teamcode.opmodes.teleop.CompDrive25;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp
public class RobotTester extends OpMode {

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

        //ARM CODE
        if(input.dpad_up.down()){
            RobotComponents.pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getCurrentPosition()+5);
        }
        if(input.dpad_down.down()){
            RobotComponents.pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getCurrentPosition()-5);
        }

        if(input.y.down()) {
            RobotComponents.right_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getCurrentPosition()+5);
            RobotComponents.left_slide_motor.setTargetPosition(RobotComponents.left_slide_motor.getCurrentPosition()+5);
        }
        if(input.a.down()) {
            RobotComponents.right_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getCurrentPosition()-5);
            RobotComponents.left_slide_motor.setTargetPosition(RobotComponents.left_slide_motor.getCurrentPosition()-5);
        }

        if(gamepad1.right_stick_button) {
            RobotComponents.right_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getCurrentPosition());
            RobotComponents.left_slide_motor.setTargetPosition(RobotComponents.right_slide_motor.getCurrentPosition());
        }
        //END OF ARM CODE

        //IntakeOuttake CODE
        if(input.right_trigger.down()) {
            RobotComponents.intakeouttake_servo.setPower(CompDrive25.intakePower);
        }
        if(input.left_trigger.down()) {
            RobotComponents.intakeouttake_servo.setPower(-CompDrive25.intakePower);
        }

        if(input.left_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(RobotComponents.wrist_servo.getPosition()+.05);
        }
        if(input.right_bumper.down()) {
            RobotComponents.wrist_servo.setPosition(RobotComponents.wrist_servo.getPosition()-.05);
        }

        if(input.x.held()) {
            RobotComponents.right_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotComponents.left_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RobotComponents.right_slide_motor.setPower(CompDrive25.slideMotorPickupPower);
            RobotComponents.left_slide_motor.setPower(CompDrive25.slideMotorPickupPower);
        }
        if(!(input.x.held())) {
            RobotComponents.right_slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotComponents.left_slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //END OF i already hate this naming convention

        //CLIMB CODE
        if(input.b.held()) {
            RobotComponents.left_climb1_servo.setPower(CompDrive25.climbServoPower);
            RobotComponents.right_climb1_servo.setPower(CompDrive25.climbServoPower);
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
