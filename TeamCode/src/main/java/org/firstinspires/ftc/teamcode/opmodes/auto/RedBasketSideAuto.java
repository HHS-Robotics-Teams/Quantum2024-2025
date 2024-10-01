package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.BlueBasketSideAuto.ALLOWABLESLIDEERROR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.opmodes.teleop.CompDrive25;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class RedBasketSideAuto extends LinearOpMode {
    private Follower follower;

    /* EVERY SINGLE LOCATION FOR AUTONOMOUS */
    Pose startPose = new Pose(136, 55.6, Math.toRadians(0));
    Point redBasketStart = new Point(136, 55.6, Point.CARTESIAN);
    Point redBasketSpeceminDrop = new Point(110, 64.5, Point.CARTESIAN);
    Point redBasketDrop = new Point(112, 14, Point.CARTESIAN);
    Point redBasketScore = new Point(123, 18, Point.CARTESIAN);
    private PathChain cycleOne;
    private PathChain pickupTwo;
    private PathChain cycleTwo;
    private PathChain pickupThree;
    private PathChain cycleThree;

    //TODO GET VALUES
    int speceminPivotDrop = 200;
    int rightSlideSpecimin = 200;
    int leftSlideSpecemin = 200;
    int leftSlideMiddleSpikemark;
    int rightSlideMiddleSpikemark;
    int leftSlideRightSpikemark;
    int rightSlideRightSpikemark;
    int leftSlideLeftSpikemark;
    int rightSlideLeftSpikemark;



    @Override
    public void runOpMode() {
        //INIT
        RobotComponents.init(hardwareMap);
        follower = new Follower(hardwareMap);

        //Initial Positions
        RobotComponents.left_slide_motor.setTargetPosition(CompDrive25.leftSlideRetractedPosition);
        RobotComponents.right_slide_motor.setTargetPosition(CompDrive25.rightSlideRetractedPosition);
        RobotComponents.pivot_motor.setTargetPosition(CompDrive25.pivotDownPosition);
        RobotComponents.wrist_servo.setPosition(CompDrive25.wristRetractedPosition);

        follower.setStartingPose(startPose);
        buildPaths();
        //END OF INIT

        //To hanging drop
        follower.followPath(toDrop, true);

        //Extend
        RobotComponents.pivot_motor.setTargetPosition(speceminPivotDrop);
        RobotComponents.right_slide_motor.setTargetPosition(rightSlideSpecimin);
        RobotComponents.left_slide_motor.setTargetPosition(leftSlideSpecemin);
        RobotComponents.wrist_servo.setPosition(CompDrive25.wristIntakePosition);

        //Place
        try { wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}
        RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.REVERSE);
        RobotComponents.intakeouttake_servo.setPower(CompDrive25.intakePower);
        try { wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}

        //Retract
        RobotComponents.wrist_servo.setPosition(CompDrive25.wristRetractedPosition);
        RobotComponents.left_slide_motor.setTargetPosition(CompDrive25.leftSlideRetractedPosition);
        RobotComponents.right_slide_motor.setTargetPosition(CompDrive25.rightSlideRetractedPosition);
        RobotComponents.pivot_motor.setTargetPosition(CompDrive25.pivotDownPosition);

        //To spikemark cycle
        follower.followPath(toCycle, true);

        //1st Pickup
        RobotComponents.left_slide_motor.setTargetPosition(leftSlideMiddleSpikemark);
        RobotComponents.right_slide_motor.setTargetPosition(rightSlideMiddleSpikemark);
        intake();

        //1st Drop
        follower.followPath(cycleOne);
        outtake();

        //2nd Pickup
        follower.followPath(pickupTwo);
        RobotComponents.left_slide_motor.setTargetPosition(leftSlideRightSpikemark);
        RobotComponents.right_slide_motor.setTargetPosition(rightSlideRightSpikemark);
        intake();

        //2nd Drop
        follower.followPath(cycleTwo);
        outtake();

        //3rd Pickup
        follower.followPath(pickupThree);
        RobotComponents.left_slide_motor.setTargetPosition(leftSlideLeftSpikemark);
        RobotComponents.right_slide_motor.setTargetPosition(rightSlideLeftSpikemark);

        //3rd Drop
        follower.followPath(cycleThree);
        outtake();

        //Park?
        //TODO follower.followPath(park);

    }

    // PATHS

    Path toDrop = (new Path(new BezierCurve(redBasketStart, redBasketSpeceminDrop)));
    Path toCycle = (new Path(new BezierCurve(redBasketSpeceminDrop, redBasketDrop)));

    public void buildPaths() {
        cycleOne = follower.pathBuilder().addPath(new BezierCurve(redBasketDrop, redBasketScore)).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45)).build();
        pickupTwo = follower.pathBuilder().addPath(new BezierCurve(redBasketScore, redBasketDrop)).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-30)).build();
        cycleTwo = follower.pathBuilder().addPath(new BezierCurve(redBasketDrop, redBasketScore)).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-45)).build();
        pickupThree = follower.pathBuilder().addPath(new BezierCurve(redBasketScore, redBasketDrop)).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(35)).build();
        cycleThree = follower.pathBuilder().addPath(new BezierCurve(redBasketDrop, redBasketScore)).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(-45)).build();
    }

    public void intake() {
        RobotComponents.wrist_servo.setPosition(CompDrive25.wristIntakePosition);
        while (!((RobotComponents.left_slide_motor.getCurrentPosition() - RobotComponents.left_slide_motor.getTargetPosition()) < ALLOWABLESLIDEERROR)) {
            try {wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}}
        RobotComponents.intakeouttake_servo.setPower(CompDrive25.intakePower);
        try {wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}
        RobotComponents.intakeouttake_servo.setPower(0);
        RobotComponents.wrist_servo.setPosition(CompDrive25.wristRetractedPosition);
        RobotComponents.right_slide_motor.setTargetPosition(CompDrive25.rightSlideRetractedPosition);
        RobotComponents.left_slide_motor.setTargetPosition(CompDrive25.leftSlideRetractedPosition);
        RobotComponents.pivot_motor.setTargetPosition(CompDrive25.pivotDownPosition);
    }

    public void outtake() {
        RobotComponents.pivot_motor.setTargetPosition(CompDrive25.pivotUpPosition);
        RobotComponents.left_slide_motor.setTargetPosition(CompDrive25.leftSlideHighBasketPosition);
        RobotComponents.right_slide_motor.setTargetPosition(CompDrive25.rightSlideHighBasketPosition);
        RobotComponents.wrist_servo.setPosition(CompDrive25.wristIntakePosition);
        try { wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}
        RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.REVERSE);
        RobotComponents.intakeouttake_servo.setPower(CompDrive25.intakePower);
        try { wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}
        RobotComponents.intakeouttake_servo.setPower(0);
        RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.FORWARD);
        RobotComponents.wrist_servo.setPosition(CompDrive25.wristRetractedPosition);
        RobotComponents.right_slide_motor.setTargetPosition(CompDrive25.rightSlideRetractedPosition);
        RobotComponents.left_slide_motor.setTargetPosition(CompDrive25.leftSlideRetractedPosition);
        RobotComponents.pivot_motor.setTargetPosition(CompDrive25.pivotDownPosition);
    }

}
