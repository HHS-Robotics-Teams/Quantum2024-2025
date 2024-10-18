package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.Constants.intakePower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristRetractedPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class BlueBasketSideAuto extends LinearOpMode {
    private Follower follower;


    /* EVERY SINGLE LOCATION FOR AUTONOMOUS */
    Pose startPose = new Pose(8, 88.2, Math.toRadians(180));
    Point blueBasketStart = new Point(8, 88.2, Point.CARTESIAN);
    Point blueBasketSpeceminDrop = new Point(34, 80.5, Point.CARTESIAN);
    Point blueBasketDrop = new Point(20, 132, Point.CARTESIAN);
    Point blueBasketScore = new Point(20, 129, Point.CARTESIAN);
    private PathChain cycleOne;
    private PathChain pickupTwo;
    private PathChain cycleTwo;
    private PathChain pickupThree;
    private PathChain cycleThree;

    //TODO GET VALUES
    public static final int ALLOWABLESLIDEERROR = 20;
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
        RobotComponents.left_slide_motor.setTargetPosition(slideRetractedPosition);
        RobotComponents.right_slide_motor.setTargetPosition(slideRetractedPosition);
        RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
        RobotComponents.wrist_servo.setPosition(wristRetractedPosition);

        follower.setStartingPose(startPose);
        buildPaths();
        //END OF INIT

        //To hanging drop
        follower.followPath(toDrop, true);

        //Extend
        RobotComponents.pivot_motor.setTargetPosition(speceminPivotDrop);
        RobotComponents.right_slide_motor.setTargetPosition(rightSlideSpecimin);
        RobotComponents.left_slide_motor.setTargetPosition(leftSlideSpecemin);
        RobotComponents.wrist_servo.setPosition(wristIntakePosition);

        //Place
        try { wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}
        RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.REVERSE);
        RobotComponents.intakeouttake_servo.setPower(intakePower);
        try { wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}

        //Retract
        RobotComponents.wrist_servo.setPosition(wristRetractedPosition);
        RobotComponents.left_slide_motor.setTargetPosition(slideRetractedPosition);
        RobotComponents.right_slide_motor.setTargetPosition(slideRetractedPosition);
        RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);

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

    Path toDrop = (new Path(new BezierCurve(blueBasketStart, blueBasketSpeceminDrop)));
    Path toCycle = (new Path(new BezierCurve(blueBasketSpeceminDrop, blueBasketDrop)));

    public void buildPaths() {
        cycleOne = follower.pathBuilder().addPath(new BezierCurve(blueBasketDrop, blueBasketScore)).setLinearHeadingInterpolation(Math.PI, Math.toRadians(135)).build();
        pickupTwo = follower.pathBuilder().addPath(new BezierCurve(blueBasketScore, blueBasketDrop)).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150)).build();
        cycleTwo = follower.pathBuilder().addPath(new BezierCurve(blueBasketDrop, blueBasketScore)).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135)).build();
        pickupThree = follower.pathBuilder().addPath(new BezierCurve(blueBasketScore, blueBasketDrop)).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(210)).build();
        cycleThree = follower.pathBuilder().addPath(new BezierCurve(blueBasketDrop, blueBasketScore)).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(135)).build();
    }

    public void intake() {
        RobotComponents.wrist_servo.setPosition(wristIntakePosition);
        while (!((RobotComponents.left_slide_motor.getCurrentPosition() - RobotComponents.left_slide_motor.getTargetPosition()) < ALLOWABLESLIDEERROR)) {
            try {wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}}
        RobotComponents.intakeouttake_servo.setPower(intakePower);
        try {wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}
        RobotComponents.intakeouttake_servo.setPower(0);
        RobotComponents.wrist_servo.setPosition(wristRetractedPosition);
        RobotComponents.right_slide_motor.setTargetPosition(slideRetractedPosition);
        RobotComponents.left_slide_motor.setTargetPosition(slideRetractedPosition);
        RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
    }

    public void outtake() {
        RobotComponents.pivot_motor.setTargetPosition(pivotUpHighTarget);
        RobotComponents.left_slide_motor.setTargetPosition(slideHighBasketPosition);
        RobotComponents.right_slide_motor.setTargetPosition(slideHighBasketPosition);
        RobotComponents.wrist_servo.setPosition(wristIntakePosition);
        try { wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}
        RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.REVERSE);
        RobotComponents.intakeouttake_servo.setPower(intakePower);
        try { wait(1); } catch (InterruptedException e) { throw new RuntimeException(e);}
        RobotComponents.intakeouttake_servo.setPower(0);
        RobotComponents.intakeouttake_servo.setDirection(CRServo.Direction.FORWARD);
        RobotComponents.wrist_servo.setPosition(wristRetractedPosition);
        RobotComponents.right_slide_motor.setTargetPosition(slideRetractedPosition);
        RobotComponents.left_slide_motor.setTargetPosition(slideRetractedPosition);
        RobotComponents.pivot_motor.setTargetPosition(pivotDownPosition);
    }

}
