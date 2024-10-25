package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.Constants.PIVOTPOWERDOWN;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.extendPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotDownPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotMiddleTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotPower2;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighAUTOTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotUpHighTarget;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketAUTOPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideHighBasketPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideRetractedPosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueBasketPark;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueBasketParkControlPoint;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueBasketPickup1;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueBasketPickup2;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueBasketPickup3;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueBasketPickupControlPoint;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueBasketScorePose;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPositions.blueBasketStartPose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.MotorPath;
import org.firstinspires.ftc.teamcode.excutil.coroutines.Coroutine;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineManager;
import org.firstinspires.ftc.teamcode.excutil.coroutines.DelayedStartCoroutine;
import org.firstinspires.ftc.teamcode.excutil.coroutines.SimpleAction;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous
public class BlueBasketSideAuto extends OpMode {

    private Follower follower;
    CoroutineManager manager = new CoroutineManager();
    private String step = "toBasket";

    private PathChain score1;
    private PathChain pickup1;
    private PathChain score2;
    private PathChain pickup2;
    private PathChain score3;
    private PathChain pickup3;
    private PathChain score4;
    private PathChain park;
    MotorPath pivotMiddle;
    MotorPath pivotUpHigh;
    MotorPath pivotDown;
    MotorPath extendLeftHigh;
    MotorPath extendRightHigh;
    private boolean armDoneUp = false;
    private boolean armDoneDown = false;
    private int currentArmStep = 0;
    private double intakeRunTime;

    @Override
    public void init() {
        RobotComponents.init(hardwareMap);
        follower = new Follower(hardwareMap);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        updateMovement();
    }

    public void buildPaths() {

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(blueBasketStartPose, blueBasketScorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(blueBasketScorePose, blueBasketPickupControlPoint, blueBasketPickup1))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(blueBasketPickup1, blueBasketScorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(blueBasketScorePose, blueBasketPickupControlPoint, blueBasketPickup2))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(blueBasketPickup2, blueBasketScorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(blueBasketScorePose, blueBasketPickup3))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(blueBasketPickup3, blueBasketScorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(blueBasketScorePose, blueBasketParkControlPoint, blueBasketPark))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

    }

    public void updateMovement() {
        telemetry.addData("Current step: ", step);
        switch (step) {
            case ("toBasket"):
                follower.followPath(score1);
                if (follower.getXOffset() < 2) {
                    step = "arm1";
                }
                break;
            case ("arm1"):
                follower.holdPoint(new BezierPoint(blueBasketScorePose), Math.toRadians(135));
                if(!armDoneUp){
                    armUpAndScore();
                }
                if (armDoneUp&&!armDoneDown) {
                    armDown();
                }
                if(armDoneDown){
                    armDoneDown = false;
                    armDoneUp = false;
                    step = "pickupPath";
                }
                break;
            case ("pickupPath"):
                follower.followPath(pickup1);
                break;
            case ("arm2"):
                follower.holdPoint(new BezierPoint(blueBasketScorePose), Math.toRadians(135));
                if(!armDoneUp){
                    armUpAndScore();
                }
                if (armDoneUp&&!armDoneDown) {
                    armDown();
                }
                if(armDoneDown){
                    armDoneDown = false;
                    armDoneUp = false;
                    step = "pickupPath2";
                }
                break;
            case ("pickupPath2"):
                follower.followPath(pickup2);
                break;
            case ("arm3"):
                follower.holdPoint(new BezierPoint(blueBasketScorePose), Math.toRadians(135));
                if(!armDoneUp){
                    armUpAndScore();
                }
                if (armDoneUp&&!armDoneDown) {
                    armDown();
                }
                if(armDoneDown){
                    armDoneDown = false;
                    armDoneUp = false;
                    step = "pickupPath3";
                }
                break;
            case ("pickupPath3"):
                follower.followPath(pickup3);
                break;
            case ("arm4"):
                follower.holdPoint(new BezierPoint(blueBasketScorePose), Math.toRadians(135));
                armUpAndScore();
                if (armDoneUp) {
                    step = "parkPath";
                    armDoneUp = false;
                }
                break;
            case ("parkPath"):
                follower.followPath(park);
                break;
        }
    }

    public void armUpAndScore() {
        telemetry.addLine("Going High Basket");

        switch (currentArmStep) {
            case (0):
                pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                if (pivotMiddle.isComplete(50, 2000)) {
                    currentArmStep = 1;
                }
                break;

            case (1):
                pivotUpHigh = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotUpHighAUTOTarget, pivotPower2);
                if (pivotUpHigh.isComplete(50, 2000)) {
                    currentArmStep = 2;
                }
                break;

            case (2):
                extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideHighBasketAUTOPosition, extendPower);
                extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideHighBasketAUTOPosition, extendPower);
                if (extendLeftHigh.isComplete(50, 2000) && extendRightHigh.isComplete(50, 2000)) {
                    currentArmStep = 3;
                }
                break;

            case (3):
                RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                currentArmStep = 4;
                break;

            case(4):
                RobotComponents.intakeouttake_servo.setPower(1);
                intakeRunTime = getRuntime();
                currentArmStep = 5;
                break;

            case(5):
                if(getRuntime()-intakeRunTime > 150) {
                    RobotComponents.intakeouttake_servo.setPower(0);
                    armDoneUp = true;
                }
                break;
        }

    }

    public void armDown() {
        telemetry.addLine("Going Retracting");

        switch (currentArmStep) {
            case (0):
                RobotComponents.intakeouttake_servo.setPower(0);
                RobotComponents.wrist_servo.setPosition(wristIntakePosition);
                currentArmStep = 1;
                break;

            case (1):
                extendLeftHigh = MotorPath.runToPosition(RobotComponents.left_slide_motor, slideRetractedPosition, extendPower);
                extendRightHigh = MotorPath.runToPosition(RobotComponents.right_slide_motor, slideRetractedPosition, extendPower);
                if (extendLeftHigh.isComplete(50, 2000) && extendRightHigh.isComplete(50, 2000)) {
                    currentArmStep = 2;
                }
                break;

            case (2):
                pivotMiddle = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotMiddleTarget, pivotPower);
                if (pivotMiddle.isComplete(50, 2000)) {
                    currentArmStep = 3;
                }
                break;

            case (3):
                pivotDown = MotorPath.runToPosition(RobotComponents.pivot_motor, pivotDownPosition, PIVOTPOWERDOWN);
                currentArmStep = 0;
                armDoneDown = true;
                break;
        }

    }

}
