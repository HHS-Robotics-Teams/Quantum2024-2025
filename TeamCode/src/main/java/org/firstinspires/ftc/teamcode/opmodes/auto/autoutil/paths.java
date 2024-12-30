package org.firstinspires.ftc.teamcode.opmodes.auto.autoutil;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import static org.firstinspires.ftc.teamcode.components.RobotComponents.intakeouttake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.left_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.right_slide_motor;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.wrist_servo;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.outtakeDuration;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.pivotIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.slideIntakePosition;
import static org.firstinspires.ftc.teamcode.opmodes.Constants.wristMiddlePosition;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class paths {
    public static Pose2d startPose = new Pose2d(0,0,Math.toRadians(90));

    public static Vector2d chamberBasketVector = new Vector2d(4,26);

    public static Pose2d spikemarkPickupOne = new Pose2d(-24, 36);
    public static Pose2d spikemarkPickupTwo = new Pose2d(-34, 36);
    public static Pose2d spikemarkPickupThree = new Pose2d(-44, 36);
    public static Pose2d basketScorePose = new Pose2d(-36, 12);

    public static TrajectorySequence toChamberBasket;
    public static TrajectorySequence basketChamberScore;
    public static TrajectorySequence chamberToSamplePickup;
    public static TrajectorySequence pickupTwo;

}
