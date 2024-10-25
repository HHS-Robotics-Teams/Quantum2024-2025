package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class AutoPositions {
    private static final double robotWidth = 14.102;
    private static final double robotHeight = 15.447;

    //TODO TUNE ALL
    public static Point blueObservationStartPose = new Point((robotWidth/2),(48+(robotHeight/2)),Point.CARTESIAN);
    public static Point blueObservationPark = new Point(((robotWidth/2)+.5),(.5+(robotHeight/2)),Point.CARTESIAN);
    public static Point blueBasketStartPose = new Point((robotWidth/2),(96-(robotHeight/2)),Point.CARTESIAN);
    public static Point blueBasketScorePose = new Point(19,125,Point.CARTESIAN);
    public static Point blueBasketPickupControlPoint = new Point(25,78,Point.CARTESIAN);
    public static Point blueBasketPickup1 = new Point(45.5,113,Point.CARTESIAN);
    public static Point blueBasketPickup2 = new Point(45.5,123,Point.CARTESIAN);
    public static Point blueBasketPickup3 = new Point(45.5,134,Point.CARTESIAN);
    public static Point blueBasketParkControlPoint = new Point(55,40,Point.CARTESIAN);
    public static Point blueBasketPark = new Point(10,34,Point.CARTESIAN);;
    public static Point redObservationStartPose = new Point(144 - (robotWidth/2),(144 - 48 - (robotHeight/2)),Point.CARTESIAN);;
    public static Point redObservationPark = new Point(144 -.5 -(robotWidth/2),(144 - .5 - (robotHeight/2)),Point.CARTESIAN);;
    public static Point redBasketStartPose;
    public static Point redBasketScorePose;

}
