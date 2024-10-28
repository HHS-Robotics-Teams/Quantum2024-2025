package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class AutoPositions {
    public static final double robotWidth = 14.102;
    public static final double robotHeight = 15.447;

    //TODO TUNE ALL
    //BLU
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
    //RED
    public static Point redObservationStartPose = new Point(144 - (robotWidth/2),(144 - 48 - (robotHeight/2)),Point.CARTESIAN);;
    public static Point redObservationPark = new Point(144 -.5 -(robotWidth/2),(144 - .5 - (robotHeight/2)),Point.CARTESIAN);;
    public static Point redBasketStartPose = new Point(144-(robotWidth/2), 48+(robotHeight/2), Point.CARTESIAN);
    public static Point redBasketScorePose = new Point(125,19,Point.CARTESIAN);
    public static Point redBasketPickup1 = new Point(98,32,Point.CARTESIAN);
    public static Point redBasketPickup2 = new Point(98,21,Point.CARTESIAN);
    public static Point redBasketPickup3 = new Point(98,11,Point.CARTESIAN);
    public static Point redBasketPark = new Point(135,110,Point.CARTESIAN);
    public static Point redBasketPickupControlPoint = new Point(120,50,Point.CARTESIAN);
    public static Point redBasketParkControlPoint = new Point(100,120,Point.CARTESIAN);

}
