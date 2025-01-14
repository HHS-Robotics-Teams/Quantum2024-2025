package org.firstinspires.ftc.teamcode.opmodes;


public class Constants {

    //FLAGS

    //Set true on arm input, set false upon completion of steps
    public static boolean armMoving = false;
    //Set true upon step completion, Set false upon down completion
    public static boolean armUp = false;
    //True if scoring basket, false for chambers
    public static boolean basket = true;
    //True if dpadRight pressed, false upon dpadLeft press
    public static boolean isExtending = false;
    //True if home button pressed, false upon retraction
    public static boolean isRetracting = false;
    //Macro toggle, toggled by back button press
    public static boolean macrosDisabled = false;
    public static final double outtakeDuration = 0.5;
    public static final double overlyLargeNumber =  2147483647;

    //AUTO TIMINGS
    public static double basketChamberTime = 5;
    public static double chmaberToPickupTime = 7.5;
    public static double pickupTwoTime = 7.5;
    public static double pickupThreeTime = 7.5;

    public static final int pivotHighThreshold = 250;
    public static final int pickupPivotAmount = 15;
    public static final int extendoPivotAmount = 50;
    public static final int pivotUpHighTarget = 1200;
    public static final int pivotWallPickup = 240;
    public static final double pivotMargin = 50;
    public static final double slideMargin = 50;
    public static final int pivotMiddleTarget = 800;
    public static final int pivotUpLowTarget = 1050;
    public static final int pivotDownPosition = 134;
    public static final int pivotPickupSubmersiblePosition = 330;
    public static final int slidePickupSubmersiblePosition = 1450;
    public static final int slideLowBasketPosition = 1450;
    public static final int slideHighBasketPosition = 2400;
    public static final int slideMaxExtensionTeleOp = 2725;
    public static final int slideRetractedPosition = 296;
    public static final int pivotHighBarTarget = 900;
    public static final int pivotLowBarTarget = 750;
    public static final int slideHighBarPosition = 1650;
    public static final int slideHighBarScorePosition = 1600;
    public static final int pivotHighBarScore = pivotHighBarTarget - 100;
    public static final int slideLowBarPosition = 1400;
    public static final int pivotIntakePosition = 100;
    public static final int slideIntakePosition = 10;
    public static final double wristMiddlePosition = 0;
    public static final double wristBarPosition = .3;
    public static final double pivotPower = .8;
    public static final double pivotPower2 = .6;
    public static final double pivotIdle = .4;
    public static final double extendPower = .8;
    public static final double slideIdle = .6;
    public static final double openPosition = 0.2;
    public static final double closedPosition = 0.55;
    public static double climbServoPower = .4;
    public static double slideMotorPickupPower = .75;
    //timeout for each macro step (in seconds)
    public static final double macroTimetoTimeout = 10;

}
