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
    public static final double overlyLargeNumber =  2147483647;
    public static final int pickupPivotAmount = 6;
    public static final int extendoPivotAmount = 6;
    public static final int pivotUpHighTarget = 1233;
    public static final double pivotMargin = 50;
    public static final double slideMargin = 50;
    public static final int pivotMiddleTarget = 800;
    public static final int pivotUpLowTarget = 1050;
    public static final int pivotDownPosition = 300;
    public static final int slideLowBasketPosition = 1450;
    public static final int slideHighBasketPosition = 2400;
    public static final int slideMaxExtensionTeleOp = 2725;
    public static final int slideRetractedPosition = 5;
    public static final int pivotHighBarTarget = 975;
    public static final int pivotLowBarTarget = 750;
    public static final int slideHighBarPosition = 1575;
    public static final int slideHighBarScorePosition = 1500;
    public static final int pivotHighBarScore = 750;
    public static final int slideLowBarPosition = 1400;
    public static final double wristBarPosition = 1;
    public static final int pivotIntakePosition = 100;
    public static final int slideIntakePosition = 10;
    public static double wristMiddlePosition = .5;
    public static double wristLeftPosition = 0;
    public static final double pivotPower = .8;
    public static final double pivotPower2 = .6;
    public static final double pivotIdle = .4;
    public static final double extendPower = .8;
    public static final double slideIdle = .2;
    public static double intakePower = .8;
    public static double climbServoPower = .4;
    public static double slideMotorPickupPower = .75;
    //timeout for each macro step (in seconds)
    public static final double macroTimetoTimeout = 10;

}
