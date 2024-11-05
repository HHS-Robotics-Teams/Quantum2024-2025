package org.firstinspires.ftc.teamcode.opmodes;


public class Constants {
    public static final int pivotUpHighTarget = 1250;
    public static final double spoolRadius = 17.5;
    //TODO TUNE AUTO
    public static final int pivotUpHighAUTOTarget = 1100;
    public static final int pivotMiddleTarget = 800;
    public static final int pivotUpLowTarget = 975;
    public static final int pivotDownPosition = 200;
    public static final int slideLowBasketPosition = 1425;
    public static final int slideHighBasketPosition = 2400;
    public static final int slideHighBasketAUTOPosition = 2500;
    public static final int slideRetractedPosition = 5;
    public static final int pivotHighBarTarget = 800;
    public static final int pivotLowBarTarget = 575;
    public static final int slideHighBarPosition = 450;
    public static final int slideLowBarPosition = 750;
    public static final double wristBarPosition = 1;
    public static double wristRetractedPosition = 1;
    public static double wristIntakePosition = .5;
    public static final double pivotPower = .8;
    public static final double pivotPower2 = .6;
    public static final double PIVOTPOWERDOWN = .2;
    public static final double extendPower = .8;
    public static double intakePower = .8;
    public static double climbServoPower = .4;
    public static double slideMotorPickupPower = extendPower / 4;
    //timeout for each macro step (in seconds)
    public static final double macroTimetoTimeout = 7.5;
    public static final double PIVOTTICKSPEREXTENDOTICK = (1/(Math.cos((Math.PI*312*spoolRadius*slideMotorPickupPower)/30)));

}
