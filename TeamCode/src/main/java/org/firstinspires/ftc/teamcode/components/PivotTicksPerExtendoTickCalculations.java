package org.firstinspires.ftc.teamcode.components;

public class PivotTicksPerExtendoTickCalculations {
    static double Pivotticksperdegree = ( ( (1/14) * 537.7) / 360);

    static double Extendoticksperinch = 537.7 / 4.5;

    static double Pivotdegreesperextendoinches = ((Math.acos(12.5/(Math.sqrt((Math.pow(12.5, 2)+Math.pow(25,2)))-(Math.acos(12.5/(Math.sqrt((Math.pow(12.5, 2)+Math.pow(15,2))))))))))/((Math.sqrt((Math.pow(12.5, 2)+Math.pow(25,2)))-(Math.sqrt((Math.pow(12.5, 2)+Math.pow(15,2))))));

    public static double Pivotticksperextendotick = (1/72); //= ( (Pivotticksperdegree/Extendoticksperinch) * Pivotdegreesperextendoinches);

    public static double pivotCounter = 0;
}
