package org.firstinspires.ftc.teamcode.opmodes.teleop.functional;

import org.firstinspires.ftc.teamcode.excutil.RMath;

public class DriveTest_2 {
    public static double deadZone(float val) {
        return Math.pow(RMath.clamp(val, -1.0, 1.0), 3);
    }
}
