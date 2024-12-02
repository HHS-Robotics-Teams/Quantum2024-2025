package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@TeleOp
public class imuTest extends OpMode {

    private BHI260IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        // TODO check
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN)
        );
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        telemetry.addData("ang vel",imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);
        telemetry.addData("heading",imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);

    }
}
