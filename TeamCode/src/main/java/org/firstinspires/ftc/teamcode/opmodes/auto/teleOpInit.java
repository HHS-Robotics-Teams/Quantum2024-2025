package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.components.RobotComponents.resetEncoders;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponents;

@Autonomous
public class teleOpInit extends OpMode {
    @Override
    public void init() {
        RobotComponents.init(hardwareMap);
        resetEncoders();
    }

    @Override
    public void loop() {

    }
}
