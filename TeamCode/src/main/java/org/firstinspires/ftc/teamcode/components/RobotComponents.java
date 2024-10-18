package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.teamcode.excutil.RMath;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineManager;
import org.firstinspires.ftc.teamcode.excutil.liveconfig.LiveSettings;
import org.firstinspires.ftc.teamcode.macros.MacroSequence;

import java.util.ArrayList;
import java.util.List;

public class RobotComponents {

    public static class ServoComponent {
        public Servo servo;
        public String name;

        public ServoComponent(Servo servo, String name) {
            this.servo = servo;
            this.name = name;
        }
    }

    public static class MotorComponent {
        public DcMotor motor;
        public String name;

        public MotorComponent(DcMotor motor, String name) {
            this.motor = motor;
            this.name = name;
        }
    }

    public static IMU.Parameters imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            )
    );

    public static DcMotorEx leftFront = null;
    public static DcMotorEx rightFront = null;
    public static DcMotorEx leftRear = null;
    public static DcMotorEx rightRear = null;

    public static DcMotor left_slide_motor = null;
    public static DcMotor right_slide_motor = null;

    public static DcMotor pivot_motor = null;

    //climb1 = climb stage 1
    public static CRServo right_climb1_servo = null;
    public static CRServo left_climb1_servo = null;
    public static Servo wrist_servo = null;
    public static CRServo intakeouttake_servo = null;

    public static IMU imu;

    public static AndroidSoundPool soundPool;


    public static List<ServoComponent> servos = new ArrayList<>();
    // only positionable (read: encoders attached) motors
    public static List<MotorComponent> motors = new ArrayList<>();

    public static CoroutineManager coroutines = new CoroutineManager();

    private static Servo registerServo(HardwareMap hardwareMap, String id, String debugName) {
        Servo servo = hardwareMap.get(Servo.class, id);

        servos.add(new ServoComponent(servo, debugName));

        return servo;
    }

    private static DcMotor registerEncodedMotor(HardwareMap hardwareMap, String id, String debugName) {
        DcMotor motor = hardwareMap.get(DcMotor.class, id);

        motors.add(new MotorComponent(motor, debugName));

        return motor;
    }

    public static void init(HardwareMap hardwareMap) {
        MacroSequence.reset();

        soundPool = new AndroidSoundPool();
        soundPool.initialize(SoundPlayer.getInstance());
        RobotComponents.soundPool.setVolume(1);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(imuParams);


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        left_slide_motor = registerEncodedMotor(hardwareMap, "left_slide_motor", "Left Slide Motor");
        right_slide_motor = registerEncodedMotor(hardwareMap, "right_slide_motor", "Right Slide Motor");
        left_slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_slide_motor.setTargetPosition(0);
        right_slide_motor.setTargetPosition(0);
        left_slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot_motor = hardwareMap.get(DcMotor.class, "pivot_motor");
        pivot_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot_motor.setTargetPosition(0);
        pivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        right_climb1_servo = hardwareMap.get(CRServo.class, "right_climb1_servo");
        left_climb1_servo = hardwareMap.get(CRServo.class, "left_climb1_servo");

        wrist_servo = registerServo(hardwareMap, "wrist_servo", "Wrist Servo");

        intakeouttake_servo = hardwareMap.get(CRServo.class, "intakeouttake_servo");

    }

    public static void tickSystems(OpMode activeMode) {
        coroutines.tick(activeMode);
        MacroSequence.tick(activeMode);
        LiveSettings.tick(activeMode);
    }

    public static final double     COUNTS_PER_ENCODER_REV    = 8192 ;    // eg: TETRIX Motor Encoder
    public static final double     ENCODER_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    public static final double     ENCODER_COUNTS_PER_INCH         = (COUNTS_PER_ENCODER_REV * ENCODER_GEAR_REDUCTION) /
                                                                (WHEEL_DIAMETER_INCHES * 3.1415);

    public static int encoderDistance(double inches) {
        return (int) (inches * ENCODER_COUNTS_PER_INCH);
    }


    public static double deadZone(float val) {
        return Math.pow(RMath.clamp(val, -1.0, 1.0), 3);
        //return val;
        //return (val > -deadZoneThreshold && val < deadZoneThreshold) ? 0 : val;
    }
}
