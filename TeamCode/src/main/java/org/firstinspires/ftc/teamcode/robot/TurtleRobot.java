package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left Front drive motor:        "leftfrontmotr"
 * Motor channel:  Right Front drive motor:        "rightfrontmotor"
 * Motor channel:  Right Back drive motor:         "rightbackmotor"
 * Motor channel:  Left Back drive motor:          "leftbackmotor"
 * Motor channel:  Arm  motor:  "ArmMotor"
 * Servo channel:  Servo to collect balls:   "armservo"
 * Servo channel:  Carousel motor: "Carouselmotor1"
 */
public class TurtleRobot {
    /* Public OpMode members. */
    public DcMotor rightfrontmotor = null;
    public DcMotor rightbackmotor = null;
    public DcMotor leftfrontmotor = null;
    public DcMotor leftbackmotor = null;
    public DcMotor ArmMotor = null;
    public DcMotor Carouselmotor1 = null;
    public CRServo armservo = null;
    public VuforiaCurrentGame vuforiaFreightFrenzy;
    public TfodCurrentGame tfodFreightFrenzy;
    public ElapsedTime runtime = new ElapsedTime();
    public Recognition recognition;
    public BNO055IMU imu;

    public static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 120 / 25.4;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.31;
    public static final double TURN_SPEED = 0.1;
    public static final double SLOW_SPEED = 0.1;
    public static final double FAST_SPEED = 0.5;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TurtleRobot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        List<Recognition> recognitions;
        int index;
        int pos;
        // Save reference to Hardware map
        hwMap = ahwMap;
        armservo = hwMap.get(CRServo.class, "armservo");
        // Define and Initialize Motors
        leftfrontmotor = hwMap.get(DcMotor.class, "leftfrontmotor");
        leftbackmotor = hwMap.get(DcMotor.class, "leftbackmotor");
        rightfrontmotor = hwMap.get(DcMotor.class, "rightfrontmotor");
        rightbackmotor = hwMap.get(DcMotor.class, "rightbackmotor");
        ArmMotor = hwMap.get(DcMotor.class, "ArmMotor");
        Carouselmotor1 = hwMap.get(DcMotor.class, "Carouselmotor1");
        leftfrontmotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftbackmotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftfrontmotor.setPower(0);
        leftbackmotor.setPower(0);
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);
        ArmMotor.setPower(0);
        Carouselmotor1.setPower(0);
        armservo.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }
}

