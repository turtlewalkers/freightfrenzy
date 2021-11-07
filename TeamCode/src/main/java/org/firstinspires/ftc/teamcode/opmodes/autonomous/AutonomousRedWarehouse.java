package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;
//help
@Autonomous
public class AutonomousRedWarehouse extends LinearOpMode {
    TurtleRobot        turtlerobot   = new TurtleRobot();   // using OUR hardware
    private ElapsedTime     runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 120 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.1;

    @Override
    public void runOpMode() {
        int index;
        int pos;
        initCam();
        turtlerobot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        turtlerobot.leftfrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turtlerobot.leftbackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turtlerobot.rightbackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turtlerobot.rightfrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                turtlerobot.leftfrontmotor.getCurrentPosition(),
                turtlerobot.leftbackmotor.getCurrentPosition(),
                turtlerobot.rightfrontmotor.getCurrentPosition(),
                turtlerobot.rightbackmotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int cam = recognizeDuckPosition();
        if (cam == 0) {
            turtlerobot.rightbackmotor.setPower(1);
            sleep(5000);
            turtlerobot.rightbackmotor.setPower(0);
        } else if (cam == 1) {
            turtlerobot.leftfrontmotor.setPower(0.5);
            sleep(1000);
            turtlerobot.leftfrontmotor.setPower(0);
        } else {
            turtlerobot.leftbackmotor.setPower(0.1);
            sleep(2500);
            turtlerobot.leftbackmotor.setPower(0);
        }
        // Field is 144 by 144 inches
        // Each square floor tile is 24 by 24 inches
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, 10, 10, 10, 10, 3);
        GyroTurn(turtlerobot, 90); //left
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, 65, 65, 65, 65, 6);  // S1: Forward 47 Inches with 5 Sec timeout
        moveCarousel(turtlerobot, true);
        sleep(2000);
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, -65, -65, -65, -65, 6);
        GyroTurn(turtlerobot, -90); //right
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 24, 24, 24, 24, 4);
        collectdrop(turtlerobot, false);
        sleep(1000);
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, -24, -24, -24, -24, 4);
        GyroTurn(turtlerobot, -90); //right
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 72, 72, 72, 72, 7);


        //EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, -10, -10, -3, -3, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
    }
    public void EncoderDrive(TurtleRobot turtlerobot, double speed,
                             double leftfrontInches, double leftbackInches,
                             double rightfrontInches, double rightbackInches,
                             double timeoutS) {
        int newLeftfrontTarget;
        int newLeftbackTarget;
        int newRightfrontTarget;
        int newRightbackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftfrontTarget = turtlerobot.leftfrontmotor.getCurrentPosition() + (int) (leftfrontInches * turtlerobot.COUNTS_PER_INCH);
            newLeftbackTarget = turtlerobot.leftbackmotor.getCurrentPosition() + (int) (leftbackInches * turtlerobot.COUNTS_PER_INCH);
            newRightfrontTarget = turtlerobot.rightfrontmotor.getCurrentPosition() + (int) (rightfrontInches * turtlerobot.COUNTS_PER_INCH);
            newRightbackTarget = turtlerobot.rightbackmotor.getCurrentPosition() + (int) (rightbackInches * turtlerobot.COUNTS_PER_INCH);
            turtlerobot.leftfrontmotor.setTargetPosition(newLeftfrontTarget);
            turtlerobot.leftbackmotor.setTargetPosition(newLeftfrontTarget);
            turtlerobot.rightfrontmotor.setTargetPosition(newRightfrontTarget);
            turtlerobot.rightbackmotor.setTargetPosition(newRightbackTarget);


            // Turn On RUN_TO_POSITION
            turtlerobot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turtlerobot.leftbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turtlerobot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turtlerobot.rightbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            turtlerobot.leftfrontmotor.setPower(Math.abs(speed));
            turtlerobot.leftbackmotor.setPower(Math.abs(speed));
            turtlerobot.rightfrontmotor.setPower(Math.abs(speed));
            turtlerobot.rightbackmotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the  will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the  continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (turtlerobot.leftfrontmotor.isBusy() &&
                    turtlerobot.leftbackmotor.isBusy()
                    && turtlerobot.rightfrontmotor.isBusy()
                    && turtlerobot.rightbackmotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d",
                        newLeftfrontTarget,
                        newLeftbackTarget,
                        newRightfrontTarget,
                        newRightbackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        turtlerobot.leftfrontmotor.getCurrentPosition(),
                        turtlerobot.leftbackmotor.getCurrentPosition(),
                        turtlerobot.rightfrontmotor.getCurrentPosition(),
                        turtlerobot.rightbackmotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            turtlerobot.leftfrontmotor.setPower(0);
            turtlerobot.leftbackmotor.setPower(0);
            turtlerobot.rightfrontmotor.setPower(0);
            turtlerobot.rightbackmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            turtlerobot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turtlerobot.leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turtlerobot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turtlerobot.rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void GyroTurn(TurtleRobot turtlerobot, double yawangle) {
        BNO055IMU.Parameters IMU_Parameters;
        ElapsedTime ElapsedTime2;
        double Left_Power;
        double Right_Power;
        float Yaw_Angle;

        turtlerobot.leftfrontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turtlerobot.leftbackmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turtlerobot.rightfrontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turtlerobot.rightbackmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse direction of one motor so robot moves
        // forward rather than spinning in place.
        // Create an IMU parameters object.
        IMU_Parameters = new BNO055IMU.Parameters();
        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU gyroscope and accelerometer to
        // calculate the relative orientation of hub and
        // therefore the robot.
        // IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Intialize the IMU using parameters object.
        turtlerobot.imu.initialize(IMU_Parameters);
        // Report the initialization to the Driver Station.
        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();
        // Wait one second to ensure the IMU is ready.
        sleep(1000);
        // Loop until IMU has been calibrated.
        while (!IMU_Calibrated()) {
            telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
            telemetry.update();
            // Wait one second before checking calibration
            // status again.
            sleep(1000);
        }
        // Report calibration complete to Driver Station.
        telemetry.addData("Status", "Calibration Complete");
        telemetry.addData("Action needed:", "Please press the start triangle");
        telemetry.update();
        // Wait for Start to be pressed on Driver Station.
        ElapsedTime2 = new ElapsedTime();
        // Set motor powers to the variable values.
        Yaw_Angle = turtlerobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        while (Yaw_Angle <= 90 || isStopRequested()) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = turtlerobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            turtlerobot.leftfrontmotor.setPower(-0.2);
            turtlerobot.leftbackmotor.setPower(-0.2);
            turtlerobot.rightfrontmotor.setPower(0.2);
            turtlerobot.rightbackmotor.setPower(0.2);
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        turtlerobot.leftfrontmotor.setPower(0);
        turtlerobot.leftbackmotor.setPower(0);
        turtlerobot.rightfrontmotor.setPower(0);
        turtlerobot.leftbackmotor.setPower(0);
        sleep(1000);
    }
    public void moveCarousel(TurtleRobot turtlerobot, boolean direction) {
        if (direction == true) {
            turtlerobot.Carouselmotor1.setPower(-0.25);
        } else if (direction == false) {
            turtlerobot.Carouselmotor1.setPower(0.25);
        }
    }
    public void collectdrop(TurtleRobot turtlerobot, boolean cargo) {
        if (cargo == true) {
            turtlerobot.intakeservo.setPower(-0.25);
        } else if (cargo == false) {
            turtlerobot.intakeservo.setPower(0.25);
        }
    }

    public int recognizeDuckPosition() {
        int pos = 0;
        List<Recognition> recognitions;
        int index;
        recognitions = turtlerobot.tfodFreightFrenzy.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (recognitions.size() == 0) {
            telemetry.addData("TFOD", "No items detected.");
        } else {
            index = 0;
            // Iterate through list and call a function to
            // display info for each recognized object.
            for (Recognition recognition_item : recognitions) {
                turtlerobot.recognition = recognition_item;
                // Display info.
                displayInfo(index);
                if (turtlerobot.recognition.getLabel().equals("Duck")) {
                    if (turtlerobot.recognition.getLeft() < 150) {
                        telemetry.addData("Duck", "Middle");
                        pos = 1; // this means that the arm will go to the middle level
                    } else {
                        telemetry.addData("Duck", "Right");
                        pos= 0; // this means that the arm will go to the top level
                    }
                } else {
                    telemetry.addData("Duck", "Left");
                    pos=2; // this means that the arm will go to the bottom level
                }
                // Increment index.
                index = index + 1;
            }
        }
        telemetry.update();
        return pos;
    }

    public void initCam() {
        turtlerobot.vuforiaFreightFrenzy = new VuforiaCurrentGame();
        turtlerobot.tfodFreightFrenzy = new TfodCurrentGame();

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        turtlerobot.vuforiaFreightFrenzy.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                90, // xAngle
                0, // yAngle
                90, // zAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        turtlerobot.tfodFreightFrenzy.initialize(turtlerobot.vuforiaFreightFrenzy, (float) 0.7, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        turtlerobot.tfodFreightFrenzy.activate();
        // Enable following block to zoom in on target.
        turtlerobot.tfodFreightFrenzy.setZoom(1, 16 / 9);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }
    public void displayInfo(int i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, turtlerobot.recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, turtlerobot.recognition.getLeft() + ", " + turtlerobot.recognition.getTop());
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        
        telemetry.addData("Right, Bottom " + i, turtlerobot.recognition.getRight() + ", " + turtlerobot.recognition.getBottom());
    }
    /**
     * Function that becomes true when gyro is calibrated and
     * reports calibration status to Driver Station in the meantime.
     */
    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", turtlerobot.imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", turtlerobot.imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", turtlerobot.imu.getSystemStatus().toString());
        return turtlerobot.imu.isGyroCalibrated();
    }
}

