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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous
public class AutonomousRedStorageunit extends LinearOpMode {
    TurtleRobot        turtlerobot   = new TurtleRobot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 120 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.01;
    static final double TURN_SPEED = 0.1;
    int counter = 0;
    Recognition recognition;

    VuforiaCurrentGame vuforiaFreightFrenzy;
    TfodCurrentGame tfodFreightFrenzy;

    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        int index;
        int pos;
        int duckPos = 0;

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
        ElapsedTime     duckruntime = new ElapsedTime();
        duckruntime.reset();
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, 2.5, 2.5, 2.5, 2.5, 1);
        // Get the duck position
        while (duckruntime.seconds() < 3) {
            EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, 1.5, 1.5, 1.5, 1.5, 1);
            EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, -1, -1, -1, -1, 1);
            sleep(333);
            duckPos = recognizeDuckPosition();
        }
        telemetry.addData("Duck Position: %d", duckPos);
        telemetry.update();



        // Field is 144 by 144 inches
        // Each square floor tile is 2 4 by 24 inches
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, 10, 10, 10, 10, 3); //this needs to be adjusted in the competition
        EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, 4, 4, -4, -4, 3); //right
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, 12.5, 12.5, 12.5, 12.5, 5);  // S1: Forward 47 Inches with 5 Sec timeout
        //EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, -12.5, -12.5 , 12.5, 12.5, 3); //left
        arm(turtlerobot, turtlerobot.DRIVE_SPEED, -(duckPos*10), 3);
       // if (duckPos == 3) { 
        //    EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 2, 2, 2, 2, 1);
        //} 
        collectdrop(turtlerobot, false, true);
        //arm(turtlerobot, turtlerobot.DRIVE_SPEED, -5, 3);
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, -17, -17,-17, -17, 3);
        //EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, -12.5, -12.5 ,-12.5, -12.5, 3);
        EncoderDrive(turtlerobot, 0.25, -14, -14, 14, 14, 3); //left
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 17, 17, 17, 17, 5);
        while(counter != 6) {
            startCarousel(turtlerobot, true);
            sleep(500);
            EncoderDrive(turtlerobot, turtlerobot.SLOW_SPEED, 0.2/counter, 0.2/counter, 0.3/counter, 0.3/counter, 1);
            counter += 1;
        } // yes sir yes sir three bags full all for ashay none for you
        stopCarousel(turtlerobot);

        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, -7, -7, -7, -7, 3);
        EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, 10, 10, -10, -10, 3); //right
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 15, 15, 15, 15, 3);
        //EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, -10, -10 , 10, 10, 3); //left
        //EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 16, 16, 16, 16, 4);
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
            turtlerobot.leftfrontmotor.setPower((Math.abs(speed)));
            turtlerobot.leftbackmotor.setPower((Math.abs(speed)));
            turtlerobot.rightfrontmotor.setPower((Math.abs(speed)));
            turtlerobot.rightbackmotor.setPower((Math.abs(speed)));

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

    public void GyroTurn(TurtleRobot turtlerobot, int turnangle) {
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
        //telemetry.addData("Status", "Calibration Complete");
        //telemetry.addData("Action needed:", "Please press the start triangle");
        //telemetry.update();
        // Wait for Start to be pressed on Driver Station.
        ElapsedTime2 = new ElapsedTime();
        // Set motor powers to the variable values.
        Yaw_Angle = turtlerobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        while (Yaw_Angle <= turnangle || isStopRequested()) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = turtlerobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
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
    
    public void startCarousel(TurtleRobot turtlerobot, boolean direction) {
        if (direction == true) {
            turtlerobot.Carouselmotor1.setPower(-0.35);
            turtlerobot.Carouselmotor2.setPower(-0.35);
        } else if (direction == false) {
            turtlerobot.Carouselmotor1.setPower(0.35);
            turtlerobot.Carouselmotor2.setPower(0.35);
        }
    }

    public void stopCarousel(TurtleRobot turtlerobot) {
        turtlerobot.Carouselmotor1.setPower(0);
        turtlerobot.Carouselmotor2.setPower(0);
    }

    public void collectdrop(TurtleRobot turtlerobot, boolean cargo, boolean holdpos) {
        if (cargo == true) {
            if (holdpos == true) {
                turtlerobot.armservo.setPower(-1);
                turtlerobot.ArmMotor.setPower(-0.05);
                sleep(1000);
                turtlerobot.armservo.setPower(0);
                turtlerobot.ArmMotor.setPower(0);
            }
            if (holdpos == false) {
                turtlerobot.armservo.setPower(-1);
                sleep(1000);
                turtlerobot.armservo.setPower(0);
            }
        } else if (cargo == false) {
            if (holdpos == true) {
                turtlerobot.armservo.setPower(1);
                turtlerobot.ArmMotor.setPower(-0.05);
                sleep(1000);
                turtlerobot.armservo.setPower(0);
                turtlerobot.ArmMotor.setPower(0);
            }
            if (holdpos == false) {
                turtlerobot.armservo.setPower(-1);
                sleep(1000);
                turtlerobot.armservo.setPower(0);
            }
        }
    }

    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", turtlerobot.imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", turtlerobot.imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", turtlerobot.imu.getSystemStatus().toString());
        return turtlerobot.imu.isGyroCalibrated();
    }

    public int recognizeDuckPosition() {
        int pos = 4;
        List<Recognition> recognitions;
        int index;
        recognitions = tfodFreightFrenzy.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (recognitions.size() == 0) {
            telemetry.addData("TFOD", "No items detected.");
            pos = 1;
        } else {
            index = 0;
            // Iterate through list and call a function to
            // display info for each recognized object.
            for (Recognition recognition_item : recognitions) {
                recognition = recognition_item;
                // Display info.
                displayInfo(index);
                if (recognition.getLabel().equals("Duck")) {
                    displayInfo(index);
                    sleep(1000);
                    if (recognition.getLeft() < 150) {
                        telemetry.addData("Duck", "Middle");
                        pos = 2;
                    } else if (recognition.getLeft() > 150) {
                        telemetry.addData("Duck", "Right");
                        pos= 3;
                    }
                } else {
                    telemetry.addData("Duck", "Left");
                    pos=1;
                }
                // Increment index.
                index = index + 1;
            }
        }
        telemetry.update();
        return pos;
    }

    public void initCam() {
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        tfodFreightFrenzy = new TfodCurrentGame();

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaFreightFrenzy.initialize(
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
        tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float) 0.7, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodFreightFrenzy.activate();
        // Enable following block to zoom in on target.\
        tfodFreightFrenzy.setZoom(1, 16/9);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }
    
    public void arm(TurtleRobot turtlerobot, double speed,
                    double arminches,
                    double timeoutS) {
        int armtarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            armtarget = turtlerobot.ArmMotor.getCurrentPosition() + (int) (arminches * turtlerobot.COUNTS_PER_INCH);
            turtlerobot.ArmMotor.setTargetPosition(armtarget);


            // Turn On RUN_TO_POSITION
            turtlerobot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            turtlerobot.ArmMotor.setPower((Math.abs(speed)));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the  will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the  continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (turtlerobot.ArmMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to :%7d",
                        armtarget);
                telemetry.addData("Path2", "Running at :%7d",
                        turtlerobot.ArmMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            turtlerobot.ArmMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            turtlerobot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void displayInfo(int i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, recognition.getLeft() + ", " + recognition.getTop());
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, recognition.getRight() + ", " + recognition.getBottom());
    }

}
