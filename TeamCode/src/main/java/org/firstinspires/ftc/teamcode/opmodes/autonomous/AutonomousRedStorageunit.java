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

@Autonomous
public class AutonomousRedStorageunit extends LinearOpMode {
    TurtleRobot        turtlerobot   = new TurtleRobot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 120 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.1;
    int counter = 0;



    @Override
    public void runOpMode() {
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

        // Field is 144 by 144 inches
        // Each square floor tile is 24 by 24 inches
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, 10, 10, 10, 10, 3);
        EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, 10, 10 , -10, -10, 3); //right
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, 24, 24, 24, 24, 5);  // S1: Forward 47 Inches with 5 Sec timeout
        EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, -10, -10 , 10, 10, 3); //left
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 10, 10 ,10, 10, 3);
        collectdrop(turtlerobot, false);
        sleep(1000)
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, -10, -10 ,-10, -10, 3);
        EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, -10, -10 , 10, 10, 3); //left
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 30, 30 ,30, 30, 5);
        while(counter != 5)
            moveCarousel(turtlerobot, true);
            sleep(1000);
            EncoderDrive(turtlerobot, turtlerobot.SLOW_SPEED, 0.5, 0.5, 0.5, 0.5, 1);
            counter += 1
        EncoderDrive(turtlerobot,turtlerobot.DRIVE_SPEED, -10, -10, -10, -10, 3);
        EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, 10, 10 , -10, -10, 3); //right
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 15, 15,15, 15, 3);
        EncoderDrive(turtlerobot, turtlerobot.TURN_SPEED, -10, -10 , 10, 10, 3); //left
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 15, 15, 15, 15, 4);


        /*GyroTurn((turtlerobot, -90)); //right
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 24, 24, 24, 24, 4);
        moveCarousel(turtlerobot, true);
        sleep(2000);
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, -24, -24, -24, -24, 4);
        GyroTurn(turtlerobot, 90); //left
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 10, 10, 10, 10, 4);
        GyroTurn(turtlerobot, -90); //right
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 20, 20, 20, 20, 3);
        GyroTurn(turtlerobot, 90); //left
        EncoderDrive(turtlerobot, turtlerobot.DRIVE_SPEED, 24, 24, 24, 24, 7);

*/

        gyro = turtlerobot.imu;

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second

        telemetry.addData("Path", "Complete");
        telemetry.update();



    }

            if (opModeIsActive()) {

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
        newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftDrive.setTargetPosition(newLeftTarget);
        robot.rightDrive.setTargetPosition(newRightTarget);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.leftDrive.setPower(speed);
        robot.rightDrive.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (opModeIsActive() &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            robot.leftDrive.setPower(leftSpeed);
            robot.rightDrive.setPower(rightSpeed);

            // Display drive status for the driver.
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
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

    public void GyroInitiazation() {
        static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
        static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
        static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


        @Override

    }



    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



    public void GyroTurn(TurtleRobot turtlerobot, double turnangle) {


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

