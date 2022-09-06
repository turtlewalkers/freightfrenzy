package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//  Controls:
// left stick forward and backward
// right stick left and right to strafe
// left stick left and right to turn

@TeleOp(name = "Mecanum")
public class Mecanum extends LinearOpMode {

    private DcMotor leftfrontmotor;
    private DcMotor leftbackmotor;
    private DcMotor rightfrontmotor;
    private DcMotor rightbackmotor;
    double frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive;
    @Override
    public void runOpMode() {
        
        leftfrontmotor = hardwareMap.get(DcMotor.class, "leftfrontmotor");
        leftbackmotor = hardwareMap.get(DcMotor.class, "leftbackmotor");
        rightfrontmotor = hardwareMap.get(DcMotor.class, "rightfrontmotor");
        rightbackmotor = hardwareMap.get(DcMotor.class, "rightbackmotor");
        leftfrontmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftbackmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
            
            frontRightDrive = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.25;
            frontLeftDrive  = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.25;
            backRightDrive  = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.25;
            backLeftDrive   = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.25;

            rightfrontmotor.setPower(frontRightDrive);
            rightbackmotor.setPower(backRightDrive);
            leftbackmotor.setPower(backLeftDrive);
            leftfrontmotor.setPower(frontLeftDrive);

            telemetry.addLine("motor name               motor speed");
            telemetry.addLine();
            telemetry.addData("Front right drive speed = ", rightfrontmotor);
            telemetry.addData("Front left drive speed  = ", leftfrontmotor);
            telemetry.addData("Back right drive speed  = ", rightbackmotor);
            telemetry.addData("Back left drive speed   = ", leftbackmotor);
            telemetry.update();
            }
        }
    }
}
