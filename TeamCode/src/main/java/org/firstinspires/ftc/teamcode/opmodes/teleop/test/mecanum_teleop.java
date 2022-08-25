package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@TeleOp(name = "Mecanum TeleOp")
public class mecanum_teleop extends LinearOpMode {
    TurtleRobot turtlerobot = new TurtleRobot();
    double frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive;

    @Override
    public void runOpMode() {
        turtlerobot.leftfrontmotor = hardwareMap.get(DcMotor.class, "leftfrontmotor");
        turtlerobot.leftbackmotor = hardwareMap.get(DcMotor.class, "leftbackmotor");
        turtlerobot.rightfrontmotor = hardwareMap.get(DcMotor.class, "rightfrontmotor");
        turtlerobot.rightbackmotor = hardwareMap.get(DcMotor.class, "rightbackmotor");

        turtlerobot.leftfrontmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turtlerobot.leftbackmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            frontRightDrive = gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_y;
            frontLeftDrive  = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_y;
            backRightDrive  = gamepad1.left_stick_x - gamepad1.left_stick_y + gamepad1.right_stick_y;
            backLeftDrive   = gamepad1.left_stick_y + gamepad1.left_stick_y - gamepad1.right_stick_y;

            turtlerobot.rightfrontmotor.setPower(frontRightDrive);
            turtlerobot.rightbackmotor.setPower(backRightDrive);
            turtlerobot.leftbackmotor.setPower(backLeftDrive);
            turtlerobot.leftfrontmotor.setPower(frontLeftDrive);

            telemetry.addLine("motor name               motor speed");
            telemetry.addLine();
            telemetry.addData("Front right drive speed = ", turtlerobot.rightfrontmotor);
            telemetry.addData("Front left drive speed  = ", turtlerobot.leftfrontmotor);
            telemetry.addData("Back right drive speed  = ", turtlerobot.rightbackmotor);
            telemetry.addData("Back left drive speed   = ", turtlerobot.leftbackmotor);
            telemetry.update();
        }
    }
}
