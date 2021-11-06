package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

// Controls:
// Gamepad1:
// driving (POV): gamepad1 right joystick
// Gamepad2:
// carousel: gamepad2 A button
// servo for arm: gamepad2 left trigger
// motor for arm: gamepad2 right joystick

@TeleOp(name = "TeleOP")
public class teleopcode extends LinearOpMode {
    TurtleRobot turtlerobot   = new TurtleRobot();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        turtlerobot.leftfrontmotor = hardwareMap.get(DcMotor.class, "leftfrontmotor");
        turtlerobot.leftbackmotor = hardwareMap.get(DcMotor.class, "leftbackmotor");
        turtlerobot.rightfrontmotor = hardwareMap.get(DcMotor.class, "rightfrontmotor");
        turtlerobot.rightbackmotor = hardwareMap.get(DcMotor.class, "rightbackmotor");
        turtlerobot.Carouselmotor1 = hardwareMap.get(DcMotor.class, "Carouselmotor1");

        turtlerobot.leftfrontmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turtlerobot.leftbackmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // For carousel. ONLY press A ONE time.
                if(gamepad2.a == true) {
                    turtlerobot.Carouselmotor1.setPower(-0.25);
                    sleep(3000);
                }
                if(gamepad2.b == true) {
                    turtlerobot.Carouselmotor1.setPower(-0.25);
                    sleep(3000);
                }
                // The Y and X axis of the Gamepad range from +1 to -1
                turtlerobot.leftfrontmotor.setPower(-(gamepad1.right_stick_y / 2) + gamepad1.right_stick_x / 2);
                turtlerobot.rightfrontmotor.setPower(-(gamepad1.right_stick_y / 2) - gamepad1.right_stick_x / 2);
                // There is a '-' sign so that the topmost position corresponds to maximum forward power.
                turtlerobot.leftbackmotor.setPower(-(gamepad1.right_stick_y / 2) + gamepad1.right_stick_x / 2);
                turtlerobot.rightbackmotor.setPower(-(gamepad1.right_stick_y / 2) - gamepad1.right_stick_x / 2);
                telemetry.addData("Left Front Pow", turtlerobot.leftfrontmotor.getPower());
                telemetry.addData("Left Pow", turtlerobot.leftbackmotor.getPower());
                telemetry.addData("Right Front Pow", turtlerobot.rightfrontmotor.getPower());
                telemetry.addData("Right Pow", turtlerobot.rightbackmotor.getPower());
                telemetry.update();
            }
        }
    }
}