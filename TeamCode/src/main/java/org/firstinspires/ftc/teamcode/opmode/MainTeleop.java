package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.support.Robot;

public class MainTeleop extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {
            // runs mecanum drive
            robot.driveBase.mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper);

            robot.driveBase.telemetry();
            robot.imu.telemetry();
        }
    }
}
