package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotPushDebugger extends LinearOpMode {
    private LinearOpMode opMode;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    DcMotor verticalEncoder, horizontalEncoder;

    IMU imu;


    public RobotPushDebugger(LinearOpMode opMode) {
        this.opMode = opMode;

        // edit this
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // edit this if using dead wheels
        boolean usingDeadWheels = true;

        leftFront = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = opMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = opMode.hardwareMap.get(DcMotorEx.class, "rightBack");

        // leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        if (usingDeadWheels) {
            verticalEncoder = opMode.hardwareMap.get(DcMotor.class, "vertical");
            horizontalEncoder = opMode.hardwareMap.get(DcMotor.class, "horizontal");

            // verticalEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
            // horizontalEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("DRIVE MOTORS - Forward should be positive")
                    .addData("Front Left Wheel", leftFront.getCurrentPosition())
                    .addData("Back Left Wheel", leftBack.getCurrentPosition())
                    .addData("Front Right Wheel", rightFront.getCurrentPosition())
                    .addData("Back Right Wheel", rightBack.getCurrentPosition())
                    .addData("Heading (Turning Left = positive)", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw()));

            if (usingDeadWheels) {
                telemetry.addLine("DEAD WHEEL ODOMETRY")
                        .addData("Vertical Encoder (Forward = positive)", verticalEncoder.getCurrentPosition())
                        .addData("Horizontal Encoder (Left = positive)", horizontalEncoder.getCurrentPosition());
            }

            telemetry.update();
        }
    }
}
