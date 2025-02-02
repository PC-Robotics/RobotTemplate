package org.firstinspires.ftc.teamcode.support;

import static org.firstinspires.ftc.teamcode.utility.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.IMUEx;

public class Robot {
    private LinearOpMode opMode;

    public PIDF drivePID, strafePID, turnPID;

    public ElapsedTime timer = new ElapsedTime();

    public DriveBase driveBase;
    public IMUEx imu;

    // here you can add more subsystems that you want
    // public Example example;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        imu = new IMUEx(opMode);
        driveBase = new DriveBase(opMode);
        // example = new Example(opMode);

        drivePID = new PIDF(
                Settings.DrivePID.kP,
                Settings.DrivePID.kI,
                Settings.DrivePID.kD,
                Settings.DrivePID.TOLERANCE,
                Settings.DrivePID.TIME_TO_SETTLE
        );

        strafePID = new PIDF(
                Settings.StrafePID.kP,
                Settings.StrafePID.kI,
                Settings.StrafePID.kD,
                Settings.StrafePID.TOLERANCE,
                Settings.StrafePID.TIME_TO_SETTLE
        );

        turnPID = new PIDF(
                Settings.TurnPID.kP,
                Settings.TurnPID.kI,
                Settings.TurnPID.kD,
                Settings.TurnPID.TOLERANCE,
                Settings.TurnPID.TIME_TO_SETTLE
        );
        turnPID.setIsAngular(true);
    }

    public void init() {
        imu.init();
        driveBase.init();
        // example.init();
    }

    public void start() {
        imu.start();
        driveBase.start();
        // example.start();
    }

    public void telemetry() {
        imu.telemetry();
        driveBase.telemetry();
        // example.telemetry();
    }

    // DRIVE
    public void driveDistance(double distance_in) {
        driveDistance(distance_in, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS, Settings.Autonomous.DEFAULT_DRIVE_MAX_POWER);
    }

    public void driveDistance(double distance_in, double maxPower) {
        driveDistance(distance_in, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS, maxPower);
    }

    public void driveDistance(double distance_in, int timeout) {
        driveDistance(distance_in, timeout, Settings.Autonomous.DEFAULT_DRIVE_MAX_POWER);
    }

    public void driveDistance(double distance_in, int timeout, double maxPower) {
        double startingTime = timer.milliseconds(); // get the current time
        double currentTime = timer.milliseconds();
        double startingHeading = imu.getHeading(AngleUnit.DEGREES);

        drivePID.reset();
        strafePID.reset();
        turnPID.reset();

        driveBase.resetOdometry();

        while (currentTime - startingTime < timeout && opMode.opModeIsActive()) {
            if (drivePID.isSettled() && strafePID.isSettled() && turnPID.isSettled()) {
                break;
            }

            double drivePower = drivePID.calculate(distance_in, driveBase.inchesTraveledY, currentTime);
            double strafePower = strafePID.calculate(0, driveBase.inchesTraveledX, currentTime);
            double turnPower = turnPID.calculate(startingHeading, imu.getHeading(AngleUnit.DEGREES), currentTime);

            driveBase.setMotorPowers(normalizePowers(new double[] {
                    drivePower + strafePower + turnPower,
                    drivePower - strafePower + turnPower,
                    drivePower - strafePower - turnPower,
                    drivePower + strafePower - turnPower
            }, maxPower));

            driveBase.updateOdometry();

            opMode.telemetry.addData("Drive Power", drivePower)
                    .addData("Strafe Power", strafePower)
                    .addData("Turn Power", turnPower)
                    .addData("Drive Error", drivePID.getError())
                    .addData("Strafe Error", strafePID.getError())
                    .addData("Turn Error", turnPID.getError())
                    .addData("Target Position", distance_in)
                    .addData("Current Position", driveBase.inchesTraveledY)
                    .addData("Heading", imu.getHeading(AngleUnit.DEGREES));
            opMode.telemetry.update();
        }
    }

    public void strafeDistance(double distance_in) {
        strafeDistance(distance_in, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS);
    }

    public void strafeDistance(double distance_in, int timeout) {
        double startingTime = timer.milliseconds(); // get the current time
        double currentTime = timer.milliseconds();
        double startingHeading = imu.getHeading(AngleUnit.DEGREES);

        drivePID.reset();
        strafePID.reset();
        turnPID.reset();

        driveBase.resetOdometry();

        while (currentTime - startingTime < timeout && opMode.opModeIsActive()) {
            if (drivePID.isSettled() && strafePID.isSettled() && turnPID.isSettled()) {
                break;
            }

            double drivePower = drivePID.calculate(0, driveBase.inchesTraveledY, currentTime);
            double strafePower = strafePID.calculate(distance_in, driveBase.inchesTraveledX, currentTime);
            double turnPower = turnPID.calculate(startingHeading, imu.getHeading(AngleUnit.DEGREES), currentTime);

            driveBase.setMotorPowers(normalizePowers(new double[] {
                    drivePower + strafePower + turnPower,
                    drivePower - strafePower + turnPower,
                    drivePower - strafePower - turnPower,
                    drivePower + strafePower - turnPower
            }));

            driveBase.updateOdometry();

            opMode.telemetry.addData("Drive Power", drivePower)
                    .addData("Strafe Power", strafePower)
                    .addData("Turn Power", turnPower)
                    .addData("Drive Error", drivePID.getError())
                    .addData("Strafe Error", strafePID.getError())
                    .addData("Turn Error", turnPID.getError())
                    .addData("Target Position", distance_in)
                    .addData("Current Position", driveBase.inchesTraveledX)
                    .addData("Heading", imu.getHeading(AngleUnit.DEGREES));
            opMode.telemetry.update();
        }
    }

    public void turnAbsolute(double targetAngle_degrees) {
        strafeDistance(targetAngle_degrees, Settings.Autonomous.DEFAULT_DRIVE_TIMEOUT_MS);
    }

    public void turnAbsolute(double targetAngle_degrees, int timeout) {
        double startingTime = timer.milliseconds(); // get the current time
        double currentTime = timer.milliseconds();

        drivePID.reset();
        strafePID.reset();
        turnPID.reset();

        driveBase.resetOdometry();

        while (currentTime - startingTime < timeout && opMode.opModeIsActive()) {
            if (drivePID.isSettled() && strafePID.isSettled() && turnPID.isSettled()) {
                break;
            }

            double drivePower = drivePID.calculate(0, driveBase.inchesTraveledY, currentTime);
            double strafePower = strafePID.calculate(0, driveBase.inchesTraveledX, currentTime);
            double turnPower = turnPID.calculate(targetAngle_degrees, imu.getHeading(AngleUnit.DEGREES), currentTime);

            driveBase.setMotorPowers(normalizePowers(new double[] {
                    drivePower + strafePower + turnPower,
                    drivePower - strafePower + turnPower,
                    drivePower - strafePower - turnPower,
                    drivePower + strafePower - turnPower
            }));

            driveBase.updateOdometry();

            opMode.telemetry.addData("Drive Power", drivePower)
                    .addData("Strafe Power", strafePower)
                    .addData("Turn Power", turnPower)
                    .addData("Drive Error", drivePID.getError())
                    .addData("Strafe Error", strafePID.getError())
                    .addData("Turn Error", turnPID.getError())
                    .addData("Target Position", targetAngle_degrees)
                    .addData("Current Position", imu.getHeading(AngleUnit.DEGREES));
            opMode.telemetry.update();
        }
    }
}
