package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUEx implements Subsystem {
    private LinearOpMode opMode;

    public IMU imu;

    // TODO - Set Parameters according to https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html (Physical Hub Mounting)
    private RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP; // edit the last word
    private RevHubOrientationOnRobot.UsbFacingDirection USBFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT; // edit the last word


    public IMUEx(LinearOpMode opMode) {
        this.opMode = opMode;
        imu = opMode.hardwareMap.get(IMU.class, "imu");
    }

    @Override
    public void init() {
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        logoFacingDirection,
                        USBFacingDirection
                )
        ));
    }

    @Override
    public void start() {}

    public double getHeading() {
        return getHeading(AngleUnit.RADIANS);
    }

    public double getHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    // WRAPPER METHODS
    public void resetYaw() {
        imu.resetYaw();
    }

    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return imu.getRobotOrientation(reference, order, angleUnit);
    }

    public Quaternion getRobotOrientationAsQuaternion() {
        return imu.getRobotOrientationAsQuaternion();
    }

    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return imu.getRobotAngularVelocity(angleUnit);
    }

    public void telemetry() {
        opMode.telemetry.addData("Heading", getHeading());
    }
}
