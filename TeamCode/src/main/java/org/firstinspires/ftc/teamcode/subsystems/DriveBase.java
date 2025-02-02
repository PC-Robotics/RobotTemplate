package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.utility.Utility.normalizePowers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Settings;

public class DriveBase implements Subsystem {
    private LinearOpMode opMode;

    // TODO - If you have different motors, change or add them here. Also edit the constructor motors array and init method and setMotorPowers
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private final DcMotorEx[] motors;

    public boolean usingOdometryWheels;
    public DcMotor verticalEncoder, horizontalEncoder;
    public double startingTicksX, startingTicksY, ticksTraveledX, ticksTraveledY, inchesTraveledX, inchesTraveledY;

    // TODO - if you have a different number of motors, change them here.
    public double[] powers = new double[4];

    public DriveBase(LinearOpMode opMode) {
        this.opMode = opMode;

        // TODO - if you have different motors or a different number of motors, change them here.
        motors = new DcMotorEx[] {leftFront, leftBack, rightFront, rightBack};
    }

    @Override
    public void init() {
        // TODO - if you have different motors or a different number of motors, change them here.
        // the "deviceName" variable should be the name on the hardware map on the driver station
        leftFront = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = opMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = opMode.hardwareMap.get(DcMotorEx.class, "rightBack");

        // TODO - reverse dead wheels if necessary. Use Robot push debugger to test orientation
        // leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        if (usingOdometryWheels) {
            // TODO - make sure name on the hardware map matches with the deviceName variable
            //      note that this code only supports 2 dead wheel + imu localization. 3 dead wheel is on the way.
            //      note it is recommended to place the odometry wheels on ports 0 and 3 of control/expansion hub, as it polls faster
            verticalEncoder = opMode.hardwareMap.get(DcMotor.class, "vertical");
            horizontalEncoder = opMode.hardwareMap.get(DcMotor.class, "horizontal");

            // TODO - reverse dead wheels if necessary. Use Robot push debugger to test orientation
            // verticalEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
            // horizontalEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        reset();

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        stop();
    }

    // TODO - if you have different motors or a different number of motors, change them here
    public void setMotorPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void setMotorPowers(double power) {
        for (DcMotorEx m : motors) {
            m.setPower(power);
        }
    }

    public void setMotorPowers(@NonNull double[] powers) throws IndexOutOfBoundsException {
        if (powers.length != motors.length) {
            throw new IndexOutOfBoundsException("The length of the powers array does not match the length of the motors array.");
        }

        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }


    public void reset() {
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx m : motors) {
            m.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Drive the robot using mecanum drive.
     *
     * @param straight Power for forward/backward movement
     * @param strafe   Power for left/right movement
     * @param turn     Power for turning
     */
    public void mecanumDrive(double straight, double strafe, double turn, boolean fineControl) {
        // calculate powers
        powers[0] = straight + strafe + turn; // front left power
        powers[1] = straight - strafe + turn; // back left power
        powers[2] = straight - strafe - turn; // front right power
        powers[3] = straight + strafe - turn; // back right power

        // powers array is updated inside this method
        normalizePowers(powers);

        for (int i = 0; i < 4; i++) {
            if (fineControl) {
                powers[i] *= 0.5;
            }
        }

        setMotorPowers(powers);
    }

    /**
     * Drive the robot using field-centric drive.
     *
     * @param straight Power for forward/backward movement
     * @param strafe   Power for left/right movement
     * @param turn     Power for turning
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">gm0 guide</a>
     */
    public void fieldCentricDrive(double straight, double strafe, double turn, double heading, boolean fineControl) {

        // calculate rotation
        double rotY = strafe * Math.sin(-heading) + straight * Math.cos(-heading);
        double rotX = strafe * Math.cos(-heading) - straight * Math.sin(-heading);

        // calculate powers
        powers[0] = rotY + rotX + turn; // front left power
        powers[1] = rotY - rotX + turn; // back left power
        powers[2] = rotY - rotX - turn; // front right power
        powers[3] = rotY + rotX - turn; // back right power

        // powers array is updated inside this method
        normalizePowers(powers);

        for (int i = 0; i < 4; i++) {
            if (fineControl) {
                powers[i] *= 0.5;
            }
        }

        setMotorPowers(powers);
    }

    public void resetOdometry() {
        startingTicksX = horizontalEncoder.getCurrentPosition();
        startingTicksY = verticalEncoder.getCurrentPosition();
    }

    public void updateOdometry() {
        ticksTraveledX = horizontalEncoder.getCurrentPosition() - startingTicksX;
        ticksTraveledY = verticalEncoder.getCurrentPosition() - startingTicksY;
        inchesTraveledX = ticksTraveledX * Settings.Odometry.ODOMETRY_WHEEL_IN_PER_TICK;
        inchesTraveledY = ticksTraveledY * Settings.Odometry.ODOMETRY_WHEEL_IN_PER_TICK;
    }

    @Override
    public void telemetry() {
        // TODO - edit telemetry if you have different wheel names or amounts
        opMode.telemetry
                .addData("Front Left Power", leftFront.getPower())
                .addData("Back Left Power", leftBack.getPower())
                .addData("Front Right Power", rightFront.getPower())
                .addData("Back Right Power", rightBack.getPower())
                .addData("Y Ticks Traveled", ticksTraveledY)
                .addData("X Ticks Traveled", ticksTraveledX);
    }
}
