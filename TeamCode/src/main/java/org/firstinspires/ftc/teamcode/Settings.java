package org.firstinspires.ftc.teamcode;

public class Settings {
    private Settings() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("This is a settings class and cannot be instantiated");
    }

    public static class Odometry {
        // TODO - currently this is set up for a goBilda 32mm odometry pod. Please change these values if you are using different wheels
        public static final double ODOMETRY_WHEEL_TICKS_PER_REVOLUTION = 2000;
        public static final double ODOMETRY_WHEEL_DIAMETER_IN = 1.2598; // 32mm
        public static final double ODOMETRY_WHEEL_IN_PER_TICK = (ODOMETRY_WHEEL_DIAMETER_IN * Math.PI) / ODOMETRY_WHEEL_TICKS_PER_REVOLUTION;
    }

    public static class DrivePID {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double[] COEFFICIENTS = {kP, kI, kD};
        public static double TOLERANCE = 1;
        public static double TIME_TO_SETTLE = 1;
    }

    public static class StrafePID {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double[] COEFFICIENTS = {kP, kI, kD};
        public static double TOLERANCE = 1;
        public static double TIME_TO_SETTLE = 1;
    }

    // all turning done in radians
    public static class TurnPID {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double[] COEFFICIENTS = {kP, kI, kD};
        public static double TOLERANCE = Math.toRadians(1);
        public static double TIME_TO_SETTLE = 1;
    }

    public static class Autonomous {
        public static int DEFAULT_DRIVE_TIMEOUT_MS = 3000;
        public static double DEFAULT_DRIVE_MAX_POWER = 0.5;
    }
}
