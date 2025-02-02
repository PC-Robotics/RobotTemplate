package org.firstinspires.ftc.teamcode.utility;

public final class Utility {
    private Utility() {
        throw new java.lang.UnsupportedOperationException(
                "This is a utility class and cannot be instantiated"
        );
    }


    /**
     * Normalize each value in the powers array if any of the values are greater than 1.
     * This makes sure that the motors won't receive a |value| > 1.0
     *
     * @param powers the array powers for all motors
     * @return powers array normalized to range [0, 1]
     */
    public static double[] normalizePowers(double[] powers) {
        // no need for check for 0 length array since length is given
        double max = 0.0;
        for (double el : powers) {
            max = Math.max(max, Math.abs(el));
        }

        // normalize values to range [0, 1]
        if (max > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }
        return powers;
    }

    /**
     * Normalize each value in the powers array if any of the values are greater than maxPower.
     * This makes sure that the motors won't receive a |value| > maxPower
     *
     * @param powers the array powers for all motors
     * @return powers array normalized to range [-maxPower, maxPower]
     */
    public static double[] normalizePowers(double[] powers, double maxPower) {
        // no need for check for 0 length array since length is given
        double max = 0.0;
        for (double el : powers) {
            max = Math.max(max, Math.abs(el));
        }

        // normalize values to range [0, 1]
        if (max > maxPower) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }
        return powers;
    }


    /**
     * apply a deadzone to an input.
     * If the input is between [-DEADZONE_THRESHOLD, THRESHOLD], return 0.
     *
     * @param value              the value of the joystick input
     * @param DEADZONE_THRESHOLD the deadzone that the pad value will be filtered by
     * @return the joystick value with the deadzone filter applied
     */
    public static double applyDeadzone(double value, double DEADZONE_THRESHOLD) {
        if (Math.abs(value) > DEADZONE_THRESHOLD) {
            return value;
        }

        return 0;
    }


    /**
     * Clamp a value between a minimum and maximum value.
     *
     * @param value the value to clamp
     * @param min   the minimum value
     * @param max   the maximum value
     * @param <T>   the type of the value
     * @return the clamped value
     */
    public static <T extends Comparable<T>> T clamp(T value, T min, T max) {
        if (value.compareTo(min) < 0) {
            return min;
        } else if (value.compareTo(max) > 0) {
            return max;
        } else {
            return value;
        }
    }

    /**
     * Normalizes an angle to the range [-180, 180).
     *
     * @param angle The original angle in degrees.
     * @return The angle in the range [-180, 180).
     */
    public static double normalizeAngle_n180_180(double angle) {
        // Shift so we can modulo properly
        double shifted = (angle + 180) % 360;

        // In Java, remainder can be negative, so ensure a positive result
        if (shifted < 0) {
            shifted += 360;
        }

        // Shift back to [-180, 180)
        return shifted - 180;
    }
}