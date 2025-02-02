package org.firstinspires.ftc.teamcode.support;

import static org.firstinspires.ftc.teamcode.support.Utility.normalizeAngle_n180_180;

/**
 * PIDF controller
 * kP: Proportional constant (slows down robot as it approaches target)
 * kI: Integral constant (makes sure robot corrects for small errors over time and doesn't stall)
 * kD: Derivative constant (makes sure robot doesn't change velocities too fast)
 * kF: Feedforward constant (guesses what the output should be based on the target)
 * tolerance: Error threshold (how close the robot should be to the target for it to be considered there)
 * timeToSettle: Time threshold (how long the robot should be within the error threshold for it to be considered there)
 * A phenomenon called integral windup can occur when the there's a huge spike in error
 * This can cause the integral to increase to a point where the robot overcorrects and overshoots like crazy
 * To prevent this, we can set an antiWindupRange, which will reset the integral if the error is greater than a specified range
 * antiWindupRange: The range at which the integral will reset if the error is greater than this value
 */
public class PIDF {

    private double kP, kI, kD, kF;
    private double timeToSettle, antiWindupRange;

    private double integral, previousError, previousTime;
    private double tolerance, settleStartTime;
    private boolean withinTolerance, isSettled;
    private boolean isAngular, isDisabled;

    public PIDF(double kP, double kI, double kD, double kF, double tolerance, double timeToSettle, double antiWindupRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
        this.timeToSettle = timeToSettle;
        this.antiWindupRange = antiWindupRange;
        integral = 0;
        previousError = 0;
        previousTime = 0;
        settleStartTime = 0;
        withinTolerance = false;
        isSettled = false;
        isAngular = false;
        isDisabled = false;
    }

    public PIDF(double kP, double kI, double kD, double kF, double tolerance, double timeToSettle) {
        this(kP, kI, kD, kF, tolerance, timeToSettle, 0);
    }

    public PIDF(double kP, double kI, double kD, double tolerance, double timeToSettle) {
        this(kP, kI, kD, 0, tolerance, timeToSettle, 0);
    }

    public PIDF(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 1, 1, 0);
    }

    public PIDF(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 1, 1, 0);
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setTimeToSettle(double timeToSettle) {
        this.timeToSettle = timeToSettle;
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getTimeToSettle() {
        return timeToSettle;
    }

    public void setAntiWindupRange(double antiWindupRange) {
        this.antiWindupRange = antiWindupRange;
    }

    public double getAntiWindupRange() {
        return antiWindupRange;
    }

    public void removeAntiWindup() {
        antiWindupRange = 0;
    }

    public double[] getCoefficients() {
        return new double[] {kP, kI, kD, kF};
    }

    public boolean isSettled() {
        return isSettled;
    }

    public void setIsAngular(boolean isAngular) {
        this.isAngular = isAngular;
    }

    public boolean getIsAngular() {
        return isAngular;
    }

    public double getError() {
        return previousError;
    }

    public void disable() {
        isDisabled = true;
    }

    public void enable() {
        isDisabled = false;
    }

    public boolean isDisabled() {
        return isDisabled;
    }

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double calculate(double targetPosition, double currentPosition, double currentTime) {
        if (isDisabled) {
            return 0.0;
        }

        // --- set previous time and calculate change between cycles ---
        // if previous time is 0 (this is the first cycle), set it to the current time
        if (previousTime == 0) {
            previousTime = currentTime;
        }

        // dT is "delta time" or the change in time between cycles
        double dT = currentTime - previousTime;

        // --- check to see if we are settling and have reach the target ---
        // error is how far we are from the target
        double error = targetPosition - currentPosition;

        // stops the robot from spinning around a circle
        if (isAngular) {
            error = normalizeAngle_n180_180(error);
        }

        // detect if the error is less than the tolerance
        withinTolerance = Math.abs(error) < tolerance;

        if (withinTolerance) {
            // if we just started settling this cycle, set the start time to the current time
            if (settleStartTime == 0) {
                settleStartTime = currentTime;
            }
            // if we have been settling for longer than the time to settle, we are "done"
            if (currentTime - settleStartTime > timeToSettle) {
                isSettled = true;
                return 0.0;
            }
            // we are not settling, so reset the settle start time
        } else {
            settleStartTime = 0;
        }

        // --- calculate the PIDF output ---
        // calculate the derivative
        // if dT is 0, derivative is 0 to avoid dividing by 0
        double derivative = dT != 0 ? (error - previousError) / dT : 0;

        // update the integral
        integral += error * dT;

        // simple anti-windup
        // we check to see if antiWindupRange isn't 0 explicitly to avoid expensive floating point operations
        if (antiWindupRange != 0 && Math.abs(error) > antiWindupRange) {
            integral = 0;
        }

        double output =   (kP * error)
                + (kI * integral)
                + (kD * derivative)
                + (kF * targetPosition);

        // update previous values
        previousError = error;
        previousTime = currentTime;

        return output;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        previousTime = 0;
        settleStartTime = 0;
        withinTolerance = false;
        isSettled = false;
    }
}