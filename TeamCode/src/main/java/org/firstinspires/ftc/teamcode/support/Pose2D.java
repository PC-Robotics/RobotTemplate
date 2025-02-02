package org.firstinspires.ftc.teamcode.support;

import androidx.annotation.NonNull;

public class Pose2D {
    public double x;
    public double y;
    public double heading;

    public Pose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2D() {
        this(0, 0, 0);
    }

    public Pose2D(Pose2D pose) {
        this(pose.x, pose.y, pose.heading);
    }

    public double distanceTo(Pose2D pose) {
        return Math.hypot(pose.x - x, pose.y - y);
    }

    public double angleTo(Pose2D pose) {
        return Math.atan2(pose.y - y, pose.x - x);
    }

    public void set(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void set(Pose2D pose) {
        this.x = pose.x;
        this.y = pose.y;
        this.heading = pose.heading;
    }

    public void translate(double x, double y) {
        this.x += x;
        this.y += y;
    }

    public void rotate(double angle) {
        heading += angle;
    }

    public void transform(Pose2D pose) {
        translate(pose.x, pose.y);
        rotate(pose.heading);
    }

    public Pose2D copy() {
        return new Pose2D(this);
    }

    @NonNull
    public String toString() {
        return x + ", " + y + ", " + heading;
    }
}
