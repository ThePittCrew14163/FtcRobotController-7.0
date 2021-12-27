package org.firstinspires.ftc.teamcode;

public class OdometryPosition {
    /**
     * AKA Yaw, or rotation about the z-axis
     */
    public double angle;
    /**
     * X coordinate in inches
     */
    public double x;
    /**
     * Y coordinate in inches
     */
    public double y;
    /**
     * Rotation about the y-axis
     */
    public double pitch;
    /**
     * Rotation about the x-axis
     */
    public double roll;

    public OdometryPosition(double angle, double x, double y, double pitch, double roll) {
        this.angle = angle;
        this.x = x;
        this.y = y;
        this.pitch = pitch;
        this.roll = roll;
    }
}
