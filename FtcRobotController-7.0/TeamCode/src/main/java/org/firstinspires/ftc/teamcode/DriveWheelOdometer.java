/*
Copyright 2021 FIRST Tech Challenge Team 14163
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

public class DriveWheelOdometer {
    public double angle = 0;
    public double angle_adjust = 0;
    public double x = 0;
    public double y = 0;
    public final double CLICKS_PER_INCH = (4/3) * 420 / (3.5433 * Math.PI);
    /**
     * Distance from the left center wheel to the center of the robot.
     */
    public final double LEFT_DIS_FROM_CENTER = 7;
    /**
     * Distance from the right center wheel to the center of the robot.
     */
    public final double RIGHT_DIS_FROM_CENTER = 7;

    public OdometryAlgorithm trackingAlgorithm = OdometryAlgorithm.Basic;

    public DriveWheelOdometer(double angle, double x, double y) {
        this.angle = angle;
        this.x = x;
        this.y = y;
    }
    public BNO055IMU imu;
    public DcMotor left_encoder;
    public DcMotor right_encoder;

    public double last_angle = angle;
    public double last_x = x;
    public double last_y = y;

    public double last_left_clicks = 0;
    public double last_right_clicks = 0;

    public LinearOpMode program;

    public void init(BNO055IMU imu, DcMotor left_encoder, DcMotor right_encoder, LinearOpMode program) {
        this.imu = imu;
        this.left_encoder = left_encoder;
        this.right_encoder = right_encoder;
        this.angle_adjust = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - this.angle;
        this.left_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.right_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.left_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.right_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.program = program;
    }

    /**
     * Calculates the robot's movement since this method was last called,
     * updating the current angle, x and y coordinates relative to where the robot started.
     * angle is in radians; x and y are in inches.
     * @return The current angle, x and y coordinates relative to where the robot started.
     */
    public ArrayList<Double> getCurrentCoordinates() {
        // save previous coordinates
        this.last_angle = this.angle;
        this.last_x = this.x;
        this.last_y = this.y;

        // get new angle
        try { this.angle = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - this.angle_adjust; }
        catch (Exception e) { this.angle = -this.angle_adjust; }
        this.angle = (this.angle+(Math.PI*2)) % (Math.PI*2);

        int l_clicks = this.left_encoder.getCurrentPosition();
        int r_clicks = this.right_encoder.getCurrentPosition();

        double l_change_inches = (l_clicks - this.last_left_clicks) / this.CLICKS_PER_INCH;
        double r_change_inches = (r_clicks - this.last_right_clicks) / this.CLICKS_PER_INCH;
        //this.program.telemetry.addData("l_change_inches", l_change_inches);
        //this.program.telemetry.addData("r_change_inches", r_change_inches);

        double changed_angle = angle - last_angle;
        // if the robot goes from PI to -PI or vice versa, we need to adjust for the wrap-around.
        if (changed_angle > Math.PI) { changed_angle -= (Math.PI*2); }
        else if (changed_angle < -Math.PI) { changed_angle += (Math.PI*2); }
        //this.program.telemetry.addData("changed_angle", changed_angle);

        double average_change_inches = (l_change_inches + r_change_inches) / 2;
        //this.program.telemetry.addData("average_change_inches", average_change_inches);

        double line_traveled;
        if (changed_angle == 0) { // TODO: Decide whether or not to use the more complicated algorithm, and move the simpler to it's own method.
            line_traveled = average_change_inches;

        } else if (this.trackingAlgorithm == OdometryAlgorithm.Basic) {
            double right_wheel_motion_in_turning = changed_angle*RIGHT_DIS_FROM_CENTER;
            double left_wheel_motion_in_turning = changed_angle*LEFT_DIS_FROM_CENTER;

            r_change_inches -= right_wheel_motion_in_turning;
            l_change_inches += left_wheel_motion_in_turning;

            line_traveled = (l_change_inches + r_change_inches) / 2;

        } else {
            double radius = average_change_inches / changed_angle;
            this.program.telemetry.addData("radius", radius);

            double relative_heading = (Math.PI - changed_angle) / 2;
            this.program.telemetry.addData("relative_heading", relative_heading);

            line_traveled = Math.sin(changed_angle) * radius / Math.sin(relative_heading);
            this.program.telemetry.addData("line_traveled", line_traveled);
        }

        double change_X = Math.cos(angle) * line_traveled;
        double change_Y = Math.sin(angle) * line_traveled;
        this.program.telemetry.addData("change_X", change_X);
        this.program.telemetry.addData("change_Y", change_Y);

        this.x = this.last_x + change_X;
        this.y = this.last_y + change_Y;

        this.last_left_clicks = l_clicks;
        this.last_right_clicks = r_clicks;

        ArrayList<Double> list = new ArrayList<Double>();
        list.add(this.angle);
        list.add(this.x);
        list.add(this.y);
        return list;
    }

    /**
     * Tracks position while delaying (sleeping) for ms milliseconds.
     * @param ms The time in milliseconds to delay
     */
    public void odSleep(int ms) {
        double start = (int)System.currentTimeMillis();
        ArrayList<Double> list = new ArrayList<Double>();
        while (start+ms > (int)System.currentTimeMillis()) {
            this.getCurrentCoordinates();
            try {
                Thread.sleep(ms);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }
    }
}

enum OdometryAlgorithm {
    Basic,
    ArcBased
}
