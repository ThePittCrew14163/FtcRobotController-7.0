/*
Copyright 2019 FIRST Tech Challenge Team 14163
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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

public class DriveWheelOdometer {
    public double angle;
    public double angle_adjust;
    public double x;
    public double y;
    public final double CLICKS_PER_INCH = 8192 / (2.362 * Math.PI);
    public final double X_DIS_FROM_CENTER = 0; //4; // distance from the x wheel to the center of the robot.
    public final double Y_DIS_FROM_CENTER = 0; //2.5; // distance from the y wheel to the center of the robot.
    public DriveWheelOdometer(double angle, double x, double y) {
        this.angle = angle;
        this.x = x;
        this.y = y;
    }
    public BNO055IMU imu;
    public DcMotor y_encoder;
    public DcMotor x_encoder;

    public double last_angle;
    public double last_x;
    public double last_y;

    public double last_x_clicks = 0;
    public double last_y_clicks = 0;

    public void init(BNO055IMU imu, DcMotor y_encoder, DcMotor x_encoder) {
        this.imu = imu;
        this.y_encoder = y_encoder;
        this.x_encoder = x_encoder;
        this.angle_adjust = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - this.angle;
        this.y_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.x_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.y_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.x_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public ArrayList<Double> getCurrentCoordinates() {
        // returns the current angle, x and y coordinates relative to where the robot started.
        // x and y are in inches.

        // save previous coordinates
        this.last_angle = this.angle;
        this.last_x = this.x;
        this.last_y = this.y;

        // get distance travelled by each wheel since last check and then save current clicks for each wheel.
        double yc = this.y_encoder.getCurrentPosition();
        double hy = (this.last_y_clicks - yc) / this.CLICKS_PER_INCH;
        this.last_y_clicks = yc;
        double xc = this.x_encoder.getCurrentPosition(); // xc = x wheel clicks
        double hx = (this.last_x_clicks - xc) / this.CLICKS_PER_INCH;  // hx = distance traveled in the last frame by the x wheel
        this.last_x_clicks = xc;
        // get new angle
        try { this.angle = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - this.angle_adjust; }
        catch (Exception e) { this.angle = -this.angle_adjust; }

        double dis = hy, theta = 0;
        // calculate new position
        // first check if the robot hasn't turned:
        if (this.last_angle - this.angle == 0 || this.last_angle - this.angle == 360 || this.last_angle - this.angle == -360) {
            this.x = last_x - (hy * Math.cos((angle+90)*Math.PI/180) + hx * Math.cos(angle*Math.PI/180));
            this.y = last_y - (hy * Math.sin((angle+90)*Math.PI/180) + hx * Math.sin(angle*Math.PI/180));

        } else {
            // the robot has turned and more complicated calculations are necessary.
            double sign;
            theta = (this.angle - this.last_angle)*Math.PI/180;
            // if the robot goes from 179 to -180 or vice versa, we need to adjust for the wrap-around.
            if (theta < 0) { sign = -1; } else { sign = 1; }
            if (Math.abs(theta) > Math.PI) { theta = sign * (Math.PI*2 - Math.abs(theta)); }

            hx += theta*X_DIS_FROM_CENTER;  // if the x wheel is not at the center of the robot, some of its movement is actually part of the turn.
            hy -= theta*Y_DIS_FROM_CENTER;
            this.x = last_x - (hy * Math.cos((angle+90)*Math.PI/180-(theta/2)) + hx * Math.cos(angle*Math.PI/180-(theta/2)));
            this.y = last_y - (hy * Math.sin((angle+90)*Math.PI/180-(theta/2)) + hx * Math.sin(angle*Math.PI/180-(theta/2)));
        }
        ArrayList<Double> list = new ArrayList<Double>();
        list.add(this.angle);
        list.add(this.x);
        list.add(this.y);
        list.add(hx);
        list.add(hy);
        list.add(dis);
        list.add(theta);
        list.add(theta*X_DIS_FROM_CENTER);
        return list;
    }
    public void odSleep(int ms) {
        // tracks position while sleeping for ms milliseconds.
        double start = (int)System.currentTimeMillis();
        ArrayList<Double> list = new ArrayList<Double>();
        while (start+ms > (int)System.currentTimeMillis()) {
            list = this.getCurrentCoordinates();
            try {
                Thread.sleep(ms);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
        }
    }
    ///////////////// problematic turning calculation code (I never figured out how to make it work) ///////////////////////////////////
    //
    //        // the robot has turned and more complicated calculations are necessary.
    //        double sign, theta = (this.angle - this.last_angle)*Math.PI/180;
    //        // if the robot goes from 179 to -180 or vice versa, we need to adjust for the wrap-around.
    //        if (theta < 0) { sign = -1; } else { sign = 1; }
    //        if (Math.abs(theta) > Math.PI) { theta = sign * (Math.PI-Math.abs(theta)); }
    //        double r = (hy / theta) - Y_DIS_FROM_CENTER;
    //        hx = -hx;
    //        hx -= theta*X_DIS_FROM_CENTER;  // if the x wheel is not at the center of the robot, some of its movement is actually part of the turn.
    //        // the arc calculations on paper make the shape of a piece of pizza. Thus, many pizza geometry references are made in the next few lines of code.
    //        double _angle = (this.last_angle-180)*Math.PI/180; // this is used a lot so it is just calculated once, here.
    //        double pizza_x = Math.cos(_angle) * r;  // get coordinates of pizza point/tip relative to last position.
    //        double pizza_y = Math.sin(_angle) * r;
    //        double pizza_angle = _angle-theta; // calculate he angle the tip of the pizza makes.
    //        this.x = -(Math.cos(pizza_angle) * (r-hx)) + pizza_x + this.last_x;  // calculate coordinates of the other pizza corner,
    //        this.y = -(Math.sin(pizza_angle) * (r-hx)) + pizza_y + this.last_y;  // which is the new robot position.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
