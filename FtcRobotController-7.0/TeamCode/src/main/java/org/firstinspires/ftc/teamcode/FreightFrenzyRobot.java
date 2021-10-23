package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

class FreightFrenzyRobot {
    public BNO055IMU imu;
    public DcMotor wheel1;
    public DcMotor wheel2;
    public DcMotor wheel3;
    public DcMotor wheel4;
    public DcMotor intake;
    public DcMotor intakeHinge;
    public DcMotor carouselTurner;
    public DcMotor lift;
    public Servo dispenserPivot;
    public Servo dispenserFlap;
    public Servo teamElementArm;
    public Servo teamElementClaw;
    public Orientation angles; // used to get info from BNO055IMU

    public DriveWheelOdometer odometer;

    private final double MIN_POINT_BUFFER_INCHES = 7;

    private LinearOpMode program; // the program using this module.  Robot requires access to the program to know when the program is trying to stop.

    public void init(HardwareMap hardwareMap, LinearOpMode program) {
        // SET UP IMU AS BNO055IMU:
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        wheel1 = hardwareMap.get(DcMotor.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotor.class, "wheel2");
        wheel3 = hardwareMap.get(DcMotor.class, "wheel3");
        wheel4 = hardwareMap.get(DcMotor.class, "wheel4");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeHinge = hardwareMap.get(DcMotor.class, "intakeHinge");
        carouselTurner = hardwareMap.get(DcMotor.class, "carouselTurner");
        lift = hardwareMap.get(DcMotor.class, "lift");
        dispenserPivot = hardwareMap.get(Servo.class, "dispenserPivot");
        dispenserFlap = hardwareMap.get(Servo.class, "dispenserFlap");
        teamElementArm = hardwareMap.get(Servo.class, "teamElementArm");
        teamElementClaw = hardwareMap.get(Servo.class, "teamElementClaw");

        SetDriveBaseZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheel4.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.program = program;
    }


    public void SetDriveBaseRunMode(DcMotor.RunMode runMode) {
        wheel1.setMode(runMode);
        wheel2.setMode(runMode);
        wheel3.setMode(runMode);
        wheel4.setMode(runMode);
    }

    public void SetDriveBaseZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        wheel1.setZeroPowerBehavior(behavior);
        wheel2.setZeroPowerBehavior(behavior);
        wheel3.setZeroPowerBehavior(behavior);
        wheel4.setZeroPowerBehavior(behavior);
    }

    /**
     * Makes the drivetrain forcefully stop.
     * After the method is over the drivetrain ZeroPowerBehavior is left on FLOAT
     * @param ms How long to spend forcing the robot to stop.
     */
    public void BrakeRobot(int ms) {
        this.SetDriveBaseZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel1.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
        wheel4.setPower(0);
        this.odometer.odSleep(ms);
        this.SetDriveBaseZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Has robot turn at speed power to heading degrees. degrees should be 180 >= degrees >= -180.
     * millis is a time limit on the method in milliseconds.
     * @param degrees
     * @param speed
     * @param millis
     */
    public void odTurn(double degrees, double speed, int millis) {
        double difference, sign, correct, final_speed, start = (int)System.currentTimeMillis();
        this.SetDriveBaseRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double angle;
        ArrayList<Double> list = odometer.getCurrentCoordinates();
        angle = Math.toRadians(list.get(0));
        while (angle != degrees && !this.program.isStopRequested()) {
            list = odometer.getCurrentCoordinates();
            angle = Math.toRadians(list.get(0));
            difference = angle-degrees;
            if (difference < 0) { sign = -1;
            } else { sign = 1; }

            // make sure that the robot turn the correct direction
            if (Math.abs(difference) > 180) {
                difference = sign * (360-Math.abs(difference));
                correct = -(difference)/120;
                if (Math.abs(correct) > 1) { correct = -sign; } // make sure that we don't correct so much that we give the motors greater power than speed.
            } else {
                correct = (difference)/120;
                if (Math.abs(correct) > 1) { correct = sign; } // make sure that we don't correct so much that we give the motors greater power than speed.
            }
            final_speed = speed*correct;
            if (Math.abs(final_speed) < 0.22) {
                if (final_speed > 0) {final_speed = 0.22;}
                if (final_speed < 0) {final_speed = -0.22;}
            }
            wheel2.setPower(-final_speed);
            wheel4.setPower(-final_speed);
            wheel1.setPower(final_speed);
            wheel3.setPower(final_speed);
            if (start+millis < (int)System.currentTimeMillis()) {
                break;
            }
        }
        wheel2.setPower(0);
        wheel4.setPower(0);
        wheel1.setPower(0);
        wheel3.setPower(0);
    }

    public void odStrafe(double speed, double x, double y) {
        odStrafe(speed, x, y, 0.02, 120000);
    }
    public void odStrafe(double speed, double x, double y, double adjustPower) {
        odStrafe(speed, x, y, adjustPower, 120000);
    }
    public void odStrafe(double speed, double x, double y, double adjustPower, int millis) {
        double start = (int)System.currentTimeMillis();

        if (speed > 1) { speed = 1;}
        if (speed < -1) { speed = -1;}
        ArrayList<Double> list = odometer.getCurrentCoordinates();

        double x_dis = x - list.get(1);
        double y_dis = y - list.get(2);
        double heading = Math.toDegrees(Math.atan2(y_dis, x_dis));
        if (speed < 0) { heading = (heading + 180)%360; }

        int turn_ms = millis > 700 ? 700 : millis;
        this.odTurn(heading, Math.abs(speed), turn_ms);

        list = odometer.getCurrentCoordinates();
        double distance_to_go = PythagoreanTheorem(x - list.get(1), y - list.get(2));
        while (distance_to_go > this.MIN_POINT_BUFFER_INCHES && this.program.isStopRequested() == false) {
            list = odometer.getCurrentCoordinates();
            x_dis = x - list.get(1);
            y_dis = y - list.get(2);
            heading = Math.toDegrees(Math.atan2(y_dis, x_dis));
            distance_to_go = PythagoreanTheorem(x - list.get(1), y - list.get(2));

            double correct_steering_power = (heading - list.get(0)) * adjustPower;

            // correction never overwhelms the drive speed.
            if (Math.abs(correct_steering_power) > Math.abs(speed)) {
                if (correct_steering_power > 0) {
                    correct_steering_power = Math.abs(speed);
                } else {
                    correct_steering_power = - Math.abs(speed);
                }
            }

            wheel1.setPower(speed+correct_steering_power);
            wheel3.setPower(speed+correct_steering_power);
            wheel2.setPower(speed-correct_steering_power);
            wheel4.setPower(speed-correct_steering_power);

            if (start+millis < (int)System.currentTimeMillis()) {
                break;
            }
        }
        wheel2.setPower(0);
        wheel4.setPower(0);
        wheel1.setPower(0);
        wheel3.setPower(0);
    }

    private double PythagoreanTheorem(double a, double b) {
        return Math.sqrt(a*a + b*b);
    }
}
