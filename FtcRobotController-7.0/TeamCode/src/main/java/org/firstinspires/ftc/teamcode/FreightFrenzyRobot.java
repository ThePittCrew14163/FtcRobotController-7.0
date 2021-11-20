package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

/**
 * FTC Robot for the Freight Frenzy season
 *
 * For autonomous code, odometry (x,y) is (0,0) for the back corner of the robot's alliance's warehouse.
 */
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
    public Servo intakeFlap;
    public Orientation angles; // used to get info from BNO055IMU

    public DriveWheelOdometer odometer;

    private final double MIN_POINT_BUFFER_INCHES = 7;

    public final double MIN_DRIVE_BASE_TURN_POWER = 0.18;

    public final int INTAKE_HINGE_DOWN_CLICKS = 1000;
    public final int INTAKE_HINGE_UP_CLICKS = 0;

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
        intakeFlap = hardwareMap.get(Servo.class, "intakeFlap");

        SetDriveBaseZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheel4.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeHinge.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeHinge.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometer = new DriveWheelOdometer(0, 0, 0);
        odometer.init(imu, wheel1, wheel2, program);

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
        odometer.odSleep(ms);
        this.SetDriveBaseZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Has robot turn at speed power to heading degrees. degrees should be 180 >= degrees >= -180.
     * millis is a time limit on the method in milliseconds.
     * @param degrees Field-relative heading the robot is to turn to (0 is towards the carousels, 90 is towards the red side, -90 is towards the blue side)
     * @param speed motor power the robot is to turn at.
     * @param millis time limit in milliseconds for this method. If time runs out, the method just ends.
     */
    public void odTurn(double degrees, double speed, int millis) {
        odTurn(degrees, speed, millis, 0.008);
    }
    /**
     * Has robot turn at speed power to heading degrees. degrees should be 180 >= degrees >= -180.
     * millis is a time limit on the method in milliseconds.
     * @param degrees Field-relative heading the robot is to turn to (0 is towards the carousels, 90 is towards the red side, -90 is towards the blue side)
     * @param speed motor power the robot is to turn at.
     * @param millis time limit in milliseconds for this method. If time runs out, the method just ends.
     * @param adjustPower basically, how little the robot eases into the correct heading. adjustPower of > 0.05 is probably a LOT. Default is 0.008.
     */
    public void odTurn(double degrees, double speed, int millis, double adjustPower) {
        double difference, sign, correct, angle, final_speed, start = (int)System.currentTimeMillis();

        if (adjustPower < 0) { adjustPower = Math.abs(adjustPower);}
        if (adjustPower > 1) { adjustPower = 1;}

        this.SetDriveBaseRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArrayList<Double> list = odometer.getCurrentCoordinates();
        angle = Math.toDegrees(list.get(0));

        while (angle != degrees && !this.program.isStopRequested()) {
            list = odometer.getCurrentCoordinates();
            angle = Math.toDegrees(list.get(0));

            difference = angle-degrees;
            if (difference < 0) { sign = -1;
            } else { sign = 1; }

            // make sure that the robot turn the correct direction
            if (Math.abs(difference) > 180) {
                difference = sign * (360-Math.abs(difference));
                correct = -(difference)*adjustPower;
                if (Math.abs(correct) > 1) { correct = -sign; } // make sure that we don't correct so much that we give the motors greater power than speed.
            } else {
                correct = (difference)*adjustPower;
                if (Math.abs(correct) > 1) { correct = sign; } // make sure that we don't correct so much that we give the motors greater power than speed.
            }
            final_speed = speed*correct;
            if (Math.abs(final_speed) < MIN_DRIVE_BASE_TURN_POWER) {
                if (final_speed > 0) {final_speed = MIN_DRIVE_BASE_TURN_POWER;}
                else if (final_speed < 0) {final_speed = -MIN_DRIVE_BASE_TURN_POWER;}
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

    public void odMove(double speed, double x, double y) {
        odMove(speed, x, y, 0.02, 120000);
    }
    public void odMove(double speed, double x, double y, double adjustPower) {
        odMove(speed, x, y, adjustPower, 120000);
    }
    public void odMove(double speed, double x, double y, double adjustPower, int millis) {
        double start = (int)System.currentTimeMillis();

        if (speed > 1) { speed = 1;}
        if (speed < -1) { speed = -1;}
        ArrayList<Double> list = odometer.getCurrentCoordinates();

        double x_dis = x - list.get(1);
        double y_dis = y - list.get(2);
        double heading = Math.toDegrees(Math.atan2(y_dis, x_dis));
        if (speed < 0) { heading = (heading + 180)%360; }

        if (Math.abs(heading - list.get(0)) > 10) {
            int turn_ms = Math.min(millis, 700);
            this.odTurn(heading, Math.abs(speed), turn_ms);
        }

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

    public void motorTurnNoReset(double speed, int clicks, DcMotor motor) {
        // has motor turn clicks at speed. Not very complicated.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(clicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
    }
    public void motorTurn(double speed, int clicks, DcMotor motor) {
        // has motor turn clicks at speed. Not very complicated.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(clicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while (motor.isBusy() && !this.program.isStopRequested()) {}
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double PythagoreanTheorem(double a, double b) {
        return Math.sqrt(a*a + b*b);
    }
}
