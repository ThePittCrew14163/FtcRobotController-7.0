package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

    LinearOpMode program; // the program using this module.  Robot requires access to the program to know when the program is trying to stop.

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

        DriveBaseSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheel4.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.program = program;
    }

    public void DriveBaseSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
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
        this.DriveBaseSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel1.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
        wheel4.setPower(0);
        this.odometer.odSleep(ms);
        this.DriveBaseSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
