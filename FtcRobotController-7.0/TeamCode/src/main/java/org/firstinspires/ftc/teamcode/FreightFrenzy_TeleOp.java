package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

@TeleOp(name="Freight Frenzy Drive")
public class FreightFrenzy_TeleOp extends LinearOpMode {
    public FreightFrenzyRobot robot = new FreightFrenzyRobot();

    double adjustAngle = 0;
    double leftStickAngle = 0;
    double theta;  // difference between robot heading and the direction it should travel towards.
    double leftStickR = 0; // distance from 0-1 from gamepad1.left_stick center to edge. is used to set power level to drive train motors.
    double xWheelsPower; // wheel 2 and 3 power
    double yWheelsPower; // wheel 1 and 4 power
    double speed;  // speed adjustment for robot to turn towards robotAngle.
    double difference;
    double sign;

    /**
     * delay in milliseconds
     */
    int intake_switch_delay = 500;
    int last_intake_switch = (int)System.currentTimeMillis();
    public boolean run_intake = false;
    
    /**
     * delay in milliseconds
     */
    int dispenser_flap_switch_delay = 400;
    int last_dispenser_flap_switch = last_intake_switch;

    boolean dispenserIsTurnedToTheRight = false;
    
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        // TODO: Set servo initial positions

        //Start robot // TODO: Decide where to start the robot
        robot.odometer.x = 0;
        robot.odometer.y = 0;

        if (Math.abs(gamepad2.right_stick_x) + Math.abs(gamepad2.right_stick_y) > 0.2) {
            adjustAngle = Math.atan2(gamepad2.right_stick_x, -gamepad2.right_stick_y) + Math.PI / 2;
        }
        adjustAngle = -(adjustAngle * 180) / Math.PI;

        // Wait for the game to start

        waitForStart();

        // run until the end of the match (driver presses STOP)
        ArrayList<Double> list;
        while (opModeIsActive()) {
            list = robot.odometer.getCurrentCoordinates();// #######################################################
            //  ###### CONTROLS TO MAKE THE DRIVE TRAIN MOVE. ######
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            // now use right stick input to get the robot's heading. the if statement ensures that the joystick is a distance away from the center where readings will be accurate.
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y) > 0.6) {
                adjustAngle = (-Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y) * 180 / Math.PI) - 90;
                if (adjustAngle <= -180) {
                    adjustAngle += 360;
                }
                if (adjustAngle >= 180) {
                    adjustAngle -= 360;
                }
            }
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                leftStickAngle = -Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y) + (Math.PI * 5 / 4);
                if (leftStickAngle >= Math.PI) {
                    leftStickAngle -= Math.PI * 2;
                }
                theta = adjustAngle / 180 * Math.PI - leftStickAngle;
                xWheelsPower = Math.cos(theta);
                yWheelsPower = Math.sin(theta);
            } else {
                xWheelsPower = 0;
                yWheelsPower = 0;
            }
            difference = robot.angles.firstAngle - adjustAngle - (this.adjustAngle * 180) / Math.PI;
            if (Math.abs(difference) > 180) {
                if (difference < 0) {
                    sign = -1;
                } else {
                    sign = 1;
                }
                difference = sign * (360 - Math.abs(difference));
                speed = -(difference) / 80;
            } else {
                speed = (difference) / 80;
            }

            leftStickR = Math.sqrt((Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2))) * 1.42;

            // this code here ensures that the robot can turn without having to strafe or sit there making a whining noise.
            double speed1 = speed, speed2 = speed; // speed1 is for the front wheels 1 and 2, and speed2 is for the back wheels 3 and 4.

            if (0.1 <= Math.abs(speed) && Math.abs(speed) < 0.2) {
                // only the back wheels move, meaning that the robot can turn but at a lower speed.
                speed2 *= 2;
                speed1 = 0;
            } else if (Math.abs(speed) < 0.1) {
                // at a certain threshold you'll get no movement, but the motors will whine. thus, it's best to just stop them.
                speed1 = 0;
                speed2 = 0;
            }
            robot.wheel1.setPower(yWheelsPower * leftStickR + speed1);
            robot.wheel4.setPower(yWheelsPower * leftStickR - speed2);
            robot.wheel2.setPower(xWheelsPower * leftStickR - speed1);
            robot.wheel3.setPower(xWheelsPower * leftStickR + speed2);

            // TODO: Add controls for all of the other parts of the robot

            telemetry.addData("Odo-given X", list.get(1));
            telemetry.addData("Odo-given Y", list.get(2));
            ////// UPDATE TELEMETRY //////
            telemetry.update();
        }
    }
}
