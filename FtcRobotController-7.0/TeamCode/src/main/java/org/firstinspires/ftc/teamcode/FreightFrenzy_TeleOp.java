package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Freight Frenzy Drive")
public class FreightFrenzy_TeleOp extends LinearOpMode {
    public FreightFrenzyRobot robot = new FreightFrenzyRobot();

    // Drivetrain variables
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

    final double TSET_TURNSTILE_INCREMENT = 0.002;
    final double TSET_PIVOT_INCREMENT = 0.002;

    double intendedArmTurnstileAngle = adjustAngle;
    double intendedArmTurnstileClicks = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        // TODO: Set servo initial positions
        // odo pods up
        robot.xOdoPodLift.setPosition(0);
        robot.yOdoPodLift.setPosition(0);

        robot.TSET_Turnstile.setPosition(0.8);
        robot.TSET_Pivot.setPosition(0.5);
        robot.TSET_Extender.setPosition(0.5);

        // Start robot at starting position (x, y) // TODO: Decide where to start the robot coordinates in teleOp?
        robot.odometer.x = 0;
        robot.odometer.y = 0;


        // Set the adjusted robot heading
        if (Math.abs(gamepad2.right_stick_x) + Math.abs(gamepad2.right_stick_y) > 0.2) {
            adjustAngle = Math.atan2(gamepad2.right_stick_x, -gamepad2.right_stick_y) + Math.PI / 2;
        }
        adjustAngle = -(adjustAngle * 180) / Math.PI;

        // Wait for the game to start

        waitForStart();

        // run until the end of the match (driver presses STOP)
        OdometryPosition position;
        while (opModeIsActive()) {
            position = robot.odometer.getCurrentPosition();


            // ###############################################################
            //  ######      CONTROLS TO MAKE THE DRIVETRAIN MOVE.      ######
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



            // ###################################################################
            //  ###### OTHER GAMEPAD1 (A) CONTROLS (intake & duck spinner) ######
            if (gamepad1.left_bumper && (int)System.currentTimeMillis() - this.last_intake_switch >= intake_switch_delay) {
                run_intake = !run_intake;
                this.last_intake_switch = (int) System.currentTimeMillis();
            }
            if (gamepad1.right_bumper) {
                robot.intake.setPower(-1);
            } else if (!run_intake) {
                robot.intake.setPower(0);
            } else {
                robot.intake.setPower(1);
            }

            if (gamepad1.dpad_left) {
                robot.duckSpinner.setVelocity(-50);
            } else if (gamepad1.dpad_right) {
                robot.duckSpinner.setVelocity(50);
            } else {
                robot.duckSpinner.setVelocity(0);
            }

            // ##############################################################
            //  ######    GAMEPAD2 (B) CONTROLS (arm & TSE turret)    ######
            // TODO: Add controls for the arm
            //        right_stick_y -> power up and down
            //        left_stick -> field-centric turnstile turning (225-270 deg either way)
            //        left_trigger || right_trigger -> turn turnstile so arm is at the front of the robot
            robot.armHinge.setPower(gamepad2.right_stick_y);

            // 1- if g2.left_stick, set intended angle and from that and the robot's position derive intended clicks
            // 2- if g2.left_trigger or g2.right_trigger, set intended clicks to 0.
            // 3- turn arm to intended clicks (make sure that it turns the shortest way unless it would exceed ~250 degrees of rotation away from tthe initial position)


            if (gamepad2.left_bumper) {
                robot.TSET_Turnstile.setPosition(robot.TSET_Turnstile.getPosition() - TSET_TURNSTILE_INCREMENT);
            } else if (gamepad2.right_bumper) {
                robot.TSET_Turnstile.setPosition(robot.TSET_Turnstile.getPosition() + TSET_TURNSTILE_INCREMENT);
            }

            if (gamepad2.dpad_left) {
                robot.TSET_Pivot.setPosition(robot.TSET_Pivot.getPosition() - TSET_PIVOT_INCREMENT);
            } else if (gamepad2.dpad_right) {
                robot.TSET_Pivot.setPosition(robot.TSET_Pivot.getPosition() + TSET_PIVOT_INCREMENT);
            }

            if (gamepad2.a) {
                robot.TSET_Extender.setPosition(0);
            } else if (gamepad2.y) {
                robot.TSET_Extender.setPosition(1);
            } else {
                robot.TSET_Extender.setPosition(0.5);
            }

            telemetry.addData("Odo-given X", position.x);
            telemetry.addData("Odo-given Y", position.y);
            /////////////////     UPDATE TELEMETRY     /////////////////
            telemetry.update();
        }
    }
}
