package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

@TeleOp(name="Freight Frenzy Drive")
public class FreightFrenzyTeleOp extends LinearOpMode {
    public FreightFrenzyRobot robot = new FreightFrenzyRobot();
    double robotAngle = 0;  // heading (in degrees) robot is to maintain. is set by gamepad1.right_stick.
    double leftStickAngle = 0;
    double theta;  // difference between robot heading and the direction it should travel towards.
    double leftStickR = 0; // distance from 0-1 from gamepad1.left_stick center to edge. is used to set power level to drive train motors.
    double speed;  // speed adjustment for robot to turn towards robotAngle.
    double difference;
    double sign;
    double theta;

    int intake_switch_delay = 500; // delay in ms
    int last_intake_switch = (int)System.currentTimeMillis();
    public boolean run_intake = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        double adjustAngle = 0;
        // TODO: Set servo initial positions

        //Start robot // TODO: Decide where to start the robot
        robot.odometer.x = 0;
        robot.odometer.y = 0; //TODO: fix/finish odometer for this season

        if (Math.abs(gamepad2.right_stick_x) + Math.abs(gamepad2.right_stick_y) > 0.2) {
            adjustAngle = Math.atan2(gamepad2.right_stick_x, -gamepad2.right_stick_y) + Math.PI / 2;
        }
        robotAngle = -(adjustAngle * 180) / Math.PI;
        telemetry.addData("Status", "Initialized. Please do something already.");
        telemetry.addData("adjustAngle", (adjustAngle * 180) / Math.PI);
        telemetry.update();

        // Wait for the game to start

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ArrayList<Double> list = robot.odometer.getCurrentCoordinates();





            // #######################################################
            //  ###### CONTROLS TO MAKE THE DRIVE TRAIN MOVE. ######
            // TODO: Replace these controls wih field-centric driving
            //  (we can't go straight forward reliably with the current controls)
            robot.wheel1.setPower(-gamepad1.left_stick_y);
            robot.wheel3.setPower(-gamepad1.left_stick_y);
            robot.wheel2.setPower(-gamepad1.left_stick_y);
            robot.wheel4.setPower(-gamepad1.left stick_y);

            telemetry.addData("Angle", list.get(0));
            telemetry.addData("X value", list.get(1));
            telemetry.addData("Y value", list.get(2));

            telemetry.addData("Left power", -gamepad1.left_stick_y);
            telemetry.addData("Right power", -gamepad1.left_stick_y);

            //////////////////// GAMEPAD 1 ///////////////

            /////////////////// Drive Controls/////////////


            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y) > 0.6) {
                robotAngle = (-Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y) * 180 / Math.PI) - 90;
                if (robotAngle <= -180) {
                    robotAngle += 360;
                }
                if (robotAngle >= 180) {
                    robotAngle -= 360;
                }
            }
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                leftStickAngle = -Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y) + (Math.PI * 5 / 4);
                if (leftStickAngle >= Math.PI) {
                    leftStickAngle -= Math.PI * 2;
                }
                theta = robotAngle / 180 * Math.PI - leftStickAngle;
                xWheelsPower = Math.cos(theta);
                yWheelsPower = Math.sin(theta);
            } else {
                xWheelsPower = 0;
                yWheelsPower = 0;
            }
            difference = robot.angles.firstAngle - robotAngle - (adjustAngle * 180) / Math.PI;
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

            //////////// INTAKE CONTROLS ///////////


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

            if (gamepad1.right_trigger > 0.1) {
                robot.motorTurnNoReset(1, robot.INTAKE_HINGE_DOWN_CLICKS, robot.intakeHinge);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.motorTurnNoReset(1, robot.INTAKE_HINGE_UP_CLICKS, robot.intakeHinge);
            }

            //////////////////// GAMEPAD 2 ///////////////
            robot.carouselTurner.setPower(gamepad2.left_stick_y);

            if (gamepad2.left_trigger > 0){
                robot.lift.setPower(gamepad2.left_trigger);
            }
            else {
                robot.lift.setPower(-gamepad2.right_trigger);
            }

            ////// UPDATE TELEMETRY //////
            telemetry.update();
        }
    }
}
