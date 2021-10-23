package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

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

            // #######################################################
            //  ###### CONTROLS TO MAKE THE DRIVE TRAIN MOVE. ######
            robot.wheel1.setPower(-gamepad1.left_stick_y);
            robot.wheel3.setPower(-gamepad1.left_stick_y);
            robot.wheel2.setPower(-gamepad1.right_stick_y);
            robot.wheel4.setPower(-gamepad1.right_stick_y);

            ArrayList<Double> list = robot.odometer.getCurrentCoordinates();
            telemetry.addData("Angle", list.get(0));
            telemetry.addData("X value", list.get(1));
            telemetry.addData("Y value", list.get(2));

            telemetry.addData("Left power", -gamepad1.left_stick_y);
            telemetry.addData("Right power", - gamepad1.right_stick_y);

            //////////////////// GAMEPAD 1 ///////////////
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
