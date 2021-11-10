package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp(name="Freight Frenzy Drive")
public class FreightFrenzyTeleOp extends LinearOpMode {
    public FreightFrenzyRobot robot = new FreightFrenzyRobot();
    double intendedRobotHeading = 0;  // heading (in radians) robot is to maintain. is set by gamepad1.right_stick.
    double adjustAngle = 0;
    final double MAX_HEADING_ERROR = Math.PI/18;
    final double ADJUST_TURNING_POWER = 1;
    final double ADJUST_DRIVING_POWER = 0.5;

    int intake_switch_delay = 500; // delay in ms
    int last_intake_switch = (int)System.currentTimeMillis();
    public boolean run_intake = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        // TODO: Set servo initial positions

        //Start robot // TODO: Decide where to start the robot
        robot.odometer.x = 0;
        robot.odometer.y = 0; //TODO: fix/finish odometer for this season

        if (Math.abs(gamepad2.right_stick_x) + Math.abs(gamepad2.right_stick_y) > 0.2) {
            adjustAngle = Math.atan2(-gamepad2.right_stick_y, gamepad2.right_stick_x) + Math.PI / 2;
        }
        intendedRobotHeading = adjustAngle;
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

            telemetry.addData("Angle", list.get(0));
            telemetry.addData("X value", list.get(1));
            telemetry.addData("Y value", list.get(2));

            telemetry.addData("Left power", -gamepad1.left_stick_y);
            telemetry.addData("Right power", -gamepad1.left_stick_y);

            //////////////////// GAMEPAD 1 ///////////////

              //////////////// SETS THE HEADING
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y) > 0.6) {
                intendedRobotHeading = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2 + adjustAngle;
                intendedRobotHeading %= (Math.PI*2);
            }

            if (Math.abs(list.get(0) - intendedRobotHeading) > MAX_HEADING_ERROR) {
                int sign;
                double correct;
                double difference = list.get(0)-intendedRobotHeading;
                if (difference < 0) { sign = -1;
                } else { sign = 1; }

                // make sure that the robot turn the correct direction
                if (Math.abs(difference) > Math.PI) {
                    difference = sign * (Math.PI*2 -Math.abs(difference));
                    correct = -(difference)* ADJUST_TURNING_POWER;
                    if (Math.abs(correct) > 1) { correct = -sign; } // make sure that we don't correct so much that we give the motors greater power than speed.
                } else {
                    correct = (difference)* ADJUST_TURNING_POWER;
                    if (Math.abs(correct) > 1) { correct = sign; } // make sure that we don't correct so much that we give the motors greater power than speed.
                }
                if (Math.abs(correct) < robot.MIN_DRIVE_BASE_TURN_POWER) {
                    if (correct > 0) {correct = robot.MIN_DRIVE_BASE_TURN_POWER;}
                    else if (correct < 0) {correct = -robot.MIN_DRIVE_BASE_TURN_POWER;}
                }
                robot.wheel2.setPower(-correct);
                robot.wheel4.setPower(-correct);
                robot.wheel1.setPower(correct);
                robot.wheel3.setPower(correct);
            }
            else {
                double power = -gamepad1.left_stick_y, adjust = 0;
                if (power != 0) {
                    adjust = Math.abs(power)*(list.get(0) - intendedRobotHeading)*ADJUST_DRIVING_POWER;
                }
                robot.wheel2.setPower(power-adjust);
                robot.wheel4.setPower(power-adjust);
                robot.wheel1.setPower(power+adjust);
                robot.wheel3.setPower(power+adjust);
            }

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
