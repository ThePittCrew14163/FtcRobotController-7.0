package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Duck side (Red or Blue)")
public class FreightFrenzy_DuckSideAuto extends LinearOpMode {
    public FreightFrenzyRobot robot = new FreightFrenzyRobot();
    Alliance side = Alliance.Red;
    int s = 1;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        if (gamepad1.x || gamepad2.x) {
            side = Alliance.Blue;
            s = 1;
        } else if (gamepad1.b || gamepad2.b) {
            side = Alliance.Red;
            s = -1;
        }
        robot.odometer.x = 9*s;
        robot.odometer.y = 96;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            // TODO: Implemet vision software
            //telemetry.addData("position", positionOfTeamElement);
            telemetry.update();
        }


    }
}
