package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Warehouse side (Red or Blue)")
public class FreightFrenzy_WarehouseSideAuto extends FreightFrenzy_BaseAutoSetup {

    @Override
    public void initOdometryCoordinates() {
        robot.odometer.x = 9 * this.s;
        robot.odometer.y = 106;
    }

    @Override
    public void runRobotAuto() {

        // score pre-loaded freight

        // loop until robot has less than 4-5 seconds {
        //     align with wall and go into warehouse
        //     dig into the freight pile until distance sensor detects freight or until ~3 seconds are up
        //     drive out and score that freight
        // }

        // park

    }
}
