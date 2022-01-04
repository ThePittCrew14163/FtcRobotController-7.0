package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="  Duck side    (Red or Blue)")
public class FreightFrenzy_DuckSideAuto extends FreightFrenzy_BaseAutoSetup {

    @Override
    public void initOdometryCoordinates() {
        robot.odometer.x = 9 * this.s;
        robot.odometer.y = 96;
    }

    @Override
    public void runRobotAuto() {

        // score pre-loaded freight

        // spin carousel

        // pick up duck TODO: make duck-finding vision software

        // score duck

        // park

    }
}
