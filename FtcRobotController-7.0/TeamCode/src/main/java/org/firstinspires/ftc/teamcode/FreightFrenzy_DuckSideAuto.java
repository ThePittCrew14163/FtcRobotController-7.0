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
        robot.odometer.x = 103 * this.s;
        robot.odometer.y = 16;
    }

    @Override
    public void runRobotAuto() {
        if (this.side == Alliance.Red) {
            this.redSideDuckAuto();
        } else {
            this.blueSideDuckAuto();
        }
    }

    public void redSideDuckAuto() {
        // drive around TSE and score pre-loaded freight
        robot.intake.setPower(robot.INTAKE_ON_POWER);
        robot.odStrafe(0, 0.6, -110, 20, 4, 1000, 0.05, false);
        robot.motorTurnNoReset(0.5, robot.ARM_HINGE_UP_CLICKS, robot.armHinge);
        robot.odStrafe(0, 1, -132, 22, 6, 2000, 0.05, false);
        robot.motorTurnNoReset(0.5, (int)robot.ARM_TURNSTILE_CLICKS_PER_DEG*180, robot.armTurnstile);
        robot.odStrafe(0, 1, -130, 55, 6, 2000, 0.05, false);
        robot.odTurn(-90, 1, 800, 0.008, false);
        if (this.tse_position == TSE_Position.LEFT) { // bottom tray
            robot.motorTurnNoReset(0.5, 350, robot.armHinge);
            robot.odStrafe(-90, 1, -110, 50, 6, 1500, 0.05, false);
            robot.odStrafe(-90, 0.3, -104, 50, 3, 1000);

        } else if (this.tse_position == TSE_Position.CENTER) { // bottom tray
            robot.motorTurnNoReset(0.5, 800, robot.armHinge);
            robot.odStrafe(-90, 1, -108, 50, 6, 1500, 0.05, false);
            robot.odStrafe(-90, 0.3, -102, 50, 3, 1000);

        } else { // top tray
            robot.motorTurnNoReset(0.5, 1350, robot.armHinge);
            robot.odStrafe(-90, 1, -106, 50, 6, 1500, 0.05, false);
            robot.odStrafe(-90, 0.3, -100, 50, 3, 1000);

        }

        // spin carousel

        // pick up duck TODO: make duck-finding vision software

        // score duck

        // park

    }

    public void blueSideDuckAuto() {
        // score pre-loaded freight
        if (this.tse_position == TSE_Position.LEFT) {

        } else if (this.tse_position == TSE_Position.CENTER) {

        } else {

        }

        // spin carousel

        // pick up duck TODO: make duck-finding vision software

        // score duck

        // park

    }
}
