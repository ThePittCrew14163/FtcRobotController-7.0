package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * odometry (x,y) is (0,0) for the back corner of the robot's alliance's warehouse.
 */
public class FreightFrenzy_BaseAutoSetup extends LinearOpMode {
    public FreightFrenzyRobot robot = new FreightFrenzyRobot();
    Alliance side = Alliance.Red;
    /**
     * 1 if side == Blue,
     * -1 if side == Red
     */
    int s = 1; // 1 is default value

    OpenCvInternalCamera phoneCam;
    TSEFinderPipeline pipeline;
    TSE_Position tse_position;

    /**
     * Start robot // TODO: decide where to start robot for duck-side auto
     */
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

        telemetry.addData("Alliance", side.toString());
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new TSEFinderPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // supported camera resolutions (for the gold E4) are: 1280x720, 960x720, 768x432, 720x480, 640x480, 320x240, 176x144
                phoneCam.startStreaming(720, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int i) {
                // display given error because I don't know what else to do
                telemetry.addData("Error integer", i);
            }
        });

        initOdometryCoordinates();

        // Set servo initial positions
        robot.setOdoPodsDown();
        robot.setTSETtoInitPosition();

        while (!isStarted() && !isStopRequested()) {
            tse_position = pipeline.tse_position;
            telemetry.addData("Left yellow", pipeline.leftTotal);
            telemetry.addData("Center yellow", pipeline.centerTotal);
            telemetry.addData("Right yellow", pipeline.rightTotal);
            telemetry.addData("TSE position", pipeline.tse_position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping
            robot.odometer.odSleep(50);
        }

        // duck-side or warehouse side or other auto will run here
        runRobotAuto();

        // make sure that robot pulls up those pods at the end // TODO: should this not be done until teleOp?
        robot.setOdoPodsUp();
    }

    /**
     * Inheriting autonomous classes override this method to add the code for the 30 seconds of robot movement
     */
    public void runRobotAuto() {
    }

    /**
     * Inheriting autonomous classes override this method to set odometry coordinates       <br>
     * Note that odometry (x,y) is (0,0) for the back corner of the robot's alliance's warehouse.
     *      <br><br>
     * Note that this.s == -1 if side == Red, and this.s == 1 if side == Blue               <br>
     * Note that this.Alliance is properly set by the time this method is called
     *      <br><br>
     * example use:                <br>
     *    robot.odometer.x = 9*s;  <br>
     *    robot.odometer.y = 96;
     */
    public void initOdometryCoordinates() {
        robot.odometer.x = 9*s;
        robot.odometer.y = 96;
    }
}
