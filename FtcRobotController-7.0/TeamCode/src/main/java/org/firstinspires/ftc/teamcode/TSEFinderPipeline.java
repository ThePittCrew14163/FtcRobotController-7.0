package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TSEFinderPipeline extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public TSE_Position position = TSE_Position.CENTER;
    private int leftBoxX = 70;
    private int centerBoxX = 230;
    private int rightBoxX = 370;
    private int boxWidth = 110;
    private int boxHeight = 70;
    private int boxY = 330;

    private double leftTotal, centerTotal, rightTotal;

    public TSEFinderPipeline() {
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(boxY, boxY+boxHeight, leftBoxX, leftBoxX+boxWidth);
        Mat matCenter = workingMatrix.submat(boxY, boxY+boxHeight, centerBoxX, centerBoxX+boxWidth);
        Mat matRight = workingMatrix.submat(boxY, boxY+boxHeight, rightBoxX, rightBoxX+boxWidth);

        Imgproc.rectangle(workingMatrix, new Rect(leftBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(centerBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));
        Imgproc.rectangle(workingMatrix, new Rect(rightBoxX, boxY, boxWidth, boxHeight), new Scalar(0,255,0));

        // look for yellowest area (the Y of YCrCb)
        leftTotal = Core.sumElems(matLeft).val[0];
        centerTotal = Core.sumElems(matCenter).val[0];
        rightTotal = Core.sumElems(matRight).val[0];

        if (leftTotal > centerTotal) {
            if (leftTotal > rightTotal) {
                // team shipping element is to the left
                position = TSE_Position.LEFT;
            } else {
                // team shipping element is to the right
                position = TSE_Position.RIGHT;
            }
        } else {
            if (centerTotal > rightTotal) {
                // team shipping element is in the center
                position = TSE_Position.CENTER;
            } else {
                // team shipping element is to the right
                position = TSE_Position.RIGHT;
            }
        }

        return workingMatrix;
    }
}

