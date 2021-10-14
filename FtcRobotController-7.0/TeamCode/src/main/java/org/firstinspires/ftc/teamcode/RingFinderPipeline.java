package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RingFinderPipeline extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    private ArrayList<Mat> boxes = new ArrayList<Mat>();
    public int[] positions = {0, 0}; // up to two rings to get from a position

    private int numBoxes = 14;
    public double[] threshholds = new double[numBoxes]; // the threshholds for each box

    private int boxWidth = 34;
    private int boxHeight = 130;
    private int boxArea = boxWidth*boxHeight;
    private int startX = 2;
    private int startY = 498;
    private int endY = 435;
    private int incrementY = (endY-startY)/numBoxes;

    private double isRingThreshhold = 149;
    public String s = "nothing so far";

    public double cameraDegreeRange = 43;
    public double cameraDegreeOffset = 10;


    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }
        s = "";
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        // Sets up boxes for looking around the field
        boxes.clear();
        for (int i = 0; i < numBoxes; i++){
            // workingMatrix.submat(rowStart, rowEnd, colStart, colEnd) row is y col is x
            int colStart = startX+(boxWidth*i);
            int colEnd = colStart + boxWidth;
            int rowStart = startY + (incrementY*i);
            int rowEnd = rowStart + boxHeight;
            try {
                Mat newMat = workingMatrix.submat(rowStart, rowEnd, colStart, colEnd);
                boxes.add(newMat);
            } catch (Exception e) {
                this.s += "Bad "+i;
            }
            Imgproc.rectangle(workingMatrix, new Rect(colStart, rowStart, boxWidth, boxHeight), new Scalar(0, 255, 0), 2);
        }

        int count = 0, bestPosition = -1, secondBestPosition = -1;
        double bestPositionValue = 75, secondBestPositionValue = 75;
        double boxValue;

        // Loop through every box where the robot looks for rings and find the places where there are most likely rings
        // The value we're looking at is lowest in the presence of rings
        // Old way: boxValue = Core.sumElems(box).val[2]/boxArea; private double isRingThreshhold = 105;
        for (Mat box : this.boxes) {
            boxValue = Core.sumElems(box).val[1]/boxArea; // should look for redishness
            threshholds[count] = boxValue;
            if (boxValue > isRingThreshhold){
                if (boxValue > bestPositionValue){
                    // If the best position has been surpassed, what used to be best is now second best
                    secondBestPositionValue = bestPositionValue;
                    secondBestPosition = bestPosition;
                    // save new best position
                    bestPositionValue = boxValue;
                    bestPosition = count;

                } else if (boxValue > secondBestPositionValue){
                    // if the probability isn't a new best, but is better than the current second best, save it as the second best.
                    secondBestPositionValue = boxValue;
                    secondBestPosition = count;
                }
                // because a ring can fill two boxes, don't allow two adjacent boxes to be selected.
                //if (Math.abs(secondBestPosition-bestPosition) <= 1) {
                //    secondBestPosition = -1;
                //    secondBestPositionValue = 125;
                //}
            }
            count++;
        }

        // save the positions for the robot to go get rings
        positions[0] = bestPosition;
        positions[1] = secondBestPosition;

        return workingMatrix;
    }

    // given a position, a robot, and a distance in inches to travel, it returns the angle, x, and y of where the robot needs to go to get a ring.
    public double[] getRingPath(int position, FreightFrenzyRobot robot, double distance) {
        // quick estimates as to where the phone is actually looking from, assumes robot faces 0 degrees
        return getRingPath(position, robot, distance, 7, 8.5);
    }
    public double[] getRingPath(int position, FreightFrenzyRobot robot, double distance, double phoneXOffset, double phoneYOffset){
        double phoneX, phoneY, finalX, finalY, positionAngle;
        // commented out because this robot has no odometer
//        ArrayList<Double> currentCoordinates = robot.odometer.getCurrentCoordinates();
//
//        phoneX = robot.odometer.x + phoneXOffset;
//        phoneY = robot.odometer.y + phoneYOffset;
//
//        positionAngle = robot.odometer.angle - (position * this.cameraDegreeRange/numBoxes) + this.cameraDegreeRange/2 - this.cameraDegreeOffset;
//
//        finalX = phoneX + Math.cos(Math.toRadians(90+positionAngle))*distance;
//        finalY = phoneY + Math.sin(Math.toRadians(90+positionAngle))*distance;

        double[] output = {};// = {positionAngle, finalX, finalY};
        return output;
    }
}