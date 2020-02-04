package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.VisionSubSystem;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class REDFoundationDetection extends VisionSubSystem {

    public REDFoundationDetection(Robot robot) {
        super(robot);
    }

    @Override
    public Mat onCameraFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat binary1 = new Mat(), binary2 = new Mat(), binary = new Mat();

        Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), binary1);
        Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(179, 255, 255), binary2);

        Core.bitwise_or(binary1,binary2,binary);

        Mat sChannel = new Mat();
        Core.extractChannel(hsv, sChannel, 1);

        Imgproc.cvtColor(binary, binary, Imgproc.COLOR_GRAY2RGB);

        Core.bitwise_and(input, binary, input);

        Mat shapes = input;

        Mat edges = new Mat();
        Imgproc.Canny(shapes,edges,10,255);
        Mat kernal = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_CROSS, new Size(3, 3));
        Mat edges2 = new Mat();
        Imgproc.dilate(edges,edges2,kernal,new
                Point(),1);

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(edges2,contours,new

                Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);
        for(
                MatOfPoint c :contours)

        {
            double peri = Imgproc.arcLength(new MatOfPoint2f((c.toArray())), true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(c.toArray()), approx, 0.08 * peri, true);
            if (approx.toList().size() > 5) {
                Imgproc.drawContours(shapes, contours, contours.indexOf(c), new Scalar(0, 255, 0), 10);
            }
        }

        return input;
    }

    public void init() {
        startVision();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }
}
