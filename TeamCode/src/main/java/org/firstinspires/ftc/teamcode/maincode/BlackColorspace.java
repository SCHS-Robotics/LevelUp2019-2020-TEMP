package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class BlackColorspace implements Runnable {


    private Mat rgb, output;
    private boolean isAlive;
    public BlackColorspace(Mat rgb, Mat output) {
        this.rgb = rgb;
        this.output = output;
        isAlive = false;
    }

    @Override
    public void run() {
        isAlive = true;

        List<Mat> rgbLst = new ArrayList<>();
        Mat gray = new Mat();

        Thread rgbCvt = new Thread(() -> {
            Core.split(rgb,rgbLst);

            rgbLst.get(0).convertTo(rgbLst.get(0), CvType.CV_32F);
            rgbLst.get(1).convertTo(rgbLst.get(1),CvType.CV_32F);
            rgbLst.get(2).convertTo(rgbLst.get(2),CvType.CV_32F);
        });

        Thread grayCvt = new Thread(() -> {
            Imgproc.cvtColor(rgb,gray,Imgproc.COLOR_RGB2GRAY);
            gray.convertTo(gray,CvType.CV_32F);
        });

        rgbCvt.run();
        grayCvt.run();

        while(rgbCvt.isAlive() || grayCvt.isAlive());

        Mat RG = new Mat(), RB = new Mat(), GB = new Mat();
        Core.subtract(rgbLst.get(0),rgbLst.get(1),RG);
        Core.subtract(rgbLst.get(0),rgbLst.get(2),RB);
        Core.subtract(rgbLst.get(1),rgbLst.get(2),GB);

        rgbLst.get(0).release();
        rgbLst.get(1).release();
        rgbLst.get(2).release();

        Core.pow(RG,2,RG);
        Core.pow(RB,2,RB);
        Core.pow(GB,2,GB);

        Mat accum = new Mat();

        Core.add(RG,RB,accum);
        RG.release();
        RB.release();

        Core.add(accum,GB,accum);
        GB.release();

        Core.sqrt(accum,accum);

        Core.multiply(accum,new Scalar(127.5/255.0),accum);
        Core.multiply(gray,new Scalar(127.5/255.0),gray);

        Core.add(accum,gray,accum);
        gray.release();

        accum.convertTo(accum,CvType.CV_8U);
        Core.bitwise_not(accum,accum);
        accum.convertTo(accum,CvType.CV_32F);

        Core.multiply(accum,new Scalar(1.0/255),accum);
        Core.pow(accum,3,accum);
        Core.multiply(accum,new Scalar(255),accum);

        accum.convertTo(accum,CvType.CV_8U);

        accum.copyTo(output);

        isAlive = false;
    }

    public boolean isAlive() {
        return isAlive;
    }
}
