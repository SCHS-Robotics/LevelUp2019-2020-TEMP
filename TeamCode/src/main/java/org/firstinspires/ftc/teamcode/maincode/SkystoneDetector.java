package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.VisionSubSystem;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class SkystoneDetector extends VisionSubSystem {

    public SkystoneDetector(Robot robot) {
        super(robot);
    }

    @Override
    public Mat onCameraFrame(Mat input) {
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGBA2RGB);

        Imgproc.medianBlur(input,input,9);

        Mat hls = new Mat(), lab = new Mat(), hsv = new Mat(), ycrcb = new Mat();

        Imgproc.cvtColor(input,lab,Imgproc.COLOR_RGB2Lab);
        Imgproc.cvtColor(input,hls,Imgproc.COLOR_RGB2HLS_FULL);
        Imgproc.cvtColor(input,hsv,Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(input,ycrcb, Imgproc.COLOR_RGB2YCrCb);

        input.release();

        Mat sChan = new Mat(), sChan2 = new Mat(), bChan = new Mat(), aChan = new Mat(), cbChan = new Mat();

        Core.extractChannel(hls,sChan,2);
        Core.extractChannel(hsv,sChan2,1);
        Core.extractChannel(lab,bChan,2);
        Core.extractChannel(lab,aChan,1);
        Core.extractChannel(ycrcb,cbChan,2);

        hls.release();
        hsv.release();
        lab.release();

        Mat yellowMask = new Mat(), redMask = new Mat();
        Imgproc.threshold(bChan,yellowMask,150,255,Imgproc.THRESH_BINARY);
        Imgproc.threshold(aChan,redMask,160,255,Imgproc.THRESH_BINARY_INV);

        sChan.convertTo(sChan, CvType.CV_32F);
        sChan2.convertTo(sChan2, CvType.CV_32F);
        bChan.convertTo(bChan, CvType.CV_32F);
        cbChan.convertTo(cbChan, CvType.CV_32F);

        Core.multiply(sChan, new Scalar(1/255.0), sChan);
        Core.multiply(sChan2, new Scalar(1/255.0), sChan2);
        Core.multiply(bChan, new Scalar(1/255.0), bChan);
        Core.multiply(cbChan, new Scalar(1/255.0), cbChan);

        Mat sbChan = new Mat(), ssbChan = new Mat(), ssbcbChan = new Mat();
        Core.add(sChan,bChan,sbChan);
        sChan.release();
        bChan.release();
        Core.add(sbChan,sChan2,ssbChan);
        sbChan.release();
        sChan2.release();
        Core.add(ssbChan, cbChan, ssbcbChan);
        ssbChan.release();
        cbChan.release();

        Mat aboveHalfYellowMask = new Mat(), belowHalfRedMask = new Mat();

        Imgproc.threshold(bChan,aboveHalfYellowMask,255/2,255,Imgproc.THRESH_BINARY);
        Imgproc.threshold(aChan,belowHalfRedMask,160,255,Imgproc.THRESH_BINARY_INV);

        aChan.release();

        Mat sChanNorm = new Mat(), bChanNorm = new Mat();

        Core.MinMaxLocResult bMinMax = Core.minMaxLoc(bChan), sMinMax = Core.minMaxLoc(sChan);

        Core.normalize(bChan,bChanNorm,0,255/2.0,Core.NORM_MINMAX);
        Core.normalize(sChan,sChanNorm,0,255/2.0,Core.NORM_MINMAX);

        bChan.release();
        sChan.release();

        //t sbChan = new Mat();
        Core.addWeighted(bChanNorm,bMinMax.maxVal/255.0,sChanNorm,sMinMax.maxVal/255.0,0,sbChan);

        bChanNorm.release();
        sChanNorm.release();

        Mat sChan2Norm = new Mat(), sbChanNorm = new Mat();

        Core.MinMaxLocResult sbMinMax = Core.minMaxLoc(sbChan), s2MinMax = Core.minMaxLoc(sChan2);

        Core.normalize(sbChan,sbChanNorm,0,255/2.0,Core.NORM_MINMAX);
        Core.normalize(sChan2,sChan2Norm,0,255/2.0,Core.NORM_MINMAX);

        sbChan.release();
        sChan2.release();

        //t ssbChan = new Mat();
        Core.addWeighted(sbChanNorm,sbMinMax.maxVal/255.0,sChan2Norm,s2MinMax.maxVal/255.0,0,ssbChan);

        Imgproc.medianBlur(ssbChan,ssbChan,9);
        Imgproc.GaussianBlur(ssbChan,ssbChan,new Size(9,9),1,1);

        ssbChan.convertTo(ssbChan, CvType.CV_32F);

        Core.normalize(ssbChan,ssbChan,0,1,Core.NORM_MINMAX);
        Core.pow(ssbChan,3,ssbChan);
        Core.normalize(ssbChan,ssbChan,0,255,Core.NORM_MINMAX);

        ssbChan.convertTo(ssbChan,CvType.CV_8U);

        Mat colorMap = new Mat();
        Imgproc.applyColorMap(ssbChan,colorMap,Imgproc.COLORMAP_JET);
        ssbChan.release();

        Imgproc.cvtColor(colorMap,colorMap,Imgproc.COLOR_BGR2RGB);

        return colorMap;
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        startVision();
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        stopVision();
    }
}
