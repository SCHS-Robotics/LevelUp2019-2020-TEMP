package org.firstinspires.ftc.teamcode.maincode;

import android.util.Log;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.VisionSubSystem;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.ximgproc.Ximgproc;

import java.util.ArrayList;
import java.util.List;

public class SconeFinder extends VisionSubSystem {
    public SconeFinder(Robot robot) {
        super(robot);
    }

    @Override
    public Mat onCameraFrame(Mat input) {

        Imgproc.medianBlur(input,input,9);

        Mat hls = new Mat(), lab = new Mat(), hsv = new Mat(), ycrcb = new Mat();

        Imgproc.cvtColor(input,lab,Imgproc.COLOR_RGB2Lab);
        Imgproc.cvtColor(input,hls,Imgproc.COLOR_RGB2HLS_FULL);
        Imgproc.cvtColor(input,hsv,Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(input,ycrcb,Imgproc.COLOR_RGB2YCrCb);

        Mat sChan = new Mat(), sChan2 = new Mat(), aChan = new Mat(), cbChan = new Mat(), bChan = new Mat();
        Core.extractChannel(hls,sChan,2);
        Core.extractChannel(hsv,sChan2,1);
        Core.extractChannel(lab,aChan,1);
        Core.extractChannel(lab,bChan,2);
        Core.extractChannel(ycrcb,cbChan,2);
        Core.bitwise_not(cbChan,cbChan);

        hls.release();
        hsv.release();
        lab.release();
        ycrcb.release();

        Mat yellowMask = new Mat(), redMask = new Mat();

        Imgproc.threshold(bChan,yellowMask,150,255,Imgproc.THRESH_BINARY);
        Imgproc.threshold(aChan,redMask,160,255,Imgproc.THRESH_BINARY_INV);
        aChan.release();

        sChan.convertTo(sChan,CvType.CV_32F);
        sChan2.convertTo(sChan2, CvType.CV_32F);
        bChan.convertTo(bChan,CvType.CV_32F);
        cbChan.convertTo(cbChan,CvType.CV_32F);

        Core.multiply(sChan,new Scalar(0.25/255.0),sChan);
        Core.multiply(sChan2,new Scalar(0.25/255.0),sChan2);
        Core.multiply(bChan,new Scalar(1.0/255.0),bChan);
        Core.multiply(cbChan,new Scalar(1.0/255.0),cbChan);

        Mat sbChan = new Mat(), ssbChan = new Mat(), ssbcbChan = new Mat();

        Core.add(sChan,bChan,sbChan);
        sChan.release();
        bChan.release();

        Core.add(sbChan,sChan2,ssbChan);
        sbChan.release();
        sChan2.release();

        Core.add(ssbChan,cbChan,ssbcbChan);
        ssbChan.release();
        cbChan.release();

        Core.multiply(ssbcbChan,new Scalar(1.0/1.5),ssbcbChan);

        Core.MinMaxLocResult theMax = Core.minMaxLoc(ssbcbChan);
        Core.multiply(ssbcbChan,new Scalar(1.0/theMax.maxVal),ssbcbChan);
        Core.pow(ssbcbChan,3,ssbcbChan);
        Core.multiply(ssbcbChan,new Scalar(theMax.maxVal),ssbcbChan);
        Core.multiply(ssbcbChan,new Scalar(255.0),ssbcbChan);

        ssbcbChan.convertTo(ssbcbChan,CvType.CV_8U);

        Core.bitwise_and(ssbcbChan,yellowMask,ssbcbChan);
        Core.bitwise_and(ssbcbChan,redMask,ssbcbChan);

        yellowMask.release();
        redMask.release();

        Imgproc.medianBlur(ssbcbChan,ssbcbChan,9);

        /*
        Mat colorMap = new Mat();
        Imgproc.applyColorMap(ssbcbChan,colorMap,Imgproc.COLORMAP_JET);
        ssbcbChan.release();

        Imgproc.cvtColor(colorMap,colorMap,Imgproc.COLOR_BGR2RGB);
*/

        Mat dist = new Mat(), secondD = new Mat(), thresh = new Mat();

        Imgproc.threshold(ssbcbChan,thresh,140,255,Imgproc.THRESH_BINARY);
        Imgproc.distanceTransform(thresh,dist,Imgproc.DIST_L1,3);

        Core.normalize(dist,dist,0,255,Core.NORM_MINMAX);

        Imgproc.boxFilter(dist,dist,CvType.CV_8U,new Size(5,5));

        Imgproc.Laplacian(dist,secondD,CvType.CV_16S);

        dist.release();

        Imgproc.GaussianBlur(secondD,secondD,new Size(3,3),5,5);

        Core.multiply(secondD,new Scalar(-1),secondD);

        //Core.normalize(secondD,secondD,0,1,Core.NORM_MINMAX);
        //Core.pow(secondD,3,secondD);
        Core.normalize(secondD,secondD,0,255,Core.NORM_MINMAX);

        secondD.convertTo(secondD,CvType.CV_8U);

        Mat closed = new Mat();
        Imgproc.morphologyEx(secondD,closed,Imgproc.MORPH_CLOSE,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(3,3)),new Point(-1,-1),3);

        secondD.release();

        Mat spookyScarySkeleton = new Mat();
        Ximgproc.thinning(closed,spookyScarySkeleton);

        closed.release();

        spookyScarySkeleton.convertTo(spookyScarySkeleton,CvType.CV_8U);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(spookyScarySkeleton,contours,new Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.cvtColor(spookyScarySkeleton,spookyScarySkeleton,Imgproc.COLOR_GRAY2BGR);

        MatOfPoint theSpookiestSkeleton = new MatOfPoint();

        for(int i = 0; i < contours.size(); i++) {
            MatOfPoint2f approx = new MatOfPoint2f();
            double peri = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), false);
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), approx, 0.01 * peri, false); //0.1 is a detail factor, higher factor = lower detail, lower factor = higher detail
            MatOfPoint approxMop = new MatOfPoint(approx.toArray());

            //List<MatOfPoint> approxList = new ArrayList<>();
            //approxList.add(approxMop);

            if(approx.toList().size() > 15) {
                //Imgproc.drawContours(spookyScarySkeleton,approxList,0,new Scalar(0,255,0),3);
                theSpookiestSkeleton.push_back(approxMop);
            }
            approxMop.release();
            approx.release();
            contours.get(i).release();
        }

        Mat theHorizon = new Mat();
        Point start = new Point(0,0), end = new Point(0,0);
        if(!theSpookiestSkeleton.empty()) {
            Imgproc.fitLine(theSpookiestSkeleton, theHorizon, Imgproc.DIST_L1, 0, 0.01, 0.01);

            Point vector = new Point(theHorizon.get(0,0)[0],theHorizon.get(1,0)[0]);
            Point realStart = new Point(theHorizon.get(2,0)[0],theHorizon.get(3,0)[0]);
            start = new Point(theHorizon.get(2,0)[0]-200*vector.x,theHorizon.get(3,0)[0]-200*vector.y);
            end = new Point(start.x + 400*vector.x, start.y + 400*vector.y);

            Imgproc.line(input,start,end,new Scalar(0,0,255),3);
        }
        theSpookiestSkeleton.release();
//------------------------------------------------------------
        List<Mat> rgb = new ArrayList<>();
        Core.split(input,rgb);

        rgb.get(0).convertTo(rgb.get(0),CvType.CV_32F);
        rgb.get(1).convertTo(rgb.get(1),CvType.CV_32F);
        rgb.get(2).convertTo(rgb.get(2),CvType.CV_32F);

        Mat gray = new Mat();
        Imgproc.cvtColor(input,gray,Imgproc.COLOR_BGR2GRAY);
        gray.convertTo(gray,CvType.CV_32F);

        //Core.multiply(bgr.get(0),new Scalar(1.0/255.0),bgr.get(0));
        //Core.multiply(bgr.get(1),new Scalar(1.0/255.0),bgr.get(1));
        //Core.multiply(bgr.get(2),new Scalar(1.0/255.0),bgr.get(2));

        Mat RG = new Mat(), RB = new Mat(), GB = new Mat();
        Core.subtract(rgb.get(0),rgb.get(1),RG);
        Core.subtract(rgb.get(0),rgb.get(2),RB);
        Core.subtract(rgb.get(1),rgb.get(2),GB);

        rgb.get(0).release();
        rgb.get(1).release();
        rgb.get(2).release();

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

        Core.multiply(accum,new Scalar(1.0/255.0),accum);
        Core.multiply(gray,new Scalar(1.0/255.0),gray);

        Core.add(accum,gray,accum);
        gray.release();
        Core.multiply(accum,new Scalar(255.0/2),accum);

        accum.convertTo(accum,CvType.CV_8U);
        Core.bitwise_not(accum,accum);
        accum.convertTo(accum,CvType.CV_32F);
        Core.multiply(accum,new Scalar(1.0/255),accum);

        Core.pow(accum,3,accum);

        Core.multiply(accum,new Scalar(255),accum);

        accum.convertTo(accum,CvType.CV_8U);

        Mat skystoneLines = new Mat();
        Imgproc.Canny(accum,skystoneLines,0,255);
        Mat skyStoneLinesClosed = new Mat();
        Imgproc.morphologyEx(skystoneLines,skyStoneLinesClosed,Imgproc.MORPH_CLOSE,Imgproc.getStructuringElement(Imgproc.MORPH_CROSS, new Size(3,3)),new Point(-1,-1),1);
        Ximgproc.thinning(skyStoneLinesClosed,skystoneLines);

        skyStoneLinesClosed.release();

        Mat mask = Mat.zeros(accum.size(),CvType.CV_8U);

        Imgproc.line(mask,start,end,new Scalar(255),5);
        Mat thresh2 = new Mat();
        Core.bitwise_and(mask,thresh,thresh2);
        thresh.release();
        Core.bitwise_xor(thresh2,mask,thresh2,mask);

        mask.release();

        MatOfInt centers = new MatOfInt();
        MatOfInt vals = new MatOfInt();
        List<MatOfPoint> regions = new ArrayList<>();

        Mat accumMasked = new Mat();
        Core.bitwise_and(accum,thresh2,accumMasked);

        Imgproc.findContours(thresh2,regions,new Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);

        thresh2.release();

        for(MatOfPoint reg : regions) {
            MatOfPoint2f approx = new MatOfPoint2f();
            double peri = Imgproc.arcLength(new MatOfPoint2f(reg.toArray()), true);
            Imgproc.approxPolyDP(new MatOfPoint2f(reg.toArray()), approx, 0.03 * peri, true);
            if(approx.toArray().length == 4) {
                Moments moments = Imgproc.moments(approx);
                centers.push_back(new MatOfInt((int) Math.round(moments.m10/moments.m00), (int) Math.round(moments.m01/moments.m00)));
                Rect bbox = Imgproc.boundingRect(approx);

                RotatedRect rotatedRect = Imgproc.minAreaRect(approx);

                Mat d = new Mat(1,1,CvType.CV_8U,new Scalar((int) Math.round(Core.sumElems(accumMasked.submat(bbox)).val[0]/(rotatedRect.size.area()))));
                vals.push_back(d);
            }
        }

        Mat valMask = new Mat();
        //System.out.println(centers.size());

        MatOfDouble mean = new MatOfDouble();
        MatOfDouble std = new MatOfDouble();

        Core.meanStdDev(accum,mean,std,mask);

        accum.release();

        vals.convertTo(vals,CvType.CV_8U);

        Imgproc.threshold(vals,valMask,mean.get(0,0)[0] + std.get(0,0)[0],255,Imgproc.THRESH_BINARY);
        Mat valMask2 = new Mat();
        valMask2.push_back(valMask.t());
        valMask2.push_back(valMask.t());

        Mat valMask3 = valMask2.t();

        Mat centers2 = centers.reshape(1,valMask.rows());
        centers2.convertTo(centers2,CvType.CV_8U);

        Mat out = new Mat();

        //Core.multiply(valMask,centers,centers);
        Core.bitwise_and(valMask3,centers2,out);
        for(int i = 0; i < centers2.rows(); i++) {
            Point p = new Point(out.get(i,0)[0],out.get(i,1)[0]);
            if(p.x != 0 && p.y != 0) {
                Log.wtf("WAHAM","SHABAM");
                Imgproc.circle(input, p, 3, new Scalar(0, 255, 0), -1);
            }
        }

        return input;
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

    }
}
