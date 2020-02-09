package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.VisionSubSystem;

import org.opencv.bioinspired.Bioinspired;
import org.opencv.bioinspired.Retina;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.ximgproc.Ximgproc;

import java.util.ArrayList;
import java.util.List;

public class SconeFinder extends VisionSubSystem {

    //A detail factor, higher factor = lower detail, lower factor = higher detail
    private static final double APPROXIMATION_FACTOR = 0.001;
    private static final double CUBE_APPROX_FACTOR = 0.03;
    //Minimum skeleton length.
    private static final double SPINAL_CORD_MIN_LENGTH = 300;
    //Percentage within which the second skystone black values must be of the first skystone.
    private static final double SECOND_SKYSTONE_DETECTION_PERCENTAGE = 0.5;
    //Lab colorspace ranges for yellow.
    private static final Scalar LOWER_Lab_RANGE = new Scalar(0,105,170), UPPER_Lab_RANGE = new Scalar(255,170,255);
    //Constant to shift the line up or down by. - is down + is up.
    private static final int LINE_SHIFT = -23;

    private static final double BADULARITY_THRESH = 0.5;

    private static double lowerSearchBound, upperSearchBound;

    //All the Mats used for processing in the skystone detection algorithm.
    private Mat cpy = new Mat(),
            blackMap = new Mat(),
            lab = new Mat(),
            thresh = new Mat(),
            spookyScarySkeleton = new Mat(),
            theHorizon = new Mat(),
            mask = new Mat(),
            gapThresh = new Mat(),
            fullLineThresh = new Mat(),
            blackMasked = new Mat(),
            edges = new Mat(),
            tempMask = new Mat();
    //A list of the locations of all detected skystones.
    private static List<Point> skystones;
    //A retina model used for lighting correction.
    private Retina retina;

    /**
     * The constructor for ScoreFinder.
     *
     * @param robot The robot the subsystem is using.
     */
    public SconeFinder(Robot robot) {
        super(robot);
        skystones = new ArrayList<>();
    }

    @Override
    public Mat onCameraFrame(Mat input) {

        //Local version of skystone list.
        List<Point> skystones = new ArrayList<>();

        //Converts input mat from rgba to rgb to make it easier to work with
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGBA2RGB);

        //Apply retina-based lighting correction.
        cpy = input.clone();
        retina.applyFastToneMapping(cpy,input);
        cpy.release();

        //Starts the conversion of the image into the black colorspace map for skystones behind the scenes.
        BlackColorspace blackColorspaceCvt = new BlackColorspace(input, blackMap);
        blackColorspaceCvt.run();

        //Converts input image from the RGB colorspace to the Lab colorspace for color thresholding
        Imgproc.cvtColor(input,lab,Imgproc.COLOR_RGB2Lab);

        //Creates a binary mask based on Lab values. All values within the given colorspace lower and upper bounds will be marked as inliers by the mask.
        Core.inRange(lab, LOWER_Lab_RANGE, UPPER_Lab_RANGE, thresh);
        lab.release();

        //Skeletonizes the binary mask to produce prominent lines
        Ximgproc.thinning(thresh,spookyScarySkeleton);

        //Finds the MatOfPoint objects for all the lines in the skeletonized image. Uses chain_approx_none for more data.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(spookyScarySkeleton,contours,new Mat(),Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_NONE);
        spookyScarySkeleton.release();

        //The Spookiest Skeleton is the list of all good points to be considered by the fitLine algorithm.
        MatOfPoint theSpookiestSkeleton = new MatOfPoint();

        //Loops through all detected skeletons.
        for(MatOfPoint skeleton : contours) {
            MatOfPoint2f approx = new MatOfPoint2f();
            MatOfPoint2f skeleton2f = new MatOfPoint2f(skeleton.toArray());

            //Calculates the length of the skeleton
            double peri = Imgproc.arcLength(skeleton2f, false);

            if(peri > SPINAL_CORD_MIN_LENGTH) {
                //Approximates the detected skeleton.
                Imgproc.approxPolyDP(skeleton2f, approx, APPROXIMATION_FACTOR * peri, false);

                MatOfPoint approxMop = new MatOfPoint(approx.toArray());

                List<MatOfPoint> approxList = new ArrayList<>();
                approxList.add(approxMop);
                Imgproc.drawContours(input,approxList,0,new Scalar(255,0,0),3);

                //Adds the approximated skeleton to the spooky skeleton data Mat.
                theSpookiestSkeleton.push_back(approxMop);
                approxMop.release();
            }

            approx.release();
            skeleton2f.release();
            skeleton.release();
        }

        //Wait until the black colorspace converter is not longer running
        waitWhile(blackColorspaceCvt::isAlive);

        if(!theSpookiestSkeleton.empty()) {

            //Fits the data with a line using linear least squares.
            Imgproc.fitLine(theSpookiestSkeleton, theHorizon, Imgproc.DIST_L2, 0, 0.01, 0.01);

            //Calculates very large start and end points so the line goes all the way across the screen.
            Point vector = new Point(theHorizon.get(0,0)[0],theHorizon.get(1,0)[0]);
            Point start = new Point(theHorizon.get(2,0)[0]-200*vector.x,theHorizon.get(3,0)[0]-200*vector.y-LINE_SHIFT);
            Point end = new Point(start.x + 400*vector.x, start.y + 400*vector.y);

            //Draws the line on the input image for debugging.
            Imgproc.line(input,start,end,new Scalar(0,0,255),3);

            //Creates an empty mask and draws a white line on it, creating a binary image.
            mask = Mat.zeros(blackMap.size(),CvType.CV_8U);
            Imgproc.line(mask,start,end,new Scalar(255),5);

            //makes sure the line is confined to the areas of detected skystones.
            Core.bitwise_and(mask,thresh,fullLineThresh);

            //Find all the gaps in the line.
            Core.bitwise_xor(fullLineThresh,mask,gapThresh,mask);
            fullLineThresh.release();
            mask.release();

            List<MatOfPoint> regions = new ArrayList<>();

            //Mask the black colorspace with the gaps in the line.
            Core.bitwise_and(blackMap,gapThresh,blackMasked);

            Imgproc.morphologyEx(blackMap,blackMap,Imgproc.MORPH_CLOSE,Imgproc.getStructuringElement(Imgproc.MORPH_CROSS,new Size(3,3)));
            Imgproc.Canny(blackMap,edges,200,255);
            Imgproc.dilate(edges,edges,Imgproc.getStructuringElement(Imgproc.MORPH_CROSS,new Size(3,3)));

            Imgproc.findContours(edges,regions,new Mat(),Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
            edges.release();

            tempMask = Mat.zeros(gapThresh.size(),gapThresh.type());

            //Finds the contour containing that skystone and blacks it out of the thresh.
            for(MatOfPoint reg : regions) {
                MatOfPoint2f approx = new MatOfPoint2f();
                MatOfPoint2f reg2f = new MatOfPoint2f(reg.toArray());
                double peri = Imgproc.arcLength(reg2f,true);
                Imgproc.approxPolyDP(reg2f, approx, CUBE_APPROX_FACTOR * peri, true);
                reg2f.release();

                Rect bbox = Imgproc.boundingRect(approx);
                if(Imgproc.contourArea(approx)/(bbox.width*bbox.height) > BADULARITY_THRESH) {
                    Imgproc.drawContours(tempMask,regions,regions.indexOf(reg),new Scalar(255),-1);
                }
            }

            Core.bitwise_and(gapThresh,tempMask,gapThresh);
            tempMask.release();

            //Find the maximum location of the gaps using the black colorspace as a metric.
            Core.MinMaxLocResult minMax = Core.minMaxLoc(blackMap,gapThresh);

            skystones.add(minMax.maxLoc);

            //Finds the next maximum value point.
            gapThresh.release();
/*
            //If the region is within a certain range of the first maximum value, it is a skystone, and is added to the list.
            if(minMax2.maxVal >= SECOND_SKYSTONE_DETECTION_PERCENTAGE*minMax.maxVal) {
                for(MatOfPoint reg : regions) {
                    MatOfPoint2f reg2f = new MatOfPoint2f(reg.toArray());
                    int containedInContour = (int) Imgproc.pointPolygonTest(reg2f,minMax2.maxLoc,false);
                    reg2f.release();

                    Rect bbox = Imgproc.boundingRect(reg);
                    if((containedInContour == 0 || containedInContour == 1) && (Imgproc.contourArea(reg)/(bbox.width*bbox.height)) >= BADULARITY_THRESH) {
                        skystones.add(minMax2.maxLoc);
                        Imgproc.drawContours(input,regions,regions.indexOf(reg),new Scalar(255,255,0),3);
                        break;
                    }
                }
            }
*/
            //Draws all the skystones.
            for(Point p : skystones) {
                Imgproc.circle(input, p, 5, new Scalar(0, 255, 0), -1);
            }

            SconeFinder.skystones = skystones;
        }
        thresh.release();
        theSpookiestSkeleton.release();
        theHorizon.release();
        blackMap.release();

        double redRight = 92, redLeft = 28;
        Imgproc.line(input,new Point(redLeft,0), new Point(redLeft,500),new Scalar(120,0,0),3);
        Imgproc.line(input,new Point(redRight,0), new Point(redRight,500),new Scalar(255,0,0),3);
        double blueRight = 182, blueLeft = 118;
        Imgproc.line(input,new Point(blueLeft,0), new Point(blueLeft,500),new Scalar(0,0,120),3);
        Imgproc.line(input,new Point(blueRight,0), new Point(blueRight,500),new Scalar(0,0,255),3);

        return input;
    }

    @Override
    public void init() {
        retina = Retina.create(new Size(240, 320),true,Bioinspired.RETINA_COLOR_BAYER);
        retina.activateMovingContoursProcessing(false);
        retina.setup();
        retina.clearBuffers();

        lowerSearchBound = 0;
        upperSearchBound = 240;
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
        stopVision();
    }

    /**
     * Gets a list of all detected skystones. This list is cached.
     *
     * @return A list of all detected skystones.
     */
    public List<Point> getSkystones() {
        return skystones;
    }

    public void setUpperSearchBound(double upperSearchBound) {
        SconeFinder.upperSearchBound = upperSearchBound;
    }

    public void setLowerSearchBound(double lowerSearchBound) {
        SconeFinder.lowerSearchBound = lowerSearchBound;
    }
}
