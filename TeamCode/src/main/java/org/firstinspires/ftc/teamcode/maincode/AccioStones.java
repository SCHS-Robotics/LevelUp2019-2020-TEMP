package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.VisionSubSystem;

import org.opencv.bioinspired.Bioinspired;
import org.opencv.bioinspired.Retina;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.ximgproc.Ximgproc;

import java.util.ArrayList;
import java.util.List;

public class AccioStones extends VisionSubSystem {
    private Retina retina;
    private Mat rgb = new Mat(),
            lab = new Mat(),
            binary = new Mat(),
            linez = new Mat(),
            skeleton = new Mat();

    public AccioStones(Robot robot) {
        super(robot);
        retina = Retina.create(new Size(240, 320),true, Bioinspired.RETINA_COLOR_BAYER);
        retina.activateMovingContoursProcessing(false);
        retina.setup();
        retina.clearBuffers();
    }

    @Override
    public Mat onCameraFrame(Mat input) {

        Imgproc.cvtColor(input,rgb,Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(rgb,lab,Imgproc.COLOR_RGB2Lab);
        rgb.release();

        Core.inRange(lab, new Scalar(0,105,170), new Scalar(255,170,255), binary);
        lab.release();

        Imgproc.morphologyEx(binary,linez, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(7,1)),new Point(),5);
        binary.release();

        Ximgproc.thinning(linez,skeleton,Ximgproc.THINNING_GUOHALL);
        linez.release();

        Mat linesP = new Mat(); // will hold the results of the detection
        Imgproc.HoughLinesP(skeleton, linesP, 1, Math.PI/180, 50, 0, 100); // runs the actual detection

        for (int x = 0; x < linesP.rows(); x++) {
            double[] l = linesP.get(x, 0);
            Line line = new Line(new Point(l[0], l[1]), new Point(l[2], l[3]));

            if(line.getAngle() < 10 || line.getAngle() > 170) {
                line.draw(input);
                break;
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

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }
}
