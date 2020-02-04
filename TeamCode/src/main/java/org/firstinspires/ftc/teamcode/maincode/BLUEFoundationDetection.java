package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.VisionSubSystem;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class BLUEFoundationDetection extends VisionSubSystem {

        public BLUEFoundationDetection(Robot robot) {
            super(robot);
        }

        @Override
        public Mat onCameraFrame (Mat input){
            Mat hsv = new Mat();
            Imgproc.cvtColor(input,input,Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Mat binary = new Mat();

            Core.inRange(hsv,new Scalar(30,0,50),new Scalar(200,255,255),binary);

            Mat sChannel = new Mat();
            Core.extractChannel(hsv, sChannel, 1);

            Imgproc.cvtColor(binary, binary, Imgproc.COLOR_GRAY2RGB);

            Core.bitwise_and(input, binary, input);
            return input;
        }
    @Override
        public void init () {
            startVision();
        }

        @Override
        public void init_loop () {

        }

        @Override
        public void start () {

        }

        @Override
        public void handle () {

        }

        @Override
        public void stop () {

        }

}

