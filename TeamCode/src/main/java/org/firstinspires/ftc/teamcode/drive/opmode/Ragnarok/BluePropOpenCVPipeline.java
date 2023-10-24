package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BluePropOpenCVPipeline extends OpenCvPipeline {
    //backlog of frames to average out to reduce noise

    public String whichSide;

    public boolean hasProcessedFrame = false;
    public int max;

    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;

    public BluePropOpenCVPipeline() {
        frameList = new ArrayList<>();
    }

    Mat YCbCr = new Mat();
    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    Scalar rectColorFound = new Scalar(255.0, 100.0, 100.0);
    Mat Cb = new Mat();
    public int avg1, avg2, avg3;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCbCr, Cb, 2);
    }
    public String getWhichSide() {
        return whichSide;
    }
    @Override
    public void init(Mat firstFrame) {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        Rect leftRect = new Rect(1, 1, 213, 359);
        Rect centerRect = new Rect(214, 1, 213, 359);
        Rect rightRect = new Rect(427, 1, 213, 359);

        region1_Cb = Cb.submat(leftRect);
        region2_Cb = Cb.submat(centerRect);
        region3_Cb = Cb.submat(rightRect);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
//        Scalar lowHSV = new Scalar(0, 100, 100);
//        Scalar highHSV = new Scalar(0, 255, 255);
        Scalar lowHSV = new Scalar(80, 50, 50);
        Scalar highHSV = new Scalar(260, 240, 240);

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();

//        Scalar strictLowHSV = new Scalar(0, 100, 100);
//        Scalar strictHighHSV = new Scalar(0, 255, 255);
        Scalar strictLowHSV = new Scalar(80, 50, 50);
        Scalar strictHighHSV = new Scalar(240, 240, 240);


        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }


        //release all the data
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        thresh.release();
        finalMask.release();

        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)

        Rect leftRect = new Rect(1, 1, 213, 359);
        Rect centerRect = new Rect(214, 1, 213, 359);
        Rect rightRect = new Rect(427, 1, 213, 359);

        Imgproc.rectangle(input, leftRect, rectColor, 2);
        Imgproc.rectangle(input, centerRect, rectColor, 2);
        Imgproc.rectangle(input, rightRect, rectColor, 2);

        Mat region1_bw = input.submat(leftRect);
        Mat region2_bw = input.submat(centerRect);
        Mat region3_bw = input.submat(rightRect);

        avg1 = (int) Core.countNonZero(region1_bw);
        avg2 = (int) Core.countNonZero(region2_bw);
        avg3 = (int) Core.countNonZero(region3_bw);

        region1_bw.release();
        region2_bw.release();
        region3_bw.release();

        int maxOneTwo = Math.max(avg1, avg2);
        max = Math.max(maxOneTwo, avg3);

        if(max == avg1) {
            whichSide = "left";
        }
        else if(max == avg2) {
            whichSide = "center";
        }
        else if(max == avg3) {
            whichSide = "right";
        }


        hasProcessedFrame = true;

        return input;
    }
}
