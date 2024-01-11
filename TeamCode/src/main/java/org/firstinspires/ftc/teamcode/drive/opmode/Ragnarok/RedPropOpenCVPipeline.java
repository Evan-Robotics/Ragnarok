package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok;

import static org.opencv.core.Core.inRange;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RedPropOpenCVPipeline extends OpenCvPipeline {
    //backlog of frames to average out to reduce noise

    public String whichSide;

    public boolean hasProcessedFrame = false;
    public int max;

    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;


    public RedPropOpenCVPipeline() {
        frameList = new ArrayList<>();
    }

    Mat YCbCr = new Mat();
    Mat region1_Cb, region2_Cb, region3_Cb;
    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
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

        // Convert the input frame to HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // Define HSV range for red color
        Scalar lowHSV = new Scalar(0, 100, 100);
        Scalar highHSV = new Scalar(10, 255, 255);

        // Create a mask for the red color
        Mat mask = new Mat();
        inRange(mat, lowHSV, highHSV, mask);

        // List of frames to reduce inconsistency
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        // Release unnecessary Mats
        mat.release();

        // Define rectangles for each region
        Rect leftRect = new Rect(1, 1, 213, 359);
        Rect centerRect = new Rect(214, 1, 213, 359);
        Rect rightRect = new Rect(427, 1, 213, 359);

        // Draw rectangles on the input frame
        Imgproc.rectangle(input, leftRect, rectColor, 2);
        Imgproc.rectangle(input, centerRect, rectColor, 2);
        Imgproc.rectangle(input, rightRect, rectColor, 2);

        // Extract submats for each region
        Mat region1_bw = new Mat(mask, leftRect);
        Mat region2_bw = new Mat(mask, centerRect);
        Mat region3_bw = new Mat(mask, rightRect);

        // Count non-zero pixels in each region
        avg1 = (int) Core.countNonZero(region1_bw);
        avg2 = (int) Core.countNonZero(region2_bw);
        avg3 = (int) Core.countNonZero(region3_bw);

        // Release unnecessary Mats
        region1_bw.release();
        region2_bw.release();
        region3_bw.release();

        // Find the region with the maximum red pixels
        int maxOneTwo = Math.max(avg1, avg2);
        max = Math.max(maxOneTwo, avg3);

        if (max == avg1) {
            whichSide = "left";
        } else if (max == avg2) {
            whichSide = "center";
        } else if (max == avg3) {
            whichSide = "right";
        }

        hasProcessedFrame = true;

        return input;
    }
}
