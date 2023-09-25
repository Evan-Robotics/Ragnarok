/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(name="--WIP LEFT-- Drop and Park")
public class LeftDrop extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private Encoder straightEncoder;
    private Encoder strafeEncoder;

    private Servo claw;

    double STRAIGHT_INCH_TO_TICKS = 1876.7176781002;
    double STRAFE_INCH_TO_TICKS = 175735. / 94.75;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        front_left = hardwareMap.get(DcMotor.class, "FRONT LEFT");
        front_right = hardwareMap.get(DcMotor.class, "FRONT RIGHT");
        back_left = hardwareMap.get(DcMotor.class, "BACK LEFT");
        back_right = hardwareMap.get(DcMotor.class, "BACK RIGHT");

        straightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FRONT RIGHT"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BACK LEFT"));

        claw = hardwareMap.get(Servo.class, "CLAW");

        claw.setPosition(0.05);

        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        straightEncoder.setDirection(Encoder.Direction.FORWARD);
        strafeEncoder.setDirection(Encoder.Direction.FORWARD);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw.setPosition(0.5);
        sleep(500);

        switch (tagOfInterest.id) {
            case 1:
                telemetry.addLine("Running park1");
                telemetry.update();
                park1();
                break;
            case 2:
                telemetry.addLine("Running park2");
                telemetry.update();
                park2();
                break;
            case 3:
                telemetry.addLine("Running park2");
                telemetry.update();
                park3();
                break;
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }

    private void forward(double in, double power) {
        int startingPos = straightEncoder.getCurrentPosition();
        int targetPos = (int) (in * STRAIGHT_INCH_TO_TICKS) + startingPos;
        while (straightEncoder.getCurrentPosition() < targetPos && !isStopRequested()) {
            front_left.setPower(power);
            front_right.setPower(power);
            back_left.setPower(power);
            back_right.setPower(power);
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos", straightEncoder.getCurrentPosition());
            telemetry.update();
            sleep(1);
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    private void back(double in, double power) {
        int startingPos = straightEncoder.getCurrentPosition();
        int targetPos = (int) (in * STRAIGHT_INCH_TO_TICKS) + startingPos;
        while (straightEncoder.getCurrentPosition() > targetPos && !isStopRequested()) {
            front_left.setPower(-power);
            front_right.setPower(-power);
            back_left.setPower(-power);
            back_right.setPower(-power);
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos", straightEncoder.getCurrentPosition());
            telemetry.update();
            sleep(1);
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    private void right(double in, double power) {
        int startingPos = strafeEncoder.getCurrentPosition();
        int targetPos = (int) (in * STRAFE_INCH_TO_TICKS) + startingPos;
        while (strafeEncoder.getCurrentPosition() < targetPos && !isStopRequested()) {
            front_left.setPower(power);
            front_right.setPower(-power);
            back_left.setPower(-power);
            back_right.setPower(power);
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos", strafeEncoder.getCurrentPosition());
            telemetry.update();
            sleep(1);
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    private void left(double in, double power) {
        int startingPos = strafeEncoder.getCurrentPosition();
        int targetPos = (int) (in * STRAFE_INCH_TO_TICKS) + startingPos;
        while (strafeEncoder.getCurrentPosition() > targetPos && !isStopRequested()) {
            front_left.setPower(-power);
            front_right.setPower(power);
            back_left.setPower(power);
            back_right.setPower(-power);
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos", strafeEncoder.getCurrentPosition());
            telemetry.update();
            sleep(1);
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    private void park1() {
        back(2, 0.2);
        sleep(1000);

        right(24, 0.4);
        sleep(100);
        claw.setPosition(0);
        sleep(1000);

        back(24. * 1.5, 0.5);
    }
    private void park2() {
        back(2, 0.2);
        sleep(1000);

        right(24, 0.4);
        sleep(100);
        claw.setPosition(0);
        sleep(1000);

        back(24, 0.3);
        sleep(1000);

        left(24, 0.4);
    }
    private void park3() {
        back(2, 0.2);
        sleep(1000);

        right(24, 0.4);
        sleep(100);
        claw.setPosition(0);
        sleep(1000);

        back(24, 0.3);
        sleep(1000);

        left(24 * 2, 0.4);
    }
}