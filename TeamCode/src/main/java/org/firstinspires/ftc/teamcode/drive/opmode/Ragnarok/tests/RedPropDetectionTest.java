/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.RedOpenCVMaster;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Red Detection Test")
public class RedPropDetectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        RedOpenCVMaster cv = new RedOpenCVMaster(this);
//      call the function to startStreaming
        cv.observeStick();
        int left;
        int right;
        int center;
        int item;
//        telemetry.addLine(cv.getItemStatus());
//        telemetry.addLine("lol");
        waitForStart();
        while (opModeIsActive()) {
            left = cv.opencv.avg1;
            center = cv.opencv.avg2;
            right = cv.opencv.avg3;
            if (left > center && left >= right) {
                item = 1;
            } else if (center >= left && center >= right) {
                item = 2;
            } else {
                item = 3;
            }

            telemetry.addData("Item: ", item);
            telemetry.addData("Debug", cv.opencv.max);
            telemetry.addData("Avg1", cv.opencv.avg1);
            telemetry.addData("Avg2", cv.opencv.avg2);
            telemetry.addData("Avg3", cv.opencv.avg3);
            telemetry.update();
            sleep(100);
        }
//        stopStreaming
        cv.stopCamera();
    }
}   // end class
