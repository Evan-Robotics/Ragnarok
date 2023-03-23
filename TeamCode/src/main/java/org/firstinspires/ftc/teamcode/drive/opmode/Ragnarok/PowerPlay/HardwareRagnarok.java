/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class HardwareRagnarok {
    /* Public OpMode members. */
    public DcMotorEx leftTower = null;
    public DcMotorEx rightTower = null;

    public Servo leftTwist = null;
    public Servo rightTwist= null;

    public static double TWIST_POS_1 = 0.87;
    public static double TWIST_POS_2 = 0.2;

    public Servo wrist = null;
    public static double WRIST_POS_1 = 0.722;
    public static double WRIST_POS_2 = 0.04;

    public Servo claw = null;
    public static double CLAW_POS_1 = 0.18;
    public static double CLAW_POS_2 = 0.3;

    public Servo guide = null;
    public static double GUIDE_POS_1 = 0.05;
    public static double GUIDE_POS_2 = 0.48;

    public Servo flag = null;
    public static double FLAG_POS_1 = 0;
    public static double FLAG_POS_2 = 0.35;

    public RevColorSensorV3 sensor = null;
    public static double CLOSE_DISTANCE_MM = 12;


    /* local OpMode members. */
    HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRagnarok(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        // Define and Initialize Motors
        leftTower = ahwMap.get(DcMotorEx.class, "LEFT TOWER");
        leftTower.setDirection(DcMotorEx.Direction.REVERSE);
        rightTower = ahwMap.get(DcMotorEx.class, "RIGHT TOWER");
        rightTower.setDirection(DcMotorEx.Direction.FORWARD);

        // Set all motors to zero power
        leftTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.


        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftTower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        leftTwist = ahwMap.get(Servo.class, "LEFT TWIST");
        leftTwist.setDirection(Servo.Direction.FORWARD);
        leftTwist.setPosition(TWIST_POS_1);

        rightTwist = ahwMap.get(Servo.class, "RIGHT TWIST");
        rightTwist.setDirection(Servo.Direction.REVERSE);
        rightTwist.setPosition(TWIST_POS_1);

        wrist = ahwMap.get(Servo.class, "WRIST");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(WRIST_POS_1);

        claw = ahwMap.get(Servo.class, "CLAW");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(CLAW_POS_1);

        guide = ahwMap.get(Servo.class, "GUIDE");
        guide.setDirection(Servo.Direction.FORWARD);
        guide.setPosition(GUIDE_POS_1);

        flag = ahwMap.get(Servo.class, "FLAG");
        flag.setDirection(Servo.Direction.FORWARD);
        flag.setPosition(FLAG_POS_1);

        sensor = ahwMap.get(RevColorSensorV3.class, "SENSOR");

    }
    public void towersPositionMode() {
        leftTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTower.setTargetPosition(2);
        rightTower.setTargetPosition(2);
        leftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveTowers(double speed) {
        leftTower.setPower(speed);
        rightTower.setPower(speed);
    }
    public void setTowerTarget(int targetPos) {
        leftTower.setTargetPosition(targetPos);
        rightTower.setTargetPosition(targetPos);
    }

    public void moveTwists(boolean pos) {
        leftTwist.setPosition(pos ? TWIST_POS_2 : TWIST_POS_1);
        rightTwist.setPosition(pos ? TWIST_POS_2 : TWIST_POS_1);
    }

    public void moveWrist(boolean pos) {
        wrist.setPosition(pos ? WRIST_POS_2 : WRIST_POS_1);
    }

    public void moveClaw(boolean pos) {
        claw.setPosition(pos ? CLAW_POS_2 : CLAW_POS_1);
    }

    public void moveGuide(boolean pos) { guide.setPosition(pos ? GUIDE_POS_2 : GUIDE_POS_1); }

    public void moveFlag(boolean pos) { flag.setPosition(pos ? FLAG_POS_2 : FLAG_POS_1); }
}