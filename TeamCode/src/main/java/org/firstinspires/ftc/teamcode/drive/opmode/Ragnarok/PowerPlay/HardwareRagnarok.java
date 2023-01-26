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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class HardwareRagnarok {
    /* Public OpMode members. */
    public DcMotor leftTower = null;
    public DcMotor rightTower = null;

    public Servo leftTwist = null;
    public Servo rightTwist= null;

    public static double TWIST_POS_1 = .895;
    public static double TWIST_POS_2 = .23;

    public Servo wrist = null;
    public static double WRIST_POS_1 = .722;
    public static double WRIST_POS_2 = 0.04;

    public Servo claw = null;
    public static double CLAW_POS_1 = 0.09;
    public static double CLAW_POS_2 = .5;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRagnarok(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        // Define and Initialize Motors
        leftTower = ahwMap.get(DcMotor.class, "LEFT TOWER");
        leftTower.setDirection(DcMotor.Direction.REVERSE);
        rightTower = ahwMap.get(DcMotor.class, "RIGHT TOWER");
        rightTower.setDirection(DcMotor.Direction.FORWARD);

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

    }
    public void moveTowers(double speed) {
        leftTower.setPower(-speed);
        rightTower.setPower(-speed);
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
}