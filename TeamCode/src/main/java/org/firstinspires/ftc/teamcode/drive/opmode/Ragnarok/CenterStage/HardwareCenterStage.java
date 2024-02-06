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

package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class HardwareCenterStage {
    /* Public OpMode members. */
    public DcMotorEx leftTower = null;
    public DcMotorEx rightTower = null;
    public static final int MAX_TOWER_HEIGHT = 4000;
    public DcMotorEx intake = null;

    public Servo leftArm  = null;
    public Servo rightArm = null;

    public static double ARM_POS_1 = 0.01;
    public static double ARM_POS_2 = 0.53;

    public CRServo bucket = null;
    public Servo launchArm = null;
    public static double LAUNCH_ARM_POS_1 = 0;
    public static double LAUNCH_ARM_POS_2 = 0.45;
    public CRServo counterRoller = null;
    public Servo trigger = null;
    public static double TRIGGER_POS_1 = 0;
    public static double TRIGGER_POS_2 = 0.3;
    public Servo liftIntake = null;
    public static double LIFT_INTAKE_POS_1 = 0;
    public static double LIFT_INTAKE_POS_2 = 1;

    /* local OpMode members. */
    HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareCenterStage(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        // Define and Initialize Motors
        leftTower = ahwMap.get(DcMotorEx.class, "LEFT TOWER");
        leftTower.setDirection(DcMotorEx.Direction.FORWARD);
        rightTower = ahwMap.get(DcMotorEx.class, "RIGHT TOWER");
        rightTower.setDirection(DcMotorEx.Direction.REVERSE);
        intake = ahwMap.get(DcMotorEx.class, "INTAKE");
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftTower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        leftArm = ahwMap.get(Servo.class, "LEFT ARM");
        leftArm.setDirection(Servo.Direction.FORWARD);
        leftArm.setPosition(ARM_POS_1);

        rightArm = ahwMap.get(Servo.class, "RIGHT ARM");
        rightArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setPosition(ARM_POS_1);

        bucket = ahwMap.get(CRServo.class, "BUCKET");
        bucket.setDirection(CRServo.Direction.REVERSE);

        launchArm = ahwMap.get(Servo.class, "LAUNCH ARM");
        launchArm.setDirection(Servo.Direction.FORWARD);

        counterRoller = ahwMap.get(CRServo.class, "COUNTER ROLLER");
        counterRoller.setDirection(CRServo.Direction.REVERSE);

        trigger = ahwMap.get(Servo.class, "TRIGGER");
        trigger.setDirection(Servo.Direction.REVERSE);

        liftIntake = ahwMap.get(Servo.class, "LIFT INTAKE");
        liftIntake.setDirection(Servo.Direction.REVERSE);
        liftIntake.setPosition(LIFT_INTAKE_POS_2);
    }

    public void towersPositionMode() {
        leftTower .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTower .setTargetPosition(2);
        rightTower.setTargetPosition(2);
        leftTower .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveTowers(double speed) {
        leftTower .setPower(speed);
        rightTower.setPower(speed);
    }
    public void setTowerTarget(int targetPos) {
        int x = Math.max(0, Math.min(targetPos, MAX_TOWER_HEIGHT));
        leftTower .setTargetPosition(x);
        rightTower.setTargetPosition(x);
    }

    public void moveIntake(double power) {
        intake.setPower(power);
        counterRoller.setPower(power);
    }

    public void moveArm(boolean pos) {
        leftArm .setPosition(pos ? ARM_POS_2 : ARM_POS_1);
        rightArm.setPosition(pos ? ARM_POS_2 : ARM_POS_1);
    }

    public void setArmPosition(double pos) {
        leftArm .setPosition(pos);
        rightArm.setPosition(pos);
    }

    public void moveBucket(double power) {
        bucket.setPower(power);
    }

    public void moveLaunchArm(boolean pos) {
        launchArm.setPosition(pos ? LAUNCH_ARM_POS_2 : LAUNCH_ARM_POS_1);
    }
    public void moveTrigger(boolean pos) {
        trigger.setPosition(pos ? TRIGGER_POS_2 : TRIGGER_POS_1);
    }
}