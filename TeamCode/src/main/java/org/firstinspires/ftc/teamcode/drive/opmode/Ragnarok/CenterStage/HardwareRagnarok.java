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
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
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
    public DcMotorEx intake = null;

    public Servo leftArm  = null;
    public Servo rightArm = null;

    public static double ARM_POS_1 = 0;
    public static double ARM_POS_2 = 1;

    public CRServo bucket = null;

    public Servo leftDrop = null;
    public Servo rightDrop = null;

    public static double DROP_POS_1 = 0;
    public static double DROP_POS_2 = 1;

    public RevColorSensorV3 sensor = null;

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
        leftTower.setDirection(DcMotorEx.Direction.FORWARD);
        rightTower = ahwMap.get(DcMotorEx.class, "RIGHT TOWER");
        rightTower.setDirection(DcMotorEx.Direction.REVERSE);
        intake = ahwMap.get(DcMotorEx.class, "INTAKE");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set all motors to run without encoders.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftTower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        leftArm = ahwMap.get(Servo.class, "LEFT ARM");
        leftArm.setDirection(Servo.Direction.FORWARD);
        leftArm.setPosition(ARM_POS_1);

        rightArm = ahwMap.get(Servo.class, "RIGHT TWIST");
        rightArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setPosition(ARM_POS_1);

        bucket = ahwMap.get(CRServo.class, "BUCKET");
        bucket.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrop = ahwMap.get(Servo.class, "LEFT DROP");
        leftDrop.setDirection(Servo.Direction.FORWARD);
        leftDrop.setPosition(DROP_POS_1);

        rightDrop = ahwMap.get(Servo.class, "RIGHT DROP");
        rightDrop.setDirection(Servo.Direction.REVERSE);
        rightDrop.setPosition(DROP_POS_1);

        sensor = ahwMap.get(RevColorSensorV3.class, "SENSOR");
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
        leftTower .setTargetPosition(targetPos);
        rightTower.setTargetPosition(targetPos);
    }

    public void moveIntake(double power) {
        intake.setPower(power);
    }

    public void moveArm(boolean pos) {
        leftArm .setPosition(pos ? ARM_POS_2 : ARM_POS_1);
        rightArm.setPosition(pos ? ARM_POS_2 : ARM_POS_1);
    }

    public void moveBucket(double power) {
        bucket.setPower(power);
    }

    public void moveDrop(boolean pos) {
        leftDrop .setPosition(pos ? DROP_POS_2 : DROP_POS_1);
        rightDrop.setPosition(pos ? DROP_POS_2 : DROP_POS_1);
    }
}