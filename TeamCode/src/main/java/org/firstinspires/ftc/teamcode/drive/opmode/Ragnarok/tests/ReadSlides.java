package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.HardwareCenterStage;

@TeleOp
public class ReadSlides extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareCenterStage robot = new HardwareCenterStage();
        robot.init(hardwareMap);
        robot.leftTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.moveTowers(0);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Left Tower Pos", robot.leftTower.getCurrentPosition());
            telemetry.addData("Right Tower Pos", robot.rightTower.getCurrentPosition());
            telemetry.update();
        }
    }
}
