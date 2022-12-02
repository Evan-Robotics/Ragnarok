package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;


@TeleOp()
public class SliderAccuracyTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);

        robot.rightTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.rightTower.setTargetPosition(-1000);
        robot.leftTower.setTargetPosition(-1000);

        robot.rightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightTower.setPower(0.5);
        robot.leftTower.setPower(0.5);

        while (robot.rightTower.isBusy() || robot.leftTower.isBusy()) {
            sleep(1);
        }

    }
}