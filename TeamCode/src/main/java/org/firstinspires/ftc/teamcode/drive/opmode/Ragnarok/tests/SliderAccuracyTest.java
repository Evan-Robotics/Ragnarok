package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;


@Config
@TeleOp()
public class SliderAccuracyTest extends LinearOpMode {
    public static int HEIGHT = 1000;

    @Override
    public void runOpMode() {
        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);

        robot.moveClaw(true);

        robot.rightTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.rightTower.setTargetPosition(-HEIGHT);
        robot.leftTower.setTargetPosition(-HEIGHT);

        robot.rightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightTower.setPower(0.5);
        robot.leftTower.setPower(0.5);

        while (robot.rightTower.isBusy() || robot.leftTower.isBusy()) {

        }
        sleep(2000);
//
//        robot.rightTower.setTargetPosition(-HEIGHT / 2);
//        robot.leftTower.setTargetPosition(-HEIGHT / 2);
//
//        robot.rightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.rightTower.setPower(0.5);
//        robot.leftTower.setPower(0.5);
//
//        while (robot.rightTower.isBusy() || robot.leftTower.isBusy()) {
//
//        }
//        sleep(2000);
//
//        robot.rightTower.setTargetPosition((int) (-HEIGHT * 1.5));
//        robot.leftTower.setTargetPosition((int) (-HEIGHT * 1.5));
//
//        robot.rightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.rightTower.setPower(0.5);
//        robot.leftTower.setPower(0.5);
//
//        while (robot.rightTower.isBusy() || robot.leftTower.isBusy()) {
//
//        }
//        sleep(2000);

        while (opModeIsActive()) {
            telemetry.addData("Tower Height", robot.leftTower.getCurrentPosition());
        }

    }
}