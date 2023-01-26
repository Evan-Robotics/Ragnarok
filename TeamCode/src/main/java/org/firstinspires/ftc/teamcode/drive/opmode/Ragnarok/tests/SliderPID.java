package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;

@Config
@Autonomous
public class SliderPID extends LinearOpMode {
    public static int TOLERANCE = 10;
    public static int HEIGHT = 1000;

    public static double Kp = 0.001;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kg = 0.1;

    HardwareRagnarok robot = new HardwareRagnarok();

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.moveClaw(true);

        robot.leftTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.moveTowers(0);

        waitForStart();

        robot.leftTower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightTower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(200);
        runToPosition(HEIGHT);

        robot.moveTowers(Kg);

        sleep(2000);

        runToPosition(HEIGHT + 500);

        robot.moveTowers(Kg);

        sleep(2000);

        runToPosition(HEIGHT);

        robot.moveTowers(Kg);

        sleep(2000);

        runToPosition(HEIGHT - 500);

        sleep(10000);

    }

    public void runToPosition(int target) {
        double integralSum = 0;
        double lastError = 0;

        timer.reset();


        while (Math.abs(target + (robot.leftTower.getCurrentPosition() + robot.rightTower.getCurrentPosition())/2) > TOLERANCE && opModeIsActive()) {
            int encoderPosition = -(robot.leftTower.getCurrentPosition() + robot.rightTower.getCurrentPosition())/2;
            int error = target - encoderPosition;

            double derivative = (error - lastError) / timer.seconds();

            integralSum += error * timer.seconds();

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + Kg;

            robot.moveTowers(out);

            lastError = error;

            timer.reset();

            telemetry.addData("target pos", target);
            telemetry.addData("current pos", -(robot.leftTower.getCurrentPosition() + robot.rightTower.getCurrentPosition())/2);
            telemetry.addData("speed", out);
            telemetry.addData("P", Kp * error);
            telemetry.addData("I", Ki * integralSum);
            telemetry.addData("D", Kd * derivative);
            telemetry.update();
        }
    }
}
