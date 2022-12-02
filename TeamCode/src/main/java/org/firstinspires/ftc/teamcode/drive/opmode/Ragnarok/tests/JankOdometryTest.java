package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;


@Autonomous(name="odometry test")
public class JankOdometryTest extends LinearOpMode {

    private void mainTrajectory() {
        forward(24. * 3., 0.5);
        sleep(1000);
        right(24., 0.3);
    }

    double INCH_TO_TICKS = 177962./96.;

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private Encoder straightEncoder;
    private Encoder strafeEncoder;



    @Override
    public void runOpMode() {
        front_left = hardwareMap.get(DcMotor.class, "FRONT LEFT");
        front_right = hardwareMap.get(DcMotor.class, "FRONT RIGHT");
        back_left = hardwareMap.get(DcMotor.class, "BACK LEFT");
        back_right = hardwareMap.get(DcMotor.class, "BACK RIGHT");

        straightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FRONT RIGHT"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BACK LEFT"));

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

        waitForStart();

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mainTrajectory();

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive()) {
            telemetry.addData("Straight Distance", straightEncoder.getCurrentPosition());
            telemetry.addData("Strafe Distance", strafeEncoder.getCurrentPosition());
            telemetry.update();
            sleep(167);
        }
    }

    private void forward(double in, double power) {
        int startingPos = straightEncoder.getCurrentPosition();
        int targetPos = (int) (in * INCH_TO_TICKS) + startingPos;
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

    private void backward(double in, double power) {
        int startingPos = straightEncoder.getCurrentPosition();
        int targetPos = (int) (in * INCH_TO_TICKS) + startingPos;
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
        int targetPos = (int) (in * INCH_TO_TICKS) + startingPos;
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
        int targetPos = (int) (in * INCH_TO_TICKS) + startingPos;
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
}
