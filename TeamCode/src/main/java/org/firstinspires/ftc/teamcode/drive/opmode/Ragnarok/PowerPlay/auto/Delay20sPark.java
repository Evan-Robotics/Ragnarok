package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class Delay20sPark extends LinearOpMode {


    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private Servo claw;


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        front_left = hardwareMap.get(DcMotor.class, "FRONT LEFT");
        front_right = hardwareMap.get(DcMotor.class, "FRONT RIGHT");
        back_left = hardwareMap.get(DcMotor.class, "BACK LEFT");
        back_right = hardwareMap.get(DcMotor.class, "BACK RIGHT");

        claw = hardwareMap.get(Servo.class, "CLAW");


        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        /*
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        claw.setPosition(0.05);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        claw.setPosition(0.5);


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


        sleep(20000);

        timeMove(-0.5, -0.5, -0.5, -0.5);
        sleep(1600);
        timeMove(0, 0, 0, 0);
        sleep(500);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void timeMove(double fl_speed, double fr_speed, double bl_speed, double br_speed) {
        front_left.setPower(fl_speed);
        front_right.setPower(fr_speed);
        back_left.setPower(bl_speed);
        back_right.setPower(br_speed);

    }
}