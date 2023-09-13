package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Simple Drive")
public class BasicDrive extends LinearOpMode {
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    @Override
    public void runOpMode() {


        front_left = hardwareMap.get(DcMotor.class, "FRONT LEFT");
        front_right = hardwareMap.get(DcMotor.class, "FRONT RIGHT");
        back_left = hardwareMap.get(DcMotor.class, "BACK LEFT");
        back_right = hardwareMap.get(DcMotor.class, "BACK RIGHT");

        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double front_left_power;
        double front_right_power;
        double back_left_power;
        double back_right_power;

        String run_type = "pov";


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_left) {
                run_type = "pov";
            } else if(gamepad1.dpad_right) {
                run_type = "tank";
            }

            front_left_power = 0;
            front_right_power = 0;
            back_left_power = 0;
            back_right_power = 0;

            switch(run_type) {

                case "pov": {
                    int strafeDirection = 1; // negative or positive
                    int turnDirection = -1; // neg or pos
                    double moveSpeed = -gamepad1.left_stick_y;
                    double turnSpeed = turnDirection * gamepad1.right_stick_x;
                    double strafeSpeed = strafeDirection * gamepad1.left_stick_x;

                    front_left_power  = moveSpeed + turnSpeed + strafeSpeed;
                    front_right_power = moveSpeed - turnSpeed - strafeSpeed;
                    back_left_power   = moveSpeed + turnSpeed - strafeSpeed;
                    back_right_power  = moveSpeed - turnSpeed + strafeSpeed;
                    break;
                }

                case "tank": {
                    front_left_power = -gamepad1.left_stick_y;
                    front_right_power = -gamepad1.right_stick_y;
                    back_left_power = -gamepad1.left_stick_y;
                    back_right_power = -gamepad1.right_stick_y;
                    break;
                }
                default: {
                    telemetry.addData("ERROR", "Run type not found");
                }
            }
            double speedChange1;
            if (gamepad1.left_bumper) {
                speedChange1 = 0.2;
            } else if (gamepad1.right_bumper) {
                speedChange1 = 1;
            } else {
                speedChange1 = 0.6;
            }

            front_left_power *= speedChange1;
            front_right_power *= speedChange1;
            back_left_power *= speedChange1;
            back_right_power *= speedChange1;


            front_right.setPower(front_right_power);
            front_left.setPower(front_left_power);
            back_right.setPower(back_right_power);
            back_left.setPower(back_left_power);


            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left", front_left_power);
            telemetry.addData("Front Right", front_right_power);
            telemetry.addData("Back Left", back_left_power);
            telemetry.addData("Back Right", back_right_power);
            telemetry.addData("Run Type", run_type);
            telemetry.update();


        }
    }
}