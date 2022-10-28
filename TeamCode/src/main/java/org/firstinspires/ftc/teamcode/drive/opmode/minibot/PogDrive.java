package org.firstinspires.ftc.teamcode.drive.opmode.minibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Minibot Simple POV Drive")
public class PogDrive extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;

    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "LEFT");
        right = hardwareMap.get(DcMotor.class, "RIGHT");

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);


        double left_power;
        double right_power;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int strafeDirection = 1; // negative or positive
            double moveSpeed = -gamepad1.left_stick_y;
            double turnSpeed = gamepad1.right_stick_x;

            left_power = moveSpeed + turnSpeed;
            right_power = moveSpeed - turnSpeed;


            double speedChange1;
            if (gamepad1.left_bumper) {
                speedChange1 = 0.2;
            } else if (gamepad1.right_bumper) {
                speedChange1 = 1;
            } else {
                speedChange1 = 0.5;
            }

            left_power *= speedChange1;
            right_power *= speedChange1;


            right.setPower(right_power);
            left.setPower(left_power);


            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left", left_power);
            telemetry.addData("Front Right", right_power);
            telemetry.update();


        }
    }
}