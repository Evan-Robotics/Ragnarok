package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PoseStorage;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name = "[MAIN] Java TeleOp", group = "Ragnarok")
public class JavaTeleOp_v1 extends LinearOpMode {

    // Driving variables
    enum Mode {
        NORMAL_CONTROL,
        TANK_CONTROL,
        GRID          // testing
    }

    private double bound(double min, double val, double max) {
        return Math.min(Math.max(min, val),max);
    }

    double speedChange1;
    double speedChange2;

    double clawPos = 0;
    int towerHeight = 0;


    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    //Target Position of Storage Unit
    private Vector2d storage_pos = new Vector2d(-66, 37);

    //Timers
    private final ElapsedTime motorTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializations
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);


        waitForStart();

        ////VARIABLES\\\\\
        // (for servos, when we have them)

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            //Gamepad Input
            double ly;
            double lx;
            double rx;

            //Initialize Localizer
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            Pose2d driveDirection = new Pose2d();

            //Initialize FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            //TelemetryPacket packet = new TelemetryPacket();

            //// Gamepad Controls \\\\

            // Speed change
            if (gamepad1.left_bumper) {
                speedChange1 = 0.5;
            } else if (gamepad1.right_bumper) {
                speedChange1 = 1;
            } else {
                speedChange1 = 0.7;
            }



            if (gamepad2.left_bumper) {
                speedChange2 = 0.5;
            } else if (gamepad1.right_bumper) {
                speedChange2 = 1;
            } else {
                speedChange2 = 0.7;
            }


            double clawSpeedFactor = 0.5;
            double clawMin = 0;
            double clawMax = 1;
            clawPos += (gamepad2.right_trigger - gamepad2.left_trigger) * speedChange2 * clawSpeedFactor;
            clawPos = bound(clawMin, clawPos, clawMax);



            double stickSpeed = 1;
            towerHeight += gamepad2.right_stick_y * stickSpeed;

            robot.rightTower.setTargetPosition(towerHeight);
            robot.leftTower.setTargetPosition(towerHeight);

            int ERROR_TOLERATION = 10;

            double towerSpeedFactor = 0.5;

            if ( (Math.abs(towerHeight - robot.leftTower.getCurrentPosition()) >= ERROR_TOLERATION
               || Math.abs(towerHeight - robot.rightTower.getCurrentPosition()) >= ERROR_TOLERATION)
               && Math.abs(robot.leftTower.getCurrentPosition() - robot.rightTower.getCurrentPosition()) <= ERROR_TOLERATION) {
                robot.rightTower.setPower(speedChange2 * towerSpeedFactor);
                robot.leftTower.setPower(speedChange2 * towerSpeedFactor);
            }


            if (gamepad1.start && gamepad1.dpad_up) {
                drive.getLocalizer().setPoseEstimate(new Pose2d(-63, 60, Math.PI/2));
            }

            switch (currentMode) {
            //Distance to Tower


                case NORMAL_CONTROL:

                    ly = -gamepad1.left_stick_y;
                    lx = -gamepad1.left_stick_x;
                    rx = -gamepad1.right_stick_x;

                    //Normal Robot Control
                    driveDirection = new Pose2d(
                            Math.abs(ly) * ly * speedChange1,
                            Math.abs(lx) * lx * speedChange1,
                            Math.abs(rx) * rx * speedChange1
                    );

                    // Switch to tank if gamepad1 left dpad is activated
                    if (gamepad1.dpad_left) {
                        currentMode = Mode.GRID;
                    }
                    break;

                case GRID: // Switch to normal control if gamepad1 dpad right is activated

                    if (gamepad1.dpad_right) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    ly = -gamepad1.left_stick_y;
                    lx = -gamepad1.left_stick_x;
                    rx = -gamepad1.right_stick_x;

                    Vector2d fieldFrameInput = new Vector2d(
                            Math.signum(ly) * Math.pow(ly, 2) * speedChange1,
                            Math.signum(lx) * Math.pow(lx, 2) * speedChange1
                    );

                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                    //headingController.setTargetPosition(rx * speedChange1);

                    double headingInput = Math.signum(rx) * Math.pow(rx, 2) * speedChange1;

                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    break;

            }

            //Dashboard View
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
            drive.setWeightedDrivePower(driveDirection);
            headingController.update(poseEstimate.getHeading());
            drive.getLocalizer().update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            //Driver Station Telemetry

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    private static double[] hToServoPos(int h) {
        return new double[2];
    }

}
