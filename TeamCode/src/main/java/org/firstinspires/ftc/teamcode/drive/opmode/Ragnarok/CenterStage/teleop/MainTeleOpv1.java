package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.teleop;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name = "--MAIN-- TeleOp")
public class MainTeleOpv1 extends LinearOpMode {

    // Driving variables
    enum Mode {
        NORMAL_CONTROL,
        TANK_CONTROL,
        GRID          // testing
    }

    public static double MAX_FACTOR = 20.0;
    public static double MIN_FACTOR = 3.0;
    public static int FACTOR_POINT_1 = 10;
    public static int FACTOR_POINT_2 = 30;
    int MAX_TOWER_HEIGHT = 3178;

    private double bound(double min, double val, double max) {
        return Math.min(Math.max( min, val), max);
    }
    double f(double d) {return (MIN_FACTOR-MAX_FACTOR)/(FACTOR_POINT_2 - FACTOR_POINT_1) * (d - FACTOR_POINT_1) + MIN_FACTOR;}

    double speedChange1;
    double speedChange2;

    int towerHeight = 0;
    int TOWER_SPEED_FACTOR = 50;

    // int targetTowerPosition;

    boolean clawPos = false;

    boolean flipState = false;
    boolean goingDown = false;
    boolean gp2_x_last_frame = false;

    boolean guideState = false;
    boolean gp2_y_last_frame = false;

    boolean sensorLastFrame = false;

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    //Timers
    private final ElapsedTime bucketDepositTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializations
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);
        robot.leftTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);


        waitForStart();

        ////VARIABLES\\\\\
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
            } else if (gamepad2.right_bumper) {
                speedChange2 = 1.5;
            } else {
                speedChange2 = 0.7;
            }

            towerHeight += TOWER_SPEED_FACTOR * -gamepad2.left_stick_y * speedChange2;
            towerHeight = (int) bound(5, towerHeight, MAX_TOWER_HEIGHT * 0.8);

            if (gamepad2.dpad_up) {
                towerHeight = 2350;
            }
            if (gamepad2.dpad_right) {
                towerHeight = 1290;
            }
            if (gamepad2.dpad_down) {
                towerHeight = 5;
            }

            double verticalSpeedGravityFactor = towerHeight - robot.leftTower.getCurrentPosition() < 0 ? 0.5 : 1;
            double verticalSpeedDistanceFactor = bound(MIN_FACTOR, f(Math.abs(towerHeight - robot.leftTower.getCurrentPosition())), MAX_FACTOR);

            robot.leftTower.setTargetPosition(towerHeight);
            robot.rightTower.setTargetPosition(towerHeight);
            robot.leftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftTower.setPower(speedChange2 * verticalSpeedGravityFactor * verticalSpeedDistanceFactor);
            robot.rightTower.setPower(speedChange2 * verticalSpeedGravityFactor * verticalSpeedDistanceFactor);


            if (gamepad2.x && !gp2_x_last_frame) {
                flipState = !flipState;
            }
            gp2_x_last_frame = gamepad2.x;

            robot.moveArm(flipState);

            ly = -gamepad1.left_stick_y;
            lx = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x;

                    //Normal Robot Control
                    driveDirection = new Pose2d(
                            ly * ly * ly * speedChange1,
                            lx * lx * lx * speedChange1,
                            rx * rx * rx * speedChange1
                    );

                    // Switch to tank if gamepad1 left dpad is activated
                    if (gamepad1.dpad_left) {
                        currentMode = Mode.GRID;
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
            telemetry.addData("Target Tower Height", towerHeight);
            telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);
            telemetry.addData("Left Tower Height", robot.leftTower.getCurrentPosition());
            telemetry.addData("Right Tower Height", robot.rightTower.getCurrentPosition());
            // telemetry.addData("Tower Speeds", leftTowerSpeed + ", " + rightTowerSpeed);
            telemetry.update();
        }
    }
}
