package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PoseStorage
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok
import org.firstinspires.ftc.teamcode.util.DashboardUtil

const val SPEED_VAL_SLOW = 0.5
const val SPEED_VAL_NORMAL = 0.7
const val SPEED_VAL_FAST = 1.0

@Disabled
@TeleOp
class KotlinBasicDrive : LinearOpMode() {
    var speedChange = 0.5

    private val headingController = PIDFController(SampleMecanumDrive.HEADING_PID)

    override fun runOpMode() {

        val drive = SampleMecanumDrive(hardwareMap)
        val robot = HardwareRagnarok()
        robot.init(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        drive.localizer.poseEstimate = PoseStorage.currentPose
        headingController.setInputBounds(-Math.PI, Math.PI)

        waitForStart()

        if (isStopRequested()) {return}

        while (opModeIsActive() && !isStopRequested) {
            val lx = gamepad1.left_stick_x
            val ly = gamepad1.left_stick_y
            val rx = gamepad1.right_stick_x

            val poseEstimate = drive.localizer.poseEstimate
            var driveDirection = Pose2d()

            var packet = TelemetryPacket()
            var fieldOverlay = packet.fieldOverlay()

            speedChange = if (gamepad1.left_bumper) SPEED_VAL_SLOW
                            else if (gamepad1.right_bumper) SPEED_VAL_FAST
                            else SPEED_VAL_NORMAL

            driveDirection = Pose2d(
                ly * ly * ly * speedChange,
                lx * lx * lx * speedChange,
                rx * rx * rx * speedChange
            )

            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
            drive.setWeightedDrivePower(driveDirection);
            headingController.update(poseEstimate.heading);
            drive.localizer.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


            //Driver Station Telemetry
            telemetry.addData("x", poseEstimate.x)
            telemetry.addData("y", poseEstimate.y)
            telemetry.addData("heading", poseEstimate.heading)
            telemetry.update()
        }
    }
}