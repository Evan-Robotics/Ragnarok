package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

const val SLOW_SPEED = 0.2
const val NORMAL_SPEED = 0.5
const val FAST_SPEED = 0.7

@TeleOp(name="TEST - minibot kotlin")
class KotlinTest : LinearOpMode() {
    val left: DcMotor = hardwareMap.get(DcMotor::class.java, "LEFT")
    val right: DcMotor = hardwareMap.get(DcMotor::class.java, "RIGHT")


    override fun runOpMode() {
        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.REVERSE

        var leftPower: Double
        var rightPower: Double

        var speedChange: Double

        telemetry.addData("Ready to start", "press play")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val driveSpeed = gamepad1.left_stick_y.toDouble()
            val turnSpeed = gamepad1.right_stick_x.toDouble()

            leftPower = driveSpeed + turnSpeed
            rightPower = driveSpeed - turnSpeed

            speedChange = if (gamepad1.left_bumper) SLOW_SPEED
                            else if (gamepad1.right_bumper) FAST_SPEED
                            else NORMAL_SPEED

            leftPower *= speedChange
            rightPower *= speedChange

            left.power = leftPower
            right.power = rightPower


            telemetry.addData("Status", "Running")
            telemetry.addData("Front Left", leftPower)
            telemetry.addData("Front Right", rightPower)
            telemetry.update()

        }

    }

}