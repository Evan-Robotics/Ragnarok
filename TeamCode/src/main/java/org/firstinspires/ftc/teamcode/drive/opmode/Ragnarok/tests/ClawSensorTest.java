package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ClawSensorTest extends LinearOpMode {
    final double OPEN_POS = 0.1;
    final double CLOSED_POS = 0.5;
    final double CLOSE_DISTANCE = 12;

    Servo claw;
    RevColorSensorV3 sensor;

    @Override
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "CLAW");
        sensor = hardwareMap.get(RevColorSensorV3.class, "CLAW SENSOR");

        waitForStart();

        while (!isStopRequested() || opModeIsActive()) {
            double distance = sensor.getDistance(DistanceUnit.MM);

            claw.setPosition(gamepad2.a ^ distance <= CLOSE_DISTANCE ? CLOSED_POS : OPEN_POS);

            sensor.enableLed(!gamepad1.b);

            telemetry.addData("distance millimeters", sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("light detected", sensor.getLightDetected());
            telemetry.addData("raw light detected", sensor.getRawLightDetected());
            telemetry.addData("raw light detected max", sensor.getRawLightDetectedMax());
            telemetry.addData("raw optical", sensor.rawOptical());
            telemetry.addData("alpha", sensor.alpha());
            telemetry.addData("argb", sensor.argb());
            telemetry.addData("red", sensor.red());
            telemetry.addData("green", sensor.green());
            telemetry.addData("blue", sensor.blue());
            telemetry.update();
        }
    }
}
