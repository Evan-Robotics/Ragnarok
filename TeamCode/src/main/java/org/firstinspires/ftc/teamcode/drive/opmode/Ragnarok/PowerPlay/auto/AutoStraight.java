package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="Delay & Straight Auto", group="drive")
public class AutoStraight extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        sleep(500);

        sleep(10000);

        drive.setMotorPowers(0.2,0.2,0.2,0.2);

        sleep(5000);

        drive.setMotorPowers(0,0,0,0);
    }
}
