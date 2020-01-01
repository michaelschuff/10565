package org.firstinspires.ftc.teamcode.Tests.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Disabled
@Autonomous(group = "HardwareTests")
public class RunIntake extends OpMode {
    private SampleMecanumDriveREVOptimized drive;

    boolean a = false;

    @Override
    public void init() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setIntakePower(0, -1);
    }

    @Override
    public void loop() {
        if (gamepad1.a && !a) {
            a = true;
            drive.setIntakePower(-1, -1);
        } else {
            a = false;
        }
    }
}
