package org.firstinspires.ftc.teamcode.Tests.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous
public class LiftTest extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;

    public static int pos = 500;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        if (isStopRequested()) return;
        waitForStart();

        drive.setLiftPos(pos);
        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.setLiftPower(0.5);

        while(!isStopRequested()){
            telemetry.addData("liftpos", (float) drive.lift.getCurrentPosition() / drive.InchesToLiftTicks);
        }
    }
}
