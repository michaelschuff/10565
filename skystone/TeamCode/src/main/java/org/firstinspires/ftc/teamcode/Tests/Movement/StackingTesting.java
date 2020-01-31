package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Disabled
@Config
@Autonomous
public class StackingTesting extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;

    public static int liftPos = 500, secondLiftPosition = 800, firstLiftDelay = 2000, secondLiftDelay = 500;
    public static double armPos = 0.8;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        if (isStopRequested()) return;
        drive.setClawGrabbing(true);

        waitForStart();

        if (isStopRequested()) return;

        drive.setArmPos(0.36, 0.64);
        drive.setLiftPos(500);
        drive.bLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.fLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.bLift.setPower(1);
        drive.fLift.setPower(1);
        sleep((long) (500));
        drive.setArmPos(0.8, 0.2);
        sleep((long) (1500));
        drive.setClawGrabbing(false);
        sleep(100);
        drive.setLiftPos(800);
        drive.bLift.setPower(.5);
        drive.fLift.setPower(.5);
        sleep(500);
        drive.setArmPos(0.36, 0.64);
        sleep(100);
        drive.setLiftPos(0);
        drive.bLift.setPower(-.5);
        drive.fLift.setPower(-.5);

        while(!isStopRequested()) {


        }
    }
}
