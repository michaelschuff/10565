package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "MovementTests")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public static double STARTINGANGLE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));
    }
}
