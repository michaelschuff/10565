package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Tests")
public class LinearTest extends LinearOpMode {
    public static double dx = 46.5;
    public static double dy = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(dx)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
