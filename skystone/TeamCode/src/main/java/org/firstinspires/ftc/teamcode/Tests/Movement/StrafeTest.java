package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous(group = "MovementTests")
public class StrafeTest extends LinearOpMode {
    public static double x = 24, y = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);


        waitForStart();

        telemetry.addLine("rdy");
        telemetry.update();
        if (isStopRequested()) return;

        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(new Vector2d(x, y)).build());
    }

}
