package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous(group="MovementTests")
public class StrafeTurnTest extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;

    public static double x = 48, y = 48, initialH = 0, finalH = 180, startingAngle = -90, startingX = -23.25 * 2 + 17.5 / 2, startingY = 70.5 - 17.5 / 2;;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(startingX, startingY, Math.toRadians(startingAngle)));
        if (isStopRequested()) return;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        LinearInterpolator headingInterpolator = new LinearInterpolator(Math.toRadians(initialH), Math.toRadians(finalH));
        Trajectory trajectory = drive.trajectoryBuilder().lineTo(new Vector2d(x, y), headingInterpolator).build();

        waitForStart();

        drive.followTrajectorySync(trajectory);

    }
}
