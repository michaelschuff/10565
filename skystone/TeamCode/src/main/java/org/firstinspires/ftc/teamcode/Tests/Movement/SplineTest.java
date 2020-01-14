package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "MovementTests")
public class SplineTest extends LinearOpMode {
    public static double dx = 46.5;
    public static double dy = 46.5;
    public static double dh = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        Trajectory trajectory = drive.trajectoryBuilder()
                .splineTo(new Pose2d(10, 10, 0))
                .splineTo(new Pose2d(20, 0, Math.toRadians(-90)))
                .splineTo(new Pose2d(10, -10, Math.toRadians(180)))
                .splineTo(new Pose2d(0, 0, Math.toRadians(90)))
                .splineTo(new Pose2d(-10, 10, Math.toRadians(180)))
                .splineTo(new Pose2d(-20, 0, Math.toRadians(-90)))
                .splineTo(new Pose2d(-10, -10, Math.toRadians(0)))
                .splineTo(new Pose2d(0, 0, Math.toRadians(90)))
                .build();
        waitForStart();
        while(!isStopRequested()) {
            drive.followTrajectorySync(trajectory);
        }


    }
}
