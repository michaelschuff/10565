package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


@Config
@Autonomous(group = "Auto")
public class BlueFoundationPark extends LinearOpMode{
    private SampleMecanumDriveREVOptimized drive;
    public static int a = 30, b = 5, c = 5, d = 10, e = 20, f = 40, g = 10, h = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(-31.5,61.5,0));

        waitForStart();
        drive.followTrajectorySync(drive.trajectoryBuilder().back(a).build());
        drive.toggleFoundation();
        sleep(2000);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(g)
                        .splineTo(new Pose2d(b + g, c, Math.toRadians(90)))
                        .back(e)
                        .build()
        );
        drive.toggleFoundation();
        sleep(2000);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(d).forward(f).strafeRight(h).build());
        drive.releaseIntake();
    }
}
