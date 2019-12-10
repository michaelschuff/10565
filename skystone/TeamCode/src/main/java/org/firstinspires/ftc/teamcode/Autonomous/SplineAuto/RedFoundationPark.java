package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


@Config
@Autonomous(group = "Auto")
public class RedFoundationPark extends LinearOpMode{
    private SampleMecanumDriveREVOptimized drive;
    public static int foundation = 30;
    public static int skybridge = 25;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-31.5,61.5,0));

        waitForStart();
        drive.releaseIntake();
        drive.followTrajectorySync(drive.trajectoryBuilder().back(foundation).build());
        drive.toggleFoundation();
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(foundation).build());
        drive.toggleFoundation();
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(skybridge).build());
    }
}
