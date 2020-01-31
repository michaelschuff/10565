package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Disabled
@Config
@Autonomous(group = "MovementTests")
public class StrafeTest extends LinearOpMode {
    public static double distance = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Trajectory trajectory;
        if (distance > 0) {
            trajectory = drive.trajectoryBuilder()
                    .strafeLeft(distance)
                    .build();
        } else {
            trajectory = drive.trajectoryBuilder()
                    .strafeRight(-distance)
                    .build();
        }


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }

}
