package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

@Autonomous(group = "Auto", name = "BlueFoundationOnly")
public class BlueFoundationOnly extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .strafeRight(5)
                .back(30)
                .build()
        );

        grabFoundation();

        try {
            BufferedWriter fileOut = new BufferedWriter(new FileWriter(new File("../Data/StartingDirection.txt")));
            fileOut.write(String.valueOf(Math.toDegrees(drive.getRawExternalHeading())));
            fileOut.close();

        } catch (Exception e) {

        }
    }

    private void grabFoundation() {
        drive.toggleFoundation();
        sleep(500);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(15)
                        .splineTo(new Pose2d(20, 5, Math.toRadians(90)))
                        .back(15)
                        .build()
        );
        drive.toggleFoundation();
        sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(15).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(30).build());
        drive.releaseIntake();
    }
}
