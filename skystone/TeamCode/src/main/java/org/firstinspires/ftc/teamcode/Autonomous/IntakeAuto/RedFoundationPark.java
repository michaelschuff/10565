package org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

@Config
@Autonomous(group="Auto")
public class RedFoundationPark extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;

    private static double startingAngle = -90, startingX = 23.25 * 2 - 17.5 / 2, startingY = -70.5 + 17.5 / 2;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(startingX,  startingY, Math.toRadians(startingAngle)));

        if (isStopRequested()) return;

        waitForStart();
        sleep(15000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(42.5, -38))
                        .build()
        );
        grabFoundation();
        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");

            BufferedWriter fileOut = new BufferedWriter(new FileWriter(file));
            fileOut.write(Double.toString(Math.toRadians(startingAngle) - drive.getRawExternalHeading()));
            fileOut.close();

        } catch (Exception e) {

        }

    }

    private void grabFoundation() {
        drive.setFoundation((short) 2);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(43, -33))
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(38, -54))
                        .build()
        );

        drive.turnSync(Math.toRadians(-135));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(20)
                        .build()
        );
        drive.setFoundation((short) 0);
        sleep(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, -60, Math.toRadians(180)))
                        .build()
        );
    }
}
