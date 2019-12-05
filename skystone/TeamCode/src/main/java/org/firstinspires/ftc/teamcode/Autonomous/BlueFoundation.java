package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

public class BlueFoundation extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        if (isStopRequested()) return;

        waitForStart();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(2 * (22.875 + .75) - 9)
                .build());

        drive.toggleFoundation();

        drive.waitForIdle();


    }
}
