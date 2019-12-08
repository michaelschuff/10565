package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.TaskThread;
import org.firstinspires.ftc.teamcode.util.ThreadLinearOpMode;

public class BlueFoundation extends ThreadLinearOpMode {
    private SampleMecanumDriveREVOptimized drive;

    @Override
    public void runMainOpMode() {
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                telemetry.addData("yeet", "it works");
            }
        }));
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        if (isStopRequested()) return;

        waitForStart();

        drive.followTrajectorySync(drive.trajectoryBuilder().back(2 * (22.875 + .75) - 9).build());

        drive.toggleFoundation();

        drive.waitForIdle();


    }
}
