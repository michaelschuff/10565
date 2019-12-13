package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.TaskThread;
import org.firstinspires.ftc.teamcode.util.ThreadLinearOpMode;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

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

        try {
            BufferedWriter fileOut = new BufferedWriter(new FileWriter(new File("../Data/StartingDirection.txt")));
            fileOut.write(String.valueOf(Math.toDegrees(drive.getRawExternalHeading())));
            fileOut.close();

        } catch (Exception e) {

        }
    }
}
