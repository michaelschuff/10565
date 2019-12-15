package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous(group = "Auto")
public class BridgeParkPlaceRobotToLeft24in extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        if(isStopRequested()) return;
        waitForStart();

        drive.releaseIntake();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .forward(25)
                .strafeRight(24)
                .build()
        );
    }
}
