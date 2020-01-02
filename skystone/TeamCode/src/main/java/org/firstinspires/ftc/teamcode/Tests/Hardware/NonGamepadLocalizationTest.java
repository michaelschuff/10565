package org.firstinspires.ftc.teamcode.Tests.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.justLocalizer;


@Config
@TeleOp
public class NonGamepadLocalizationTest extends LinearOpMode {
    private justLocalizer drive;

    public static double startX = 0, startY = 0, startH = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new justLocalizer(hardwareMap);
        drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(startH)));

        if (isStopRequested()) return;
        waitForStart();
        while(!isStopRequested()) {
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();

            drive.updatePoseEstimate();
            if (isStopRequested()) return;
        }
    }
}
