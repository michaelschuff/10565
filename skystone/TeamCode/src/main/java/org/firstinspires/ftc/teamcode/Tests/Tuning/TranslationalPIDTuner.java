package org.firstinspires.ftc.teamcode.Tests.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.TRANSLATIONAL_PID;


//have 2 Controllers that each control 2 wheel velocites
@Disabled
@Config
@Autonomous(group = "Tests")
public class TranslationalPIDTuner extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private PIDFController rController;
    private PIDFController lController;

    private double x = 0, y = 0;
    public static double targetX = 0, targetY = 0;

    public static double maxPower = .25;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        rController = new PIDFController(TRANSLATIONAL_PID);
        rController.setOutputBounds(-maxPower, maxPower);

        lController = new PIDFController(TRANSLATIONAL_PID);
        lController.setOutputBounds(-maxPower, maxPower);

        if (isStopRequested()) return;

        while (!isStopRequested()) {

        }
    }
}
