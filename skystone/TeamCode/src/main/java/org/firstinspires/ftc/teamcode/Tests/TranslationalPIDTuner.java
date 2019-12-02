package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.TRANSLATIONAL_PID;

@Config
@Autonomous(group = "Tests")
public class TranslationalPIDTuner extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private PIDFController horizontalPIDController;
    private PIDFController verticalPIDController;

    private double x = 0, y = 0;
    public static double targetX = 0, targetY = 0;

    public static double maxPower = .25;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        horizontalPIDController = new PIDFController(TRANSLATIONAL_PID);
        horizontalPIDController.setOutputBounds(-maxPower, maxPower);

        verticalPIDController = new PIDFController(TRANSLATIONAL_PID);
        verticalPIDController.setOutputBounds(-maxPower, maxPower);

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            horizontalPIDController.setTargetPosition(targetX);
            verticalPIDController.setTargetPosition(targetY);

            drive.updatePoseEstimate();
            x = horizontalPIDController.update(drive.getLocalizer().getPoseEstimate().getX());
            y = verticalPIDController.update(drive.getLocalizer().getPoseEstimate().getY());

            drive.setMotorPowers(x - y, x + y, x - y, x + y);


            telemetry.addData("x output: ", x);
            telemetry.addData("x target: ", targetX);
            telemetry.addData("x position: ", drive.getPoseEstimate().getX());
            telemetry.addData("x error: ", targetX - drive.getPoseEstimate().getX());

            telemetry.addData("y output: ", y);
            telemetry.addData("y target: ", verticalPIDController.getTargetPosition());
            telemetry.addData("y position: ", drive.getPoseEstimate().getY());
            telemetry.addData("y error: ", targetY - drive.getPoseEstimate().getY());
            telemetry.update();
        }
    }
}
