package org.firstinspires.ftc.teamcode.drive.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.HEADING_PID;

@Config
@Autonomous(group = "Tests")
public class TranslationalPIDTuner extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private PIDFController horizontalPIDController;
    private PIDFController verticalPIDController;

    private double x, y;
    public static double targetX = 0, targetY = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        horizontalPIDController = new PIDFController(HEADING_PID);
        horizontalPIDController.setOutputBounds(-1.0, 1.0);

        verticalPIDController = new PIDFController(HEADING_PID);
        verticalPIDController.setOutputBounds(-1.0, 1.0);

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            horizontalPIDController.setTargetPosition(targetX);
            verticalPIDController.setTargetPosition(targetY);

            x = horizontalPIDController.update(drive.getPoseEstimate().getX());
            y = horizontalPIDController.update(drive.getPoseEstimate().getY());

            drive.setMotorPowers(y + x, y - x, y + x, y - x);

            telemetry.addData("x output: ", x);
            telemetry.addData("x target: ", horizontalPIDController.getTargetPosition());
            telemetry.addData("x position: ", targetX);
            telemetry.addData("x error: ", horizontalPIDController.getTargetPosition() - targetX);

            telemetry.addData("y output: ", y);
            telemetry.addData("y target: ", verticalPIDController.getTargetPosition());
            telemetry.addData("y position: ", targetY);
            telemetry.addData("y error: ", verticalPIDController.getTargetPosition() - targetY);
            telemetry.update();
        }
    }
}
