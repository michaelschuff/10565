package org.firstinspires.ftc.teamcode.Tests.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.HEADING_PID;

@Config
@Autonomous(group = "PIDTuningTests")
public class HeadingPIDTuner extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    private PIDFController absoluteRotationPIDController;

    private double rotation;
    public static double TargetDirection = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        absoluteRotationPIDController = new PIDFController(HEADING_PID);
        absoluteRotationPIDController.setInputBounds(0.0, 2.0 * Math.PI);
        absoluteRotationPIDController.setOutputBounds(-1.0, 1.0);

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            if (absoluteRotationPIDController.getTargetPosition() != Math.toRadians(TargetDirection)) {
                absoluteRotationPIDController.setTargetPosition(Math.toRadians(TargetDirection));
            }

            rotation = absoluteRotationPIDController.update(drive.getRawExternalHeading());
            drive.setMotorPowers(-rotation, -rotation, rotation, rotation);

            telemetry.addData("output: ", rotation);
            telemetry.addData("target: ", TargetDirection);
            telemetry.addData("heading: ", Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.addData("error: ", Math.toDegrees(Math.toRadians(TargetDirection) - drive.getRawExternalHeading()));
            telemetry.update();
        }
    }
}
