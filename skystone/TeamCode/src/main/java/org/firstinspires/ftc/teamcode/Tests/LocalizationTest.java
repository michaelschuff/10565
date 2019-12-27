package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.TaskThread;
import org.firstinspires.ftc.teamcode.util.ThreadOpMode;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "Tests")
public class LocalizationTest extends LinearOpMode {
    private SampleMecanumDriveBase drive;
    private DcMotor leftEncoder, frontEncoder;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        frontEncoder = hardwareMap.dcMotor.get("rIntake");
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {

            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("leftEncoder", -leftEncoder.getCurrentPosition());
            telemetry.addData("frontEncoder", -frontEncoder.getCurrentPosition());
            telemetry.update();

            drive.updatePoseEstimate();
            if (isStopRequested()) return;
        }

    }
}
