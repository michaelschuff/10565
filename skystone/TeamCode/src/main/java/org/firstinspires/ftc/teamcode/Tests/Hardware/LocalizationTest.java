package org.firstinspires.ftc.teamcode.Tests.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@Config
@TeleOp(group = "HardwareTests")
public class LocalizationTest extends LinearOpMode {
    private SampleMecanumDriveBase drive;

    private DcMotor frontEncoder, leftEncoder, rightEncoder;

    private double x, y, rotation, maxPower;
    private double[] motorPowers = new double[]{0, 0, 0, 0};

    public static double startingX = 0, startingY = 0, startingHeading = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(startingX, startingY, Math.toRadians(startingHeading)));

        leftEncoder = hardwareMap.get(DcMotor.class, "lIntake");
        frontEncoder = hardwareMap.get(DcMotor.class, "fLift");

        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isStopRequested()) return;
        waitForStart();
        while(!isStopRequested()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            x = x * Math.abs(x);
            y = y * Math.abs(y);

            rotation = Math.pow(gamepad1.right_stick_x, 3);

            motorPowers = new double[]{y + x + rotation, y - x + rotation, y + x - rotation, y - x - rotation};

            if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {
                maxPower = GetMaxAbsMotorPower();
                drive.setMotorPowers(motorPowers[0] / maxPower, motorPowers[1] / maxPower, motorPowers[2] / maxPower, motorPowers[3] / maxPower);
            } else {
                drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            }

            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("leftCount", leftEncoder.getCurrentPosition());
            telemetry.addData("frontCount", frontEncoder.getCurrentPosition());
            telemetry.update();

            drive.updatePoseEstimate();
            if (isStopRequested()) return;
        }

    }

    private double GetMaxAbsMotorPower() {
        return Math.max(Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1])), Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));
    }
}
