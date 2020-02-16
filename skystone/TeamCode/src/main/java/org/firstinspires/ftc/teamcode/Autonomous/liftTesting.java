package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonReader;
import org.firstinspires.ftc.teamcode.util.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.util.gamepad.GamepadKeys;

@Config
@TeleOp
public class liftTesting extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private PIDFController liftController;
    public static PIDCoefficients liftCoeffs = new PIDCoefficients(3, 0.3, 0.1);
    public static double kV = .1;
    private static double scalar = 1000, maxPow = 1;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private int SkystonePosition = 1;

    private GamepadEx driver;

    private ButtonReader a, y;

    private double lIntakeArm = 0.40, rIntakeArm = 0.60, lStoneArm = 0.37, rStoneArm = 0.63;

    int state = 5;
    @Override
    public void runOpMode() {
        driver = new GamepadEx(gamepad1);
        a = new ButtonReader(driver, GamepadKeys.Button.A);
        y = new ButtonReader(driver, GamepadKeys.Button.Y);
        if (isStopRequested()) {
            drive.setMotorPowers(0, 0, 0, 0);
            return;
        }
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setClawGrabbing(false);
        drive.setArmPos(lIntakeArm, rIntakeArm);
        drive.setPoseEstimate(new Pose2d(-37.75, 61.75, Math.toRadians(-90)));
        liftController = new PIDFController(liftCoeffs, kV);
        liftController.setInputBounds(-100.0 / scalar, 4000.0 / scalar);
        liftController.setOutputBounds(-maxPow, maxPow);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready");
        telemetry.update();

        while(!isStarted()){
            try {
//                SkystonePosition = SkyStoneFinder.detectSkystone(camera, false) + 1;
                telemetry.addData("Skystone", SkystonePosition);
                telemetry.update();
            } catch (NullPointerException e) {
                telemetry.addData("Java Sux bc", e.getStackTrace());
            }
        }
        telemetry.clear();

        if (isStopRequested()) {
            drive.setMotorPowers(0, 0, 0, 0);
            return;
        }

        while (!isStopRequested()) {
            a.readValue();
            y.readValue();

            if (a.wasJustPressed()) {
                state--;
                if (state > 0) {
                    liftController.setTargetPosition(state * 200 / scalar);
                }
            }

            if (y.wasJustPressed()) {
                state++;
                if (state < 10) {
                    liftController.setTargetPosition(state * 200 / scalar);
                }
            }

            if (drive.getLiftPos() < 50) {
                drive.setLiftPower(Math.abs(0.75 + drive.getLiftPos() / 200.0) * liftController.update(drive.getLiftPos() / scalar));
            } else {
                drive.setLiftPower(liftController.update(drive.getLiftPos() / scalar));
            }

            drive.setLiftPower(gamepad1.right_trigger - gamepad1.left_trigger);



            telemetry.addData("pos", drive.getLiftPos());
            telemetry.addData("target", liftController.getTargetPosition() * scalar);
            telemetry.addData("error", drive.getLiftPos() - liftController.getTargetPosition() * scalar);
            telemetry.addData("state", state);
//            telemetry.addData("pow", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.update();
        }
    }
}
