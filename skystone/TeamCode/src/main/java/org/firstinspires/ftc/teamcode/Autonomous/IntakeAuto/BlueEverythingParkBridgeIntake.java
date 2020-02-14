package org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Autonomous.SkyStoneFinder;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import kotlin.Unit;

@Config
@Autonomous(group = "Auto")
public class BlueEverythingParkBridgeIntake extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private PIDFController liftController;
    private final PIDCoefficients liftCoeffs = new PIDCoefficients(1.5, 1, 0.25);
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private int SkystonePosition = 1;

    private double lArm = 0.37, rArm = 0.63;

    private enum State {
        Initial,
        GrabFirstStone,
        StackFirstStone,
        GrabSecondStone,
        StackSecondStone,
        GrabFoundation,
        Park
    }


    private State current;

    @Override
    public void runOpMode() {
        if (isStopRequested()) {
            drive.setMotorPowers(0, 0, 0, 0);
            return;
        }
        initHardware();

        telemetry.addLine("Ready");
        telemetry.update();

        while(!isStarted()){
            try {
                SkystonePosition = SkyStoneFinder.detectSkystone(camera, false) + 1;
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

        current = State.Initial;

        while (!isStopRequested()) {
            switch (current) {
                case Initial:
                    drive.followTrajectory(generateGrabFirstSkystone());
                    current = State.GrabFirstStone;
                    break;
                case GrabFirstStone:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(generateMoveToFoundation1());
                        current = State.StackFirstStone;
                    }
                    break;
                case StackFirstStone:
                    if (!drive.isBusy() && liftController.getTargetPosition() != 1 / 1000.0 && liftController.getTargetPosition() != 150 / 1000.0) {
                        drive.setArmPos(0.8, 0.2);
                        sleep(2000);
                        drive.updateClawGrabbed();
                        drive.toggleClaw();
                        liftController.setTargetPosition(150 / 1000.0);
                    } else if (!drive.isBusy() && liftController.getTargetPosition() == 150 / 1000.0 && Math.abs(liftController.getLastError()) < 0.01) {
                        drive.toggleClaw();
                        liftController.setTargetPosition(0);
                        drive.setArmPos(lArm, rArm);
                        drive.followTrajectory(generateGrabSecondSkystone());
                        current = State.GrabSecondStone;
                    }
                    break;
                case GrabSecondStone:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(generateMoveToFoundation2());
                        current = State.StackSecondStone;
                        drive.setFoundation((short) 0);
                    }
                    break;
                case StackSecondStone:
                    if (!drive.isBusy() && liftController.getTargetPosition() != 1 / 1000.0 && liftController.getTargetPosition() != 200 / 1000.0 && liftController.getTargetPosition() != 350 / 1000.0) {
                        liftController.setTargetPosition(200 / 1000.0);
                    } else if (liftController.getTargetPosition() == 200 / 1000.0 && Math.abs(liftController.getLastError()) < 0.01) {
                        drive.setArmPos(0.8, 0.2);
                        sleep(2000);
                        drive.updateClawGrabbed();
                        drive.toggleClaw();
                        liftController.setTargetPosition(350 / 1000.0);
                    } else if (liftController.getTargetPosition() == 350 / 1000.0 && Math.abs(liftController.getLastError()) < 0.01) {
                        drive.setArmPos(lArm, rArm);
                        liftController.setTargetPosition(1 / 1000.0);
                    } else if (!drive.isBusy() && liftController.getTargetPosition() == 1 / 1000.0) {
                        drive.setFoundation((short) 2);
                        sleep(1000);
                        drive.followTrajectory(generateGrabFoundation());
                        current = State.GrabFoundation;
                    }
                    break;
                case GrabFoundation:
                    if (!drive.isBusy()) {
                        drive.setFoundation((short) 1);
                        sleep(500);
                        drive.followTrajectory(generatePark());
                        current = State.Park;
                    }
            }

            if (drive.getLiftPos() < 50) {
                drive.setLiftPower(kStatic(Math.abs(0.75 + drive.getLiftPos() / 200.0) * liftController.update(Math.abs(drive.getLiftPos()) / 1000.0)));
            } else {
                drive.setLiftPower(kStatic(liftController.update(drive.getLiftPos() / 1000.0)));
            }

            drive.update();
        }

        drive.setMotorPowers(0, 0, 0, 0);

        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");
            BufferedWriter fileOut = new BufferedWriter(new FileWriter(file));
            fileOut.write(Double.toString(Math.toRadians(180 - 90) - drive.getRawExternalHeading()));
            fileOut.close();
        } catch (Exception e) {
            telemetry.addLine("Could not save heading to file");
            telemetry.update();
        }
    }

    private void initHardware() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setClawGrabbing(false);
        drive.setArmPos(lArm, rArm);
        drive.setPoseEstimate(new Pose2d(-37.75, 61.75, Math.toRadians(-90)));
        liftController = new PIDFController(liftCoeffs);
        liftController.setInputBounds(0, 4000 / 1000.0);
        liftController.setOutputBounds(-1, 1);

        camera = new VuforiaLib_Skystone();
        camera.init(this, "");
        camera.start();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    private Trajectory generateGrabFirstSkystone() {
        if (SkystonePosition == 1) {
            return drive.trajectoryBuilder()
                    .lineTo(new Vector2d(-40, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-40, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-42, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build();
        } else if (SkystonePosition == 2) {
            return drive.trajectoryBuilder()
                    .lineTo(new Vector2d(-48, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-48, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-50, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build();
        } else {
            return drive.trajectoryBuilder()
                    .lineTo(new Vector2d(-56, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-56, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-58, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(-10)))
                    .build();
        }
    }

    private Trajectory generateMoveToFoundation1() {
        return drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                .addMarker(1.5, () -> {
                    drive.setIntakePower(0, 0);
                    drive.toggleClaw();
                    return Unit.INSTANCE;
                })
                .setReversed(false)
                .lineTo(new Vector2d(43.5, 40), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-90)))
                .lineTo(new Vector2d(43.5, 36), new LinearInterpolator(Math.toRadians(90), Math.toRadians(0)))
//                .splineTo(new Pose2d(43.5, 36, Math.toRadians(90)))
                .build();
    }

    private Trajectory generateGrabSecondSkystone() {
        if (SkystonePosition == 1) {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 42, Math.toRadians(180)))
                    .lineTo(new Vector2d(-40 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-40 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-44 + 24, 26), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build();
        } else if (SkystonePosition == 2) {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 42, Math.toRadians(180)))
                    .lineTo(new Vector2d(-48 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-48 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-52 + 24, 26), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build();
        } else {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 42, Math.toRadians(180)))
                    .lineTo(new Vector2d(-56 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-56 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-60 + 24, 26), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build();
        }
    }

    private Trajectory generateMoveToFoundation2() {
        return drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                .addMarker(1.5, () -> {
                    drive.setIntakePower(0, 0);
                    liftController.setTargetPosition(200 / 1000.0);
                    drive.toggleClaw();
                    return Unit.INSTANCE;
                })
                .setReversed(false)
                .lineTo(new Vector2d(43.5, 40), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-90)))
                .lineTo(new Vector2d(43.5, 36), new LinearInterpolator(Math.toRadians(90), Math.toRadians(0)))
//                .splineTo(new Pose2d(43.5, 36, Math.toRadians(90)))
                .build();
    }

    private Trajectory generateGrabFoundation() {
        return drive.trajectoryBuilder()
                .forward(20)
                .splineTo(new Pose2d(drive.getPoseEstimate().getX() - 10, drive.getPoseEstimate().getY() + 10, Math.toRadians(180)))
                .build();
    }

    private Trajectory generatePark() {
        return drive.trajectoryBuilder()
                .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                .build();
    }

    private double kStatic(double pow) {
        if (pow > 0) {
            return 0.25 + 0.75 * pow;
        } else if (pow < 0) {
            return pow;
        }
        return 0;
    }
}
