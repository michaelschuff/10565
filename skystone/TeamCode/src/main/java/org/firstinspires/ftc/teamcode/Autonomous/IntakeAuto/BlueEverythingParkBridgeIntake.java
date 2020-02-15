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
        MoveToFirstStone,
        InsertBotFirst,
        GrabStoneFirst,
        StackFirstStone,
        MoveToSecondStone,
        InsertBotSecond,
        GrabStoneSecond,
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
                    drive.followTrajectory(generateMoveToFirstStone());
                    current = State.MoveToFirstStone;
                    break;
                case MoveToFirstStone:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(generateGrabFirstStoneInsertBot());
                        current = State.InsertBotFirst;
                    }
                    break;
                case InsertBotFirst:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(generateGrabFirstStoneInsertBot());
                        current = State.GrabStoneFirst;
                    }
                case GrabStoneFirst:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(generateMoveToFoundation1());
                        current = State.StackFirstStone;
                    }
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
                        current = State.MoveToSecondStone;
                    }
                    break;
                case MoveToSecondStone:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(generateMoveToFoundation2());
                        current = State.InsertBotSecond;
                    }
                    break;
                case InsertBotSecond:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(generateMoveToFoundation2());
                        current = State.InsertBotSecond;
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
                drive.setLiftPower(kStatic(Math.abs(0.75 + drive.getLiftPos() / 200.0) * liftController.update(drive.getLiftPos() / 1000.0)));
            } else {
                drive.setLiftPower(kStatic(liftController.update(drive.getLiftPos() / 1000.0)));
            }

            drive.update();
        }

        drive.setMotorPowers(0, 0, 0, 0);

        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");
            BufferedWriter fileOut = new BufferedWriter(new FileWriter(file));
            fileOut.write(Double.toString(Math.toRadians(-90) - drive.getRawExternalHeading()));
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
        liftController.setInputBounds(-100 / 1000.0, 4000 / 1000.0);
        liftController.setOutputBounds(-1, 1);

        camera = new VuforiaLib_Skystone();
        camera.init(this, "");
        camera.start();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    private Trajectory generateMoveToFirstStone() {
        if (SkystonePosition == 1) {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-38, 35, Math.toRadians(-135)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .build();
        } else if (SkystonePosition == 2) {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-46, 35, Math.toRadians(-135)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .build();
        } else {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-54, 35, Math.toRadians(-135)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
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
//                .setReversed(false)
//                .lineTo(new Vector2d(43.5, 40), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-90)))
//                .lineTo(new Vector2d(43.5, 36), new LinearInterpolator(Math.toRadians(90), Math.toRadians(0)))
                .splineTo(new Pose2d(43.5, 33, Math.toRadians(90)))
                .build();
    }

    private Trajectory generateGrabSecondSkystone() {
        if (SkystonePosition == 1) {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 36, Math.toRadians(180)))
                    .splineTo(new Pose2d(-38 + 24, 35, Math.toRadians(-135)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .build();
        } else if (SkystonePosition == 2) {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 36, Math.toRadians(180)))
                    .splineTo(new Pose2d(-46 + 24, 35, Math.toRadians(-135)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .build();
        } else {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 36, Math.toRadians(180)))
                    .splineTo(new Pose2d(-54 + 24, 35, Math.toRadians(-135)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
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
//                .setReversed(false)
//                .lineTo(new Vector2d(43.5, 40), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-90)))
//                .lineTo(new Vector2d(43.5, 36), new LinearInterpolator(Math.toRadians(90), Math.toRadians(0)))
                .splineTo(new Pose2d(43.5, 36, Math.toRadians(90)))
                .build();
    }

    private Trajectory generateGrabFoundation() {
        return drive.trajectoryBuilder()
                .forward(20)
                .splineTo(new Pose2d(drive.getPoseEstimate().getX() - 10, drive.getPoseEstimate().getY() + 10, Math.toRadians(180)))
                .addMarker(() -> {
                    drive.setFoundation((short) 1);
                    return Unit.INSTANCE;
                })
                .build();
    }

    private Trajectory generateGrabFirstStoneInsertBot() {
        if (SkystonePosition == 1) {
            return drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-38, 30))
                    .strafeTo(new Vector2d(-42, 28))
                    .build();
        } else if (SkystonePosition == 2) {
            return drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-48, 30))
                    .strafeTo(new Vector2d(-50, 28))
                    .build();
        } else {
            return drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-56, 30))
                    .strafeTo(new Vector2d(-58, 28))
                    .build();
        }
    }

    private Trajectory generateGrabFirstStone() {
        if (SkystonePosition == 1) {
            return drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-42, 28))
                    .build();
        } else if (SkystonePosition == 2) {
            return drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-50, 28))
                    .build();
        } else {
            return drive.trajectoryBuilder()
                    .strafeTo(new Vector2d(-58, 28))
                    .build();
        }
    }

    private Trajectory generatePark() {
        return drive.trajectoryBuilder()
                .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                .build();
    }

    private double kStatic(double pow) {
        if (pow > 0) {
            return 0.1 + 0.9 * pow;
        } else if (pow < 0) {
            return pow;
        }
        return 0;
    }
}
