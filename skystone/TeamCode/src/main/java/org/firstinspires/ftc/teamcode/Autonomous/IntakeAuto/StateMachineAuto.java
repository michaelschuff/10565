package org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

import java.util.Stack;

import kotlin.Unit;

@Config
@Autonomous(group = "Auto")
public class StateMachineAuto extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private PIDFController liftController;
    public static PIDCoefficients liftCoeffs = new PIDCoefficients(3, 0.05, 0.2);
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static boolean runAuto = false;

    private int SkystonePosition = 1;

    private boolean updateLift;

    private enum State {
        Initial,
        GrabFirstStone,
        StackStone,
        GrabSecondStone,
        GrabFoundation
    }


    public State current;

    @Override
    public void runOpMode() {
        if (isStopRequested()) {
            drive.setMotorPowers(0, 0, 0, 0);
            return;
        }
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setClawGrabbing(false);
        drive.setPoseEstimate(new Pose2d(-37.75, 61.75, Math.toRadians(-90)));
        liftController = new PIDFController(liftCoeffs);
        liftController.setInputBounds(0, 4000);
        liftController.setOutputBounds(-0.75, 0.75);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        if (isStopRequested()) {
            drive.setMotorPowers(0, 0, 0, 0);
            return;
        }

        current = State.Initial;

        if (runAuto) {
            updateLift = false;
            Trajectory GrabFirstStone = generateGrabFirstSkystone();
            Trajectory GrabSecondStone = generateGrabSecondSkystone();


            Trajectory StackFirstStone = drive.trajectoryBuilder()
                    .addMarker(1, () -> {
                        drive.setIntakePower(0, 0);
                        return Unit.INSTANCE;
                    })
                    .reverse()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .addMarker(() -> {
                        liftController.setTargetPosition(1000);
                        updateLift = true;
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(43.5, 33), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-90)))
                    .build();

            while (!isStopRequested()) {
                switch (current) {
                    case Initial:
                        drive.followTrajectory(GrabFirstStone);
                        current = State.GrabFirstStone;
                        break;
                    case GrabFirstStone:
                        if (!drive.isBusy()) {
                            drive.followTrajectory(StackFirstStone);
                            current = State.StackStone;
                        }
                        break;
                    case StackStone:
                        if (!drive.isBusy()) {
                            drive.followTrajectory(GrabSecondStone);
                            current = State.GrabSecondStone;
                        }
                        break;
                }
                if (updateLift) {
                    drive.setLiftPower(liftController.update(drive.getLiftPos(), drive.getLiftVel()));
                }
            }
        } else {
            TelemetryPacket fastTelem = new TelemetryPacket();
            updateLift = true;
            liftController.setTargetPosition(1);
            while (!isStopRequested()) {
                switch (current) {
                    case Initial:
                            drive.followTrajectory(drive.trajectoryBuilder(new DriveConstraints(10, 10, 10, Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)))
                                    .forward(100)
                                    .build()
                            );
                            current = State.GrabFirstStone;
                            break;
                }
                if (updateLift) {
                    drive.setLiftPower(liftController.update(drive.getLiftPos() / 1000.0, drive.getLiftVel() / 1000.0));
                }
                fastTelem.put("liftPos", drive.getLiftPos());
                fastTelem.put("targetPos", liftController.getTargetPosition() * 1000);
                fastTelem.put("error", 1000 * liftController.getTargetPosition() - drive.getLiftPos());
                dashboard.sendTelemetryPacket(fastTelem);
                drive.update();
            }
        }

        drive.setMotorPowers(0, 0, 0, 0);
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

    private Trajectory generateGrabSecondSkystone() {
        if (SkystonePosition == 1) {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-40 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-40 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-42 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build();
        } else if (SkystonePosition == 2) {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-48 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-48 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-50 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build();
        } else {
            return drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-56 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-56 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-58 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build();
        }
    }
}
