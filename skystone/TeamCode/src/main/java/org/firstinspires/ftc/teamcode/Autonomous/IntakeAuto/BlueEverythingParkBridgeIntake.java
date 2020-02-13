package org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Autonomous.SkyStoneFinder;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.Timer;
import java.util.TimerTask;

import kotlin.Unit;

@Config
@Autonomous(group = "Auto")
public class BlueEverythingParkBridgeIntake extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private PIDFController liftController;
    public static PIDCoefficients liftCoeffs = new PIDCoefficients(0, 0, 0);
    public static double kV = 0;
    private VectorF vuforiaPosition = null;
    private float vuforiaHeading;


    public static int SkystonePosition = 1;

    //starting position
    private static double startingAngle = -90, startingX = -23.25 * 2 + 17.5 / 2, startingY = 70.5 - 17.5 / 2;

    public static double armPos = .37;

    public static boolean grabFirst = true, stackFirst = true, grabSecond = true, stackSecond = true, grabFoundation = false;

    @Override
    public void runOpMode() {

        if (isStopRequested()) return;
        initHardware();
        liftController = new PIDFController(liftCoeffs, kV);
        liftController.setInputBounds(0, 4000);
        liftController.setOutputBounds(-1, 1);
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

        if (isStopRequested()) return;


        if (grabFirst) {
            grabFirstSkystone();
        }

        if (stackFirst) {
            stackSkystone(false);
        }

        if (grabSecond) {
            grabSecondSkystone();
        }

        if (stackSecond) {
            stackSkystone(true);
        }

        if (grabFoundation) {
            grabFoundation();
        }



        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");

            BufferedWriter fileOut = new BufferedWriter(new FileWriter(file));
            fileOut.write(Double.toString(Math.toRadians(startingAngle) - drive.getRawExternalHeading()));
            fileOut.close();

        } catch (Exception e) {
            telemetry.addLine("Could not save heading to file");
            telemetry.update();
        }
        while(!isStopRequested()) {

        }
    }

    private void stackSkystone(boolean second) {
        if (!second) {
            drive.setFoundation((short) 1);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .addMarker(1, () -> {
                                drive.setIntakePower(0, 0);
                                return Unit.INSTANCE;
                            })
                            .reverse()
                            .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
//                            .splineTo(new Pose2d(43.5, 33, Math.toRadians(90)))
                            .setReversed(false)
                            .lineTo(new Vector2d(43.5, 34), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-90)))
                            .build()
            );
//            drive.setFoundation((short) 2);
            drive.setArmPos(0.8, 0.2);
//            sleep(250);
//            drive.turnSync(Math.toRadians(45));
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(28, 40, Math.toRadians(180)))
//                            .build()
//            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .addMarker(1, () -> {
                                drive.setIntakePower(0, 0);
                                return Unit.INSTANCE;
                            })
                            .reverse()
                            .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
//                            .addMarker(.5, () -> {
//                                drive.setArmPos(0.7, 0.3);
//                                drive.setLiftPos(500);
//                                drive.setLiftPower(0.75);
//                                return Unit.INSTANCE;
//                            })
//                            .splineTo(new Pose2d(43.5, 33, Math.toRadians(90)))
                            .setReversed(false)
                            .lineTo(new Vector2d(43.5, 34), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-90)))
                            .build()
            );
        }
    }

    private void grabFirstSkystone() {
        drive.setArmPos(armPos, 1 - armPos);
        drive.setClawGrabbing(false);

        if (SkystonePosition == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-40, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .addMarker(() -> {
                                drive.setIntakePower(-1, -1);
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(-40, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-42, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else if (SkystonePosition == 2) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-48, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .addMarker(() -> {
                                drive.setIntakePower(-1, -1);
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(-48, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-50, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-56, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .addMarker(() -> {
                                drive.setIntakePower(-1, -1);
                                return Unit.INSTANCE;
                            })
                            .lineTo(new Vector2d(-56, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-58, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(-10)))
                            .build()
            );
        }

    }

    private void grabSecondSkystone() {
        drive.setIntakePower(-1, -1);

        if (SkystonePosition == 1) {
            drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-40 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
//                    .lineTo(new Vector2d(-40 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
//                    .lineTo(new Vector2d(-42 + 24, 26), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-40 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-42 + 24, 26), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else if (SkystonePosition == 2) {
            drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-48 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-48 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-50 + 24, 26), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build()
            );
        } else {
            drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-56 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .addMarker(() -> {
                        drive.setIntakePower(-1, -1);
                        return Unit.INSTANCE;
                    })
                    .lineTo(new Vector2d(-56 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-58 + 24, 26), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build()
            );
        }

    }

    private void grabFoundation() {
        drive.setFoundation((short) 2);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(43, 32))
                        .lineTo(new Vector2d(38, 54), new LinearInterpolator(Math.toRadians(90), Math.toRadians(135)))
                        .lineTo(new Vector2d(53, 32))
                        .build()
        );
        drive.setFoundation((short) 0);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 36, Math.toRadians(180)))
                        .build()
        );
    }

    private void initHardware() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        camera = new VuforiaLib_Skystone();
        camera.init(this, "");
        camera.start();

        drive.setClawGrabbing(false);
        drive.setPoseEstimate(new Pose2d(startingX,  startingY, Math.toRadians(startingAngle)));
    }

    private VectorF CheckVuforia(int loops) {
        while(opModeIsActive()) {
            camera.loop(false);
            try {
                VectorF a = camera.getFieldPosition();
                return a;
            } catch (Exception e) {
                loops--;
                if (loops < 0) {
                    return null;
                }
            }
        }
        return null;
    }

    private VectorF GetVuforia() {
        float maxError = 12;
        while(!isStopRequested()) {
            camera.loop(false);
            try {
                VectorF a = camera.getFieldPosition();
                if (a != null) {
                    return a;
//                    double b = Math.sqrt(Math.pow(drive.getPoseEstimate().getX() - mmToInches(a.get(0)), 2) + Math.pow(drive.getPoseEstimate().getY() - mmToInches(a.get(1)), 2));
//
//                    if (b < maxError) {
//                    }
                }

            } catch (Exception e) {

            }
        }
        return new VectorF(0, 0, 0);
    }

    private float GetVuforiaH() {
        int c = 0;
        float a = 0f;
        while(!isStopRequested() && c < 5) {
            camera.loop(false);
            try {
                a += camera.getOrientation().thirdAngle;
                c++;
            } catch (Exception e) {

            }
        }
        return a / c;
    }

    private double mmToInches(double mm) {
        return mm / 25.4;
    }

    private boolean tooFast(){
        if(drive.getMaxMotorVelocity() > DriveConstants.BASE_CONSTRAINTS.maxVel){
            this.stop();
            return true;
        }
        return false;
    }
}
