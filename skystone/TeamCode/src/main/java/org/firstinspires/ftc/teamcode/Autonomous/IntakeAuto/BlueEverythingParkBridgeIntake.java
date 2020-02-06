package org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Config
@Autonomous(group = "Auto")
public class BlueEverythingParkBridgeIntake extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private VectorF vuforiaPosition = null;
    private float vuforiaHeading;


    public static int SkystonePosition = 2;

    //starting position
    private static double startingAngle = -90, startingX = -23.25 * 2 + 17.5 / 2, startingY = 70.5 - 17.5 / 2;

    public static double armPos = .37;

    public static boolean grabFirst = true, stackFirst = true, grabSecond = true, stackSecond = true, grabThird = false, grabFoundation = false;

    @Override
    public void runOpMode() {

        if (isStopRequested()) return;
        initHardware();
        telemetry.addLine("Ready");
        telemetry.update();
        while(!isStarted()){
            try {
//                SkystonePosition = SkyStoneFinder.detectSkystone(camera, false) + 1;
//                telemetry.addData("Skystone", SkystonePosition);
//                telemetry.update();
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

        if (grabThird) {
            grabThirdSkystone();
        }

        if (grabFoundation) {
            grabFoundation();
        }



        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");

            BufferedWriter fileOut = new BufferedWriter(new FileWriter(file));
            fileOut.write(Double.toString(drive.getRawExternalHeading() - Math.toRadians(startingAngle)));
            fileOut.close();

        } catch (Exception e) {
            telemetry.addLine("Could not save heading to file");
            telemetry.update();
        }
        while(!isStopRequested()) {

        }
    }

    private void stackSkystone(boolean second) {
        drive.followTrajectorySync(
            drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                .splineTo(new Pose2d(43.5, 39, Math.toRadians(90)))
//                .setReversed(false)
//                .lineTo(new Vector2d(35, 40), new LinearInterpolator(Math.toRadians(180), Math.toRadians(-90)))
//                .lineTo(new Vector2d(43.5, 39), new LinearInterpolator(Math.toRadians(90), Math.toRadians(0)))
                .build()
        );
//        if (second) {
//            drive.setArmPos(armPos, 1 - armPos);
//            drive.setLiftPos(500);
//            drive.setLiftPower(liftPower);
//            sleep((long) (500));
//            drive.setArmPos(stackArmPos, 1 - stackArmPos);
//            sleep((long) (1500));
//            drive.setClawGrabbing(false);
//            sleep(100);
//            drive.setLiftPos(800);
//            drive.setLiftPower(liftPower);
//            sleep(500);
//            drive.setArmPos(armPos, 1 - armPos);
//            drive.setFoundation((short) 2);
//            sleep(100);
//            drive.setLiftPos(0);
//            drive.setLiftPower(-liftPower);
//
//        } else {
//            drive.setArmPos(stackArmPos, 1 - stackArmPos);
//            sleep(900);
//            drive.setClawGrabbing(false);
//            drive.setLiftPos(300);
//            drive.setLiftPower(liftPower);
//            sleep(600);
//            drive.setArmPos(armPos, 1 - armPos);
//            drive.setLiftPos(0);
//            drive.setLiftPower(-liftPower);
//        }
    }

    private void grabFirstSkystone() {
        drive.setArmPos(armPos, 1 - armPos);
        drive.setClawGrabbing(false);
        drive.setIntakePower(-1, -1);

        if (SkystonePosition == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-40, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-40, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-42, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else if (SkystonePosition == 2) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-48, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-48, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-50, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-56, 38), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(-45)))
                            .lineTo(new Vector2d(-56, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-58, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(-10)))
                            .build()
            );
        }
        SetIntake setIntake = new SetIntake(1, 0, 0);

    }

    private void grabSecondSkystone() {
        drive.setIntakePower(-1, -1);

        if (SkystonePosition == 1) {
            drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-40 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .lineTo(new Vector2d(-40 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-42 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build()
            );
        } else if (SkystonePosition == 2) {
            drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-48 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .lineTo(new Vector2d(-48 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-50 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build()
            );
        } else {
            drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                    .lineTo(new Vector2d(-56 + 24, 38), new LinearInterpolator(Math.toRadians(180), Math.toRadians(45)))
                    .lineTo(new Vector2d(-56 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .lineTo(new Vector2d(-58 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                    .build()
            );
        }

        SetIntake setIntake = new SetIntake(1, 0, 0);

    }

    private void grabThirdSkystone() {
        drive.setIntakePower(-1, -1);

        if (SkystonePosition == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                            .lineTo(new Vector2d(-39.5 + 24, 38), new LinearInterpolator(Math.toRadians(90), Math.toRadians(225)))
                            .lineTo(new Vector2d(-39.5 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-42 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else if (SkystonePosition == 2) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                            .lineTo(new Vector2d(-48 + 24, 38), new LinearInterpolator(Math.toRadians(90), Math.toRadians(135)))
                            .lineTo(new Vector2d(-48 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-50 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                            .lineTo(new Vector2d(-56 + 24, 38), new LinearInterpolator(Math.toRadians(90), Math.toRadians(135)))
                            .lineTo(new Vector2d(-56 + 24, 30), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .lineTo(new Vector2d(-58 + 24, 28), new LinearInterpolator(Math.toRadians(-135), Math.toRadians(0)))
                            .build()
            );
        }

        SetIntake setIntake = new SetIntake(1, 0, 0);

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
//        drive.setLiftPos(0);
//        drive.bLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.fLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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



    public class SetIntake {
        Timer timer;
        double p1, p2;

        public SetIntake(double seconds, double p1, double p2) {
            timer = new Timer();
            timer.schedule(new RemindTask(), (int) seconds*1000);
            this.p1 = p1;
            this.p2 = p2;
        }

        class RemindTask extends TimerTask {
            public void run() {
                drive.setIntakePower(p1, p2);
                drive.setClawGrabbing(true);
                timer.cancel(); //Terminate the timer thread
            }
        }
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
