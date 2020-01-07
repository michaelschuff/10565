package org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
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


    public static int SkystonePosition = 1;


    //field Constants
    private double tileWidth = 23.5, botWidth = 17.125, botLength = 17.1875;





    //starting position
    private static double startingAngle = -90, startingX = -24 - 17.125 / 2, startingY = 70.5 - 17.1875 / 2;

    public static double liftPower = .5;

    public static double strafex = 44.5, strafey = 45, angle = 135, splinex = 0, spliney = 50;
    @Override
    public void runOpMode() {

        if (isStopRequested()) return;
        initHardware();
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        telemetry.clear();

        if (isStopRequested()) return;
//        grabFirstSkystone();
//
//        stackSkystone(false);
//
//        grabSecondSkystone();
//
//        stackSkystone(true);

        grabFoundation();



//        try {
//            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");
//
//            BufferedWriter fileOut = new BufferedWriter(new FileWriter(file));
//            fileOut.write(Double.toString(drive.getRawExternalHeading() + Math.toRadians(startingAngle)));
//            fileOut.close();
//
//        } catch (Exception e) {
//
//        }
    }

    private void stackSkystone(boolean second) {
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 40, Math.toRadians(185)))
                        .splineTo(new Pose2d(35, 42, Math.toRadians(180)))
                        .build()
        );

        drive.turnSync(Math.toRadians(-90));

        vuforiaPosition = GetVuforia();
        vuforiaHeading = GetVuforiaH();
        drive.setPoseEstimate(new Pose2d(mmToInches(vuforiaPosition.get(0)), mmToInches(vuforiaPosition.get(1)), Math.toRadians(vuforiaHeading)));
//        if (!second) {
//            drive.setPoseEstimate(new Pose2d(mmToInches(vuforiaPosition.get(0)), mmToInches(vuforiaPosition.get(1)), Math.toRadians(vuforiaHeading)));
//        } else {
//            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(vuforiaHeading)));
//        }

        drive.turnSync(Math.toRadians(90 - vuforiaHeading));
        if (second) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(42.5, 33))
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(42.5, 33))
                            .build()
            );
        }

        if (second) {
            drive.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.setArmPos(0.33, 0.67);
            drive.setLiftPos(500);
            drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.lift.setPower(liftPower);
            sleep((long) (500));
            drive.setArmPos(0.8, 0.2);
            sleep((long) (1500));
            drive.setClawGrabbing(false);
            sleep(100);
            drive.setLiftPos(800);
            drive.lift.setPower(liftPower);
            sleep(500);
            drive.setArmPos(0.33, 0.67);
            drive.setFoundation((short) 2);
            sleep(100);
            drive.setLiftPos(0);
            drive.lift.setPower(-liftPower);

        } else {
            drive.setArmPos(0.8, 0.2);
            sleep(900);
            drive.setClawGrabbing(false);
            drive.setLiftPos(300);
            drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.lift.setPower(liftPower);
            sleep(500);
            drive.setArmPos(0.33, 0.67);
            drive.setLiftPos(0);
            drive.lift.setPower(-liftPower);
        }
    }

    private VectorF CheckVuforia(int loops) {
        while(opModeIsActive()) {
            camera.loop(false);
            try {
                VectorF a = camera.getFieldPosition();
                telemetry.addData("c",loops);
                telemetry.addData("x", a.get(0));
                telemetry.addData("y", a.get(1));
                telemetry.addData("z", a.get(2));
                telemetry.update();
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
        while(!isStopRequested()) {
            camera.loop(false);
            try {
                VectorF a = camera.getFieldPosition();
                if (a != null) {
                    return a;
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

    private void initHardware() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        camera = new VuforiaLib_Skystone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();

        drive.setClawGrabbing(false);
        drive.setPoseEstimate(new Pose2d(startingX,  startingY, Math.toRadians(startingAngle)));
    }

    private void grabFirstSkystone() {
        drive.setArmPos(0.33, 0.67);
        drive.setIntakePower(-.75, -.75);
        SetIntake timer = new SetIntake(8, 0, 0);

        if (SkystonePosition == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-16, 38, Math.toRadians(230)))
                            .build()
            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(-16,32))
                            .build()
            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(3)
                            .build()
            );
        } else if (SkystonePosition == 2) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-24, 38, Math.toRadians(230)))
                            .build()
            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(-24,32))
                            .build()
            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(3)
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-32, 38, Math.toRadians(230)))
                            .build()
            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(-32,32))
                            .build()
            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(3)
                            .build()
            );
        }

    }

    private void grabSecondSkystone() {

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(35, 36, Math.toRadians(180)))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(-40.5, 38))
                        .build()
        );
        drive.setIntakePower(-.75, -.75);
        SetIntake timer = new SetIntake(6, 0, 0);

        vuforiaPosition = GetVuforia();
        vuforiaHeading = GetVuforiaH();


        drive.setPoseEstimate(new Pose2d(mmToInches(vuforiaPosition.get(0)), mmToInches(vuforiaPosition.get(1)), Math.toRadians(vuforiaHeading)));

        if (SkystonePosition == 1) {
            drive.turnSync(Math.toRadians(40));
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(splinex, spliney, Math.toRadians(230)))
//                            .build()
//            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(-40.5, 32))
                            .build()
            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(3)
                            .build()
            );
        } else if (SkystonePosition == 2) {

        } else {

        }

    }

    private void grabFoundation() {
        drive.setPoseEstimate(new Pose2d(44.5, 33, Math.toRadians(90)));
        drive.setFoundation((short) 2);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(strafex, strafey))
                        .build()
        );

//        vuforiaPosition = GetVuforia();
//        vuforiaHeading = GetVuforiaH();
//        drive.setPoseEstimate(new Pose2d(mmToInches(vuforiaPosition.get(0)), mmToInches(vuforiaPosition.get(1)), Math.toRadians(vuforiaHeading)));

//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(20, 5, Math.toRadians(180)))
//                        .build()
//        );

        drive.turnSync(Math.toRadians(angle));

        drive.setFoundation((short) 0);
        sleep(500);

        drive.setPoseEstimate(new Pose2d(30, 52, Math.toRadians(180)));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(splinex, spliney, Math.toRadians(180)))
                        .build()
        );
    }

    public class SetIntake {
        Timer timer;
        double p1, p2;

        public SetIntake(int seconds, double p1, double p2) {
            timer = new Timer();
            timer.schedule(new RemindTask(), seconds*1000);
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
}