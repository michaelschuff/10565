package org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

@Autonomous(group = "Auto")
public class RedEverythingParkBridgeIntake extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private VectorF vuforiaPosition = null;
    //TODO: raise lift by 6.29921in
    //strafe to foundation position


    int SkystonePosition = -1;

    //phone offsets
    private double pxOffset = 0, pyOffset = 0, pzOffset = 0;

    //field Constants
    private double tileWidth = 23.5, botLength = 17.25, stoneWidth = 4, stoneLength = 8, stoneHeight = 4;

    //starting parameters
    private double startingAngle = Math.toRadians(0), startingX = -3 * tileWidth + 5.5 * stoneLength, startingY = -3 * tileWidth + botLength / 2 - pxOffset;

    @Override
    public void runOpMode() {

        initHardware();

        if (isStopRequested()) return;

        waitForStart();

        if (isStopRequested()) return;
        grabFirstSkystone();

        stackSkystone();

        grabSecondSkystone();

        stackSecondSkystone();

        grabFoundation();



        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");

            BufferedWriter fileOut = new BufferedWriter(new FileWriter(file));
            fileOut.write(Double.toString(drive.getRawExternalHeading() + Math.toRadians(startingAngle)));
            fileOut.close();

        } catch (Exception e) {

        }
    }

    private VectorF GetVuforia(int loops) {
        while(opModeIsActive()) {
            camera.loop(false);
            try {
                telemetry.addData("c",loops);
                telemetry.update();
                return camera.getFieldPosition();
            } catch (Exception e) {
                loops--;
                if (loops < 0) {
                    return null;
                }
            }
        }
        return null;
    }

    private void initHardware() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        camera = new VuforiaLib_Skystone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();

        drive.setClawGrabbing(false);
        drive.setFoundationGrabbing(false);
        drive.setPoseEstimate(new Pose2d(startingX, startingY, startingAngle));
    }

    private void grabFirstSkystone() {
        drive.releaseIntake();
        drive.setIntakePower(-1, -1);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(startingX, startingY + 20))
                        .build()
        );

        vuforiaPosition = GetVuforia(7500);
        if (vuforiaPosition != null) {
            if (vuforiaPosition.get(1) < 0) {//TODO: figure out which vuforia value to get and then change this code accordingly
                SkystonePosition = 1;
                //TODO: put where bot should be for middle skystone here (blue side furthest on left)
                drive.followTrajectorySync(
                        drive.trajectoryBuilder(new DriveConstraints(15, 15, 0, Math.PI, Math.PI, 0))
                                .splineTo(new Pose2d(-8.26772, 10.82677, Math.toRadians(-110)))
                                .build()
                );
            } else {
                SkystonePosition = 2;
                //TODO: put where bot should be for edge skystone here (blue side second from left)
                drive.followTrajectorySync(
                        drive.trajectoryBuilder(new DriveConstraints(15, 15, 0, Math.PI, Math.PI, 0))
                                .splineTo(new Pose2d(-8.26772, 10.82677, Math.toRadians(-110)))
                                .build()
                );
            }

        } else {
            //TODO: put where bot should be for inner skystone here (blue side third from left)
            drive.followTrajectorySync(
                    drive.trajectoryBuilder(new DriveConstraints(15, 15, 0, Math.PI, Math.PI, 0))
                            .splineTo(new Pose2d(-16.26772, 10.82677, Math.toRadians(-110)))
                            .build()
            );

        }
        drive.setClawGrabbing(true);
        drive.setIntakePower(0, 0);
    }

    private void stackSkystone() {
        //TODO: put position of bot where we stack on initial foundation position
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(40, 30, Math.toRadians(-90)))
                        .build()
        );

        //TODO: find correct delay for inbetween these
        drive.setArmPos(1, 0);
        drive.setClawGrabbing(false);
        drive.resetArm();
    }

    private void stackSecondSkystone() {
        //TODO: put position of bot where we stack on initial foundation position
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(40, 30, Math.toRadians(-90)))
                        .build()
        );

        //TODO: find correct delay for inbetween these
        //TODO: add pid onto lift and tune it and then set it to height we need to stack (about 4-6 in is good)
        drive.setArmPos(1, 0);
        drive.setClawGrabbing(false);
        drive.resetArm();
    }

    private void grabSecondSkystone() {
        drive.setIntakePower(-1, -1);
        //TODO: put position of bot where we grab corresponding skystone
        if (SkystonePosition == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(0, 45, Math.toRadians(15)))
                            .splineTo(new Pose2d(-47, 25, Math.toRadians(30)))
                            .build()
            );
        } if (SkystonePosition == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(0, 45, Math.toRadians(12)))
                            .splineTo(new Pose2d(-55, 25, Math.toRadians(30)))
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(0, 40, Math.toRadians(5)))
                            .splineTo(new Pose2d(-63, 25, Math.toRadians(30)))
                            .build()
            );
        }
        drive.setIntakePower(0, 0);

    }

    private void grabFoundation() {
        //TODO: Optimize this. we may not need to push all the way against wall, just get it in asap and go to bridge

        drive.toggleFoundation();
        sleep(500);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        //TODO: find actualy position to spline to
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(15)
                        .splineTo(new Pose2d(20, 5, Math.toRadians(90)))
                        .back(15)
                        .build()
        );
        drive.toggleFoundation();
        sleep(500);
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeLeft(7).build());
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(40).build());
    }
}