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
public class BlueEverythingParkBridgeIntake extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private VectorF skystonePosition = null;

    //phone offsets
    private double pxOffset = 0, pyOffset = 0, pzOffset = 0;

    //field Constants
    private double tileWidth = 23.5, botLength = 17.25, stoneWidth = 4, stoneLength = 8, stoneHeight = 4;

    //starting parameters
    private double startingAngle = Math.toRadians(0), startingX = -3 * tileWidth + 5.5 * stoneLength, startingY = -3 * tileWidth + botLength / 2 - pxOffset;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        camera = new VuforiaLib_Skystone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();

        drive.setClawGrabbing(false);
        drive.setPoseEstimate(new Pose2d(startingX, startingY, startingAngle));

        waitForStart();

        drive.releaseIntake();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(startingX, startingY - 20))
                        .build()
        );



        Float yPos = null;
        VectorF fieldPosition = null;
        boolean a = false;


        while (true) {
            int c = 0;
            while(opModeIsActive()) {
                camera.loop(false);
                try {
                    fieldPosition = camera.getFieldPosition();
                    yPos = fieldPosition.get(1);
                    a = true;
                    break;
                } catch (Exception e) {
                    c++;
                    if (c > 7500) {
                        break;
                    }
                }
            }
            if (a) {
                break;
            } else {
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .back(7)
                                .build()
                );
            }

        }


        if (yPos / 25.4 - 1 > 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder(new DriveConstraints(15, 15, 0, Math.PI, Math.PI, 0))
                            .back(yPos / 25.4 - 1)
                            .build()
            );

        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder(new DriveConstraints(15, 15, 0, Math.PI, Math.PI, 0))
                            .forward(1 - yPos / 25.4)
                            .build()
            );
        }

        drive.turnSync(Math.toRadians(90));


        drive.setArmPos(1, 0);


        sleep(1300);
        drive.toggleClaw();
        sleep(400);
        drive.setArmPos(0.8, 0.2);
        sleep(500);
        drive.turnSync(Math.toRadians(-90));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(2 * tileWidth, drive.getPoseEstimate().getY()))
                        .build()
        );

        drive.turnSync(Math.toRadians(90));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(10)
                        .build()
        );
        drive.toggleClaw();

        grabFoundation();

        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");

            BufferedWriter fileOut = new BufferedWriter(new FileWriter(file));
            fileOut.write(Double.toString(drive.getRawExternalHeading() + Math.toRadians(startingAngle)));
            fileOut.close();

        } catch (Exception e) {

        }
    }

    private void grabFoundation() {
        drive.toggleFoundation();
        sleep(500);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

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
        drive.releaseIntake();
    }
}