package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

@Autonomous(group = "Auto", name = "BlueDoubleSkystone")
public class BlueDoubleSkystone extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private VectorF skystonePosition = null;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-31.5,61.5,0));

        camera = new VuforiaLib_Skystone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();

        waitForStart();

        drive.releaseIntake();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(20.5)
                        .build()
        );



        Float yPos = null;
        VectorF fieldPosition = null;
        boolean a = false, b = false;


        while (true) {
            int c = 0;
            while(opModeIsActive()) {
                camera.loop(false);
                try {
                    yPos = camera.getFieldPosition().get(1);
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
                                .forward(7)
                                .build()
                );
                b = true;
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
        drive.setArmPos(0.85, 0.15);
        sleep(500);
        drive.turnSync(Math.toRadians(-90));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(70)
                        .build()
        );
        drive.toggleClaw();

//        if (b) {
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .forward(94)
//                            .build()
//            );
//        } else {
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .forward(86)
//                            .build()
//            );
//        }
//
//
//        drive.turnSync(Math.toRadians(90));
//
//
//        drive.setArmPos(1, 0);
//
//
//        sleep(1300);
//        drive.toggleClaw();
//        sleep(400);
//        drive.setArmPos(0.85, 0.15);
//        sleep(500);
//        drive.turnSync(Math.toRadians(-90));
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .back(70)
//                        .build()
//        );
//        drive.toggleClaw();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(30)
                        .build()
        );

        try {
            BufferedWriter fileOut = new BufferedWriter(new FileWriter(new File("../Data/StartingDirection.txt")));
            fileOut.write(String.valueOf(Math.toDegrees(drive.getRawExternalHeading())));
            fileOut.close();

        } catch (Exception e) {

        }
    }
}
