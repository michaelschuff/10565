package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.TaskThread;
import org.firstinspires.ftc.teamcode.util.ThreadLinearOpMode;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

@Autonomous(group = "Auto")
public class RedSkystones extends ThreadLinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private VectorF skystonePosition = null;

    @Override
    public void runMainOpMode() {
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                camera.loop(false);
                try {
                    skystonePosition = camera.getFieldPosition();
                } catch (Exception e) {

                }
            }
        }));

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-31.5,61.5,0));

        camera = new VuforiaLib_Skystone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();

        waitForStart();

        drive.releaseIntake();
        drive.setIntakePower(-1, -1);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .strafeRight(19)
                .build()
        );

//        try {
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(drive.getPoseEstimate().getX() + skystonePosition.get(1), drive.getPoseEstimate().getY() + skystonePosition.get(0), Math.toRadians(-135)))
//                            .build()
//            );
//        } catch (Exception e) {
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(drive.getPoseEstimate().getX() - 14, drive.getPoseEstimate().getY() - 20, Math.toRadians(-135)))
//                            .build()
//            );
//        }


//        drive.turnSync(135);//TODO: turn to face 90
//
//        drive.toggleClaw();
//
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .forward(15)
//                        .strafeTo(new Vector2d(drive.getPoseEstimate().getY(), 19))
//                        .build()
//        );
//
//        drive.toggleArm();



        while(!isStopRequested()) {
            telemetry.addData("skystonePosition", skystonePosition);
            telemetry.update();
        }



        try {
            BufferedWriter fileOut = new BufferedWriter(new FileWriter(new File("../Data/StartingDirection.txt")));
            fileOut.write(String.valueOf(Math.toDegrees(drive.getRawExternalHeading())));
            fileOut.close();

        } catch (Exception e) {

        }
    }
}
