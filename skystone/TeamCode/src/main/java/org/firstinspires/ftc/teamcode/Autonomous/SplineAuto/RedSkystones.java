package org.firstinspires.ftc.teamcode.Autonomous.SplineAuto;

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
import java.util.Vector;

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
                    if (c > 1000) {
                        break;
                    }
                }
            }
            if (a) {
                break;
            } else {
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .forward(8)
                                .build()
                );
            }

        }

        if (yPos > 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(yPos / 25.4)
                            .build()
            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(-yPos / 25.4)
                            .build()
            );
        }



        while(!isStopRequested()) {
            telemetry.addData("yPos",yPos);
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
