package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.TaskThread;
import org.firstinspires.ftc.teamcode.util.ThreadLinearOpMode;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

@Config
@Autonomous(group = "Auto")
public class BlueGrabSkystone extends ThreadLinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private Float skystonePosition = null;



    @Override
    public void runMainOpMode() {
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                camera.loop(true);
                try {
                    skystonePosition = camera.getFieldPosition().get(1) / 25.4f;
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

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeRight(36)
                .build());

        while (!isStopRequested()) {

            if (skystonePosition == null) {
                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .back(2)
                        .build());
            } else {
                break;
            }
        }

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(skystonePosition)
                .build()
        );

        drive.turnSync(Math.toRadians(-45));


        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeRight(20)
                .build());

        drive.setIntakePower(.5,.5);

        drive.setMotorPowers(.125, .125, .125, .125);
        sleep(2000);
        drive.setMotorPowers(0, 0, 0, 0);



    }
}
