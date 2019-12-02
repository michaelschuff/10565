package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

@Config
@Autonomous(group = "Auto")
public class BlueGrabSkystone extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private float skystonePosition = 0;
    public static double sideways = 20;
    public static double forwards = 36;
    public static double angle = -45;
    public static double secondforward = .125;
    //tilesize = 22.875
    //meshsize = .75

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-31.5,61.5,0));

        camera = new VuforiaLib_Skystone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();

        waitForStart();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeRight(forwards)
                .build());

        int c = 0;
        while (!isStopRequested()) {
            camera.loop(true);
            try {
                skystonePosition = camera.getFieldPosition().get(1) / 25.4f;
                break;
            } catch (Exception e) {
                c++;
            }

            if (c > 10000) {
                drive.followTrajectorySync(drive.trajectoryBuilder()
                        .back(2)
                        .build());
                c = 0;
            }
        }
        telemetry.addData("skystone", skystonePosition);
        telemetry.update();

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .back(skystonePosition)
                .build()
        );

        drive.turnSync(Math.toRadians(angle));


        drive.followTrajectorySync(drive.trajectoryBuilder()
                .strafeRight(sideways)
                .build());

        drive.ToggleIntake();

        drive.setMotorPowers(secondforward, secondforward, secondforward, secondforward);
        sleep(3000);
        drive.setMotorPowers(0, 0, 0, 0);



    }
}
